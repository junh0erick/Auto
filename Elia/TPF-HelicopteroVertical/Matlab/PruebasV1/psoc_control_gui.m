function psoc_control_gui()
% GUI UARTP: m/i/c/t/s + plot float32 u,y
% Firmware esperado (TU VERSION ACTUAL):
%   m: send 'm' -> 'R' -> 4 bytes [impl,0,0,0] -> 'K'
%   c: send 'c' -> 'R' -> 64 bytes (16 float32) -> 'K'
%       donde:
%         c[14] = N (single)        (PSoC: uint16(N) & 0xFFFF)
%         c[15] = period_ticks      (PSoC: uint16(period) & 0xFFFF)
%   t: send 't' -> 'S' -> 64 bytes -> 'K'
%   i: send 'i' -> 'R' -> float32 ref/u0 -> 'K'   (también durante CONTROL)
%   s: send 's' -> 'K'  (en CONTROL por ISR)
%
% Streaming: float32 alternando u luego y (4B cada uno), cada N muestras.

    S = struct();
    S.sp = [];
    S.isConnected = false;

    S.expectU = true;
    S.uTemp = 0;

    S.maxPoints = 6000;
    S.k = 0;
    S.t = nan(S.maxPoints,1);
    S.u = nan(S.maxPoints,1);
    S.y = nan(S.maxPoints,1);

    % Config actual (para estimar t)
    S.fclk = 1e6;
    S.Fs   = 1000;
    S.period_ticks = 1500;  % ticks "tal cual" (lo que le pasas a WritePeriod)
    S.Nval  = 20;
    S.ts_est = 0.02; % N*Ts

    % -------------------- UI --------------------
    fig = uifigure('Name','PSoC Control GUI (UARTP)','Position',[60 60 1280 720]);

    ax = uiaxes(fig,'Position',[20 220 820 480]);
    ax.XGrid = 'on'; ax.YGrid = 'on';
    title(ax,'Streaming u,y'); xlabel(ax,'t (s)'); ylabel(ax,'value');
    hU = plot(ax, nan, nan); hold(ax,'on');
    hY = plot(ax, nan, nan); hold(ax,'off');
    legend(ax, {'u','y'}, 'Location','northwest');

    % Log
    txtLog = uitextarea(fig,'Editable','off','Position',[20 20 820 180]);
    txtLog.Value = strings(0,1);

    % Connection
    pConn = uipanel(fig,'Title','Connection','Position',[860 610 400 90]);
    uilabel(pConn,'Text','COM:','Position',[10 35 35 22]);
    edtCom = uieditfield(pConn,'text','Value','COM9','Position',[50 35 90 22]);
    edtBaud = uieditfield(pConn,'numeric','Value',115200,'Limits',[1200 2000000],'Position',[150 35 90 22]);
    uilabel(pConn,'Text','baud','Position',[245 35 40 22]);

    btnConnect = uibutton(pConn,'Text','Connect','Position',[10 5 90 24],...
        'ButtonPushedFcn',@onConnectToggle);
    lblStat = uilabel(pConn,'Text','DISCONNECTED','Position',[110 5 280 24]);

    % Mode
    pMode = uipanel(fig,'Title','Mode','Position',[860 430 400 170]);
    ddType = uidropdown(pMode,'Items',{'TF','SS','Open-loop'},'Position',[10 120 120 24],...
        'ValueChangedFcn',@(~,~)refreshVisibility());
    ddObserver = uidropdown(pMode,'Items',{'Predictor','Actual'},'Position',[150 120 120 24],...
        'Tooltip','Only applies for SS');
    cbIntegrator = uicheckbox(pMode,'Text','Integrator','Position',[290 120 110 24]);

    cbStream = uicheckbox(pMode,'Text','Stream (N>0)','Value',true,'Position',[10 85 120 24]);
    uilabel(pMode,'Text','N','Position',[140 85 20 24]);
    edtN = uieditfield(pMode,'numeric','Limits',[0 65535],'RoundFractionalValues','on','Value',20,'Position',[165 85 90 24],...
        'ValueChangedFcn',@(~,~)updateTsEstimate());

    btnSendMode = uibutton(pMode,'Text','Send Mode (m)','Position',[270 85 120 24],...
        'ButtonPushedFcn',@onSendMode);

    btnSendCoeffs = uibutton(pMode,'Text','Send Coeffs (c)','Position',[10 45 140 28],...
        'ButtonPushedFcn',@onSendCoeffs);
    cbVerify = uicheckbox(pMode,'Text','Verify (t)','Value',true,'Position',[160 50 120 24]);

    % Reference / control
    pRef = uipanel(fig,'Title','Control','Position',[860 320 400 100]);
    uilabel(pRef,'Text','ref','Position',[10 45 60 22]);
    edtU0 = uieditfield(pRef,'numeric','Value',0.25,'Position',[80 40 100 26]);
    btnStart = uibutton(pRef,'Text','Send ref (i)','Position',[200 40 90 26],...
        'ButtonPushedFcn',@onStart);
    btnStop = uibutton(pRef,'Text','Stop (s)','Position',[300 40 80 26],...
        'ButtonPushedFcn',@onStop);

    % Sampling (solo calcula period_ticks, no manda 'p')
    pSamp = uipanel(fig,'Title','Sampling (embedded in c[15])','Position',[860 210 400 100]);
    uilabel(pSamp,'Text','Timer clk (Hz):','Position',[10 55 100 22]);
    edtFclk = uieditfield(pSamp,'numeric','Value',1000000,'Limits',[1 70000000],'Position',[115 55 110 22],...
        'ValueChangedFcn',@(~,~)onCalcPeriod());
    uilabel(pSamp,'Text','Fs (Hz):','Position',[240 55 55 22]);
    edtFs = uieditfield(pSamp,'numeric','Value',1000,'Limits',[1 200000],'Position',[295 55 90 22],...
        'ValueChangedFcn',@(~,~)onCalcPeriod());

    btnCalcP = uibutton(pSamp,'Text','Calc period','Position',[240 20 140 24],...
        'ButtonPushedFcn',@(~,~)onCalcPeriod());

    lblPeriod = uilabel(pSamp,'Text','period_ticks = ?','Position',[10 20 210 24]);

    % TF panel
    pTF = uipanel(fig,'Title','TF Coeffs (16 floats layout)','Position',[860 20 400 180]);
    uilabel(pTF,'Text','Numerator b0..b5','Position',[10 135 200 18]);
    bEdt = gobjects(1,6);
    for i=1:6
        bEdt(i) = uieditfield(pTF,'text','Value','0','Position',[10+63*(i-1) 110 60 22]);
    end
    uilabel(pTF,'Text','Denominator a0..a5','Position',[10 75 200 18]);
    aEdt = gobjects(1,6);
    for i=1:6
        aEdt(i) = uieditfield(pTF,'text','Value','0','Position',[10+63*(i-1) 50 60 22]);
    end
    aEdt(1).Value = '1';

    % SS panel
    pSS = uipanel(fig,'Title','SS Coeffs (2 estados)','Position',[860 20 400 180]);

    uilabel(pSS,'Text','A (2x2)','Position',[10 135 60 18]);
    edtA11 = uieditfield(pSS,'text','Value','0','Position',[70 132 55 22]);
    edtA12 = uieditfield(pSS,'text','Value','0','Position',[130 132 55 22]);
    edtA21 = uieditfield(pSS,'text','Value','0','Position',[70 105 55 22]);
    edtA22 = uieditfield(pSS,'text','Value','0','Position',[130 105 55 22]);

    uilabel(pSS,'Text','B (2)','Position',[200 135 40 18]);
    edtB1 = uieditfield(pSS,'text','Value','0','Position',[240 132 55 22]);
    edtB2 = uieditfield(pSS,'text','Value','0','Position',[300 132 55 22]);

    uilabel(pSS,'Text','C (2)','Position',[200 105 40 18]);
    edtC1 = uieditfield(pSS,'text','Value','0','Position',[240 102 55 22]);
    edtC2 = uieditfield(pSS,'text','Value','0','Position',[300 102 55 22]);

    uilabel(pSS,'Text','D','Position',[10 75 20 18]);
    edtD = uieditfield(pSS,'text','Value','0','Position',[35 72 55 22]);

    uilabel(pSS,'Text','L (2)','Position',[100 75 40 18]);
    edtL1 = uieditfield(pSS,'text','Value','0','Position',[140 72 55 22]);
    edtL2 = uieditfield(pSS,'text','Value','0','Position',[200 72 55 22]);

    uilabel(pSS,'Text','K (2)','Position',[10 45 40 18]);
    edtK1 = uieditfield(pSS,'text','Value','0','Position',[50 42 55 22]);
    edtK2 = uieditfield(pSS,'text','Value','0','Position',[110 42 55 22]);

    uilabel(pSS,'Text','Ki (integrador)','Position',[200 45 90 18]);
    edtKi = uieditfield(pSS,'text','Value','0','Position',[295 42 60 22]);

    % Timer para polling RX (stream)
    tmr = timer('ExecutionMode','fixedSpacing','Period',0.02,'TimerFcn',@onPollSerial);
    start(tmr);

    onCalcPeriod();
    refreshVisibility();
    updateTsEstimate();

    fig.CloseRequestFcn = @onClose;

    % -------------------- callbacks --------------------
    function refreshVisibility()
        typ = ddType.Value;
        pTF.Visible = strcmp(typ,'TF');
        pSS.Visible = strcmp(typ,'SS');
        ddObserver.Enable = strcmp(typ,'SS');
        cbIntegrator.Enable = strcmp(typ,'SS');
        btnSendCoeffs.Enable = ~strcmp(typ,'Open-loop'); % por ahora tu fw no soporta mode=5
    end

    function onConnectToggle(~,~)
        if ~S.isConnected
            com = strtrim(edtCom.Value);
            if com == ""
                logMsg("COM vacío.");
                return;
            end
            try
                S.sp = serialport(com, edtBaud.Value, "Timeout", 0.2);
                flush(S.sp);
                S.isConnected = true;
                btnConnect.Text = "Disconnect";
                lblStat.Text = "CONNECTED: " + com;
                logMsg("Connected to " + com);
            catch e
                logMsg("Connect error: " + string(e.message));
            end
        else
            try
                if ~isempty(S.sp)
                    flush(S.sp);
                    delete(S.sp);
                end
            catch
            end
            S.sp = [];
            S.isConnected = false;
            btnConnect.Text = "Connect";
            lblStat.Text = "DISCONNECTED";
            logMsg("Disconnected.");
        end
    end

    function onSendMode(~,~)
        if ~requireConn(); return; end

        impl = computeImpl();
        if impl > 4
            logMsg("Open-loop (impl=5) todavía no soportado por firmware actual (acepta 0..4).");
            return;
        end

        payload = uint8([impl, 0, 0, 0]);

        [ok, msg] = cmd_m(payload);
        if ok
            logMsg(sprintf("m OK: impl=%d", impl));
        else
            logMsg("m: " + msg);
        end
    end

    function onCalcPeriod()
        fclk = double(edtFclk.Value);
        Fs   = double(edtFs.Value);

        if ~isfinite(fclk) || fclk <= 0
            logMsg("fclk inválido.");
            return;
        end
        if ~isfinite(Fs) || Fs <= 0
            logMsg("Fs inválida.");
            return;
        end

        period = round(fclk / Fs);
        period = clamp(period, 1, 65535);

        S.fclk = fclk;
        S.Fs   = Fs;
        S.period_ticks = uint16(period);

        lblPeriod.Text = sprintf("period_ticks = %d", double(S.period_ticks));

        updateTsEstimate();
    end

    function onSendCoeffs(~,~)
        if ~requireConn(); return; end

        typ = ddType.Value;
        coeff16 = zeros(16,1,'single');

        % N y period salen de UI actual
        if cbStream.Value
            S.Nval = uint16(clamp(round(double(edtN.Value)),0,65535));
        else
            S.Nval = uint16(0);
        end
        updateTsEstimate();

        Nsingle = single(double(S.Nval));
        Psingle = single(double(S.period_ticks));

        if strcmp(typ,'TF')
            b = parse6(bEdt,'b');
            a = parse6(aEdt,'a');
            coeff16(1:6)  = single(b);
            coeff16(7:12) = single(a);
            coeff16(13:14)= single([0 0]);
            coeff16(15)   = Nsingle;
            coeff16(16)   = Psingle;

        elseif strcmp(typ,'SS')
            A = single([parse1(edtA11,'A11') parse1(edtA12,'A12') parse1(edtA21,'A21') parse1(edtA22,'A22')]);
            B = single([parse1(edtB1,'B1') parse1(edtB2,'B2')]);
            C = single([parse1(edtC1,'C1') parse1(edtC2,'C2')]);
            D = single(parse1(edtD,'D'));
            L = single([parse1(edtL1,'L1') parse1(edtL2,'L2')]);
            K = single([parse1(edtK1,'K1') parse1(edtK2,'K2')]);
            Ki = single(parse1(edtKi,'Ki'));

            coeff16(1:4)   = A;
            coeff16(5:6)   = B;
            coeff16(7:8)   = C;
            coeff16(9)     = D;
            coeff16(10:11) = L;
            coeff16(12:13) = K;
            coeff16(14)    = Ki;
            coeff16(15)    = Nsingle;
            coeff16(16)    = Psingle;

        else
            logMsg("Open-loop: por ahora no mandamos coeficientes (tu fw no soporta mode=5).");
            return;
        end

        raw = typecast(coeff16, 'uint8');

        [ok, msg] = cmd_c(raw);
        if ~ok
            logMsg("c: " + msg);
            return;
        end
        logMsg(sprintf("c OK: 16 floats enviados (N=%d, period=%d).", double(S.Nval), double(S.period_ticks)));

        if cbVerify.Value
            [ok2, rx, msg2] = cmd_t();
            if ~ok2
                logMsg("t: " + msg2);
                return;
            end
            if isequal(uint8(rx), uint8(raw(:)))
                logMsg("Verify OK: t == c");
            else
                logMsg("Verify FAIL: t != c");
            end
        end
    end

    function onStart(~,~)
        if ~requireConn(); return; end

        r0 = single(edtU0.Value);
        raw = typecast(r0,'uint8');

        [ok, msg] = cmd_i(raw);
        if ok
            logMsg(sprintf("i OK: ref=%.6g", double(r0)));
        else
            logMsg("i: " + msg);
        end
    end

    function onStop(~,~)
        if ~requireConn(); return; end
        write(S.sp, uint8('s'), "uint8");
        b = readByteWithTimeout(0.5);
        if isempty(b)
            logMsg("s sent (no rsp).");
        else
            logMsg("s rsp: " + char(b));
        end
    end

    function onPollSerial(~,~)
        if ~S.isConnected || isempty(S.sp); return; end
        if ~cbStream.Value
            if S.sp.NumBytesAvailable > 0
                read(S.sp, S.sp.NumBytesAvailable, "uint8");
            end
            return;
        end

        n = S.sp.NumBytesAvailable;
        if n < 4; return; end

        n4 = floor(n/4)*4;
        bytes = read(S.sp, n4, "uint8");

        for ii=1:4:n4
            val = typecast(uint8(bytes(ii:ii+3)), 'single');

            if S.expectU
                S.uTemp = double(val);
                S.expectU = false;
            else
                yval = double(val);
                S.expectU = true;

                S.k = S.k + 1;
                if S.k > S.maxPoints
                    S.t(1:end-1) = S.t(2:end);
                    S.u(1:end-1) = S.u(2:end);
                    S.y(1:end-1) = S.y(2:end);
                    S.k = S.maxPoints;
                end

                if S.k == 1
                    S.t(S.k) = 0;
                else
                    S.t(S.k) = S.t(S.k-1) + S.ts_est;
                end
                S.u(S.k) = S.uTemp;
                S.y(S.k) = yval;

                set(hU,'XData',S.t(1:S.k),'YData',S.u(1:S.k));
                set(hY,'XData',S.t(1:S.k),'YData',S.y(1:S.k));
                drawnow limitrate nocallbacks;
            end
        end
    end

    % -------------------- helpers --------------------
    function ok = requireConn()
        ok = S.isConnected && ~isempty(S.sp);
        if ~ok
            logMsg("Not connected.");
        end
    end

    function logMsg(msg)
        ts = string(datestr(now,'HH:MM:SS.FFF'));
        line = "[" + ts + "] " + string(msg);

        v = txtLog.Value;
        if ~isstring(v), v = string(v); end
        v(end+1,1) = line;

        if numel(v) > 250
            v = v(end-250:end);
        end
        txtLog.Value = v;
        drawnow limitrate;
    end

    function impl = computeImpl()
        typ = ddType.Value;
        if strcmp(typ,'TF')
            impl = uint8(0);
            return;
        end
        if strcmp(typ,'Open-loop')
            impl = uint8(5); % reservado
            return;
        end

        isAct = strcmp(ddObserver.Value,'Actual');
        hasI  = cbIntegrator.Value;

        if ~isAct && ~hasI
            impl = uint8(1);
        elseif isAct && ~hasI
            impl = uint8(2);
        elseif ~isAct && hasI
            impl = uint8(3);
        else
            impl = uint8(4);
        end
    end

    function updateTsEstimate()
        % Ts base desde fclk/period
        fclk = double(S.fclk);
        period = double(S.period_ticks);
        Ts = period / fclk;

        if cbStream.Value
            Nval = double(edtN.Value);
            if ~isfinite(Nval) || Nval < 1, Nval = 1; end
        else
            Nval = 1;
        end
        S.ts_est = Ts * Nval;
    end

    function b = readByteWithTimeout(toutSec)
        b = [];
        t0 = tic;
        while toc(t0) < toutSec
            if S.sp.NumBytesAvailable >= 1
                tmp = read(S.sp, 1, "uint8");
                b = tmp(1);
                return;
            end
            pause(0.002);
        end
    end

    % ---- commands ----
    function [ok, msg] = cmd_m(payload4)
        ok = false; msg = "";
        flush(S.sp);
        write(S.sp, uint8('m'), "uint8");
        r = readByteWithTimeout(0.6);
        if isempty(r), msg="timeout esperando R"; return; end
        if char(r)~='R', msg=sprintf("expected R, got '%s' (0x%02X)", char(r), r); return; end
        write(S.sp, uint8(payload4), "uint8");
        k = readByteWithTimeout(0.6);
        if isempty(k), msg="timeout esperando K"; return; end
        if char(k)~='K', msg=sprintf("expected K, got '%s' (0x%02X)", char(k), k); return; end
        ok = true;
    end

    function [ok, msg] = cmd_i(payload4)
        ok = false; msg = "";
        flush(S.sp);
        write(S.sp, uint8('i'), "uint8");
        r = readByteWithTimeout(0.6);
        if isempty(r), msg="timeout esperando R"; return; end
        if char(r)~='R', msg=sprintf("expected R, got '%s' (0x%02X)", char(r), r); return; end
        write(S.sp, uint8(payload4), "uint8");
        k = readByteWithTimeout(0.6);
        if isempty(k), msg="timeout esperando K"; return; end
        if char(k)~='K', msg=sprintf("expected K, got '%s' (0x%02X)", char(k), k); return; end
        ok = true;
    end

    function [ok, msg] = cmd_c(raw64)
        ok = false; msg = "";
        raw64 = uint8(raw64(:));
        if numel(raw64) ~= 64
            msg = "payload c debe ser 64 bytes";
            return;
        end

        flush(S.sp);
        write(S.sp, uint8('c'), "uint8");
        r = readByteWithTimeout(0.8);
        if isempty(r), msg="timeout esperando R"; return; end
        if char(r)~='R', msg=sprintf("expected R, got '%s' (0x%02X)", char(r), r); return; end
        write(S.sp, raw64, "uint8");
        k = readByteWithTimeout(1.2);
        if isempty(k), msg="timeout esperando K"; return; end
        if char(k)~='K', msg=sprintf("expected K, got '%s' (0x%02X)", char(k), k); return; end
        ok = true;
    end

    function [ok, rx, msg] = cmd_t()
        ok = false; msg = ""; rx = [];
        flush(S.sp);
        write(S.sp, uint8('t'), "uint8");
        s = readByteWithTimeout(0.8);
        if isempty(s), msg="timeout esperando S"; return; end
        if char(s)~='S', msg=sprintf("expected S, got '%s' (0x%02X)", char(s), s); return; end

        rx = readExact(64, 1.2);
        if isempty(rx), msg="timeout leyendo 64 bytes"; return; end

        k = readByteWithTimeout(0.8);
        if isempty(k), msg="timeout esperando K"; return; end
        if char(k)~='K', msg=sprintf("expected K, got '%s' (0x%02X)", char(k), k); return; end

        ok = true;
    end

    function data = readExact(nbytes, tout)
        data = [];
        t0 = tic;
        buf = uint8([]);
        while toc(t0) < tout
            na = S.sp.NumBytesAvailable;
            if na > 0
                buf = [buf; read(S.sp, na, "uint8")]; %#ok<AGROW>
                if numel(buf) >= nbytes
                    data = buf(1:nbytes);
                    return;
                end
            end
            pause(0.002);
        end
    end

    % parsing
    function v = parse6(edts, name)
        v = zeros(1,6);
        for kk=1:6
            v(kk) = parse1(edts(kk), sprintf("%s[%d]",name,kk-1));
        end
    end

    function v = parse1(edt, name)
        s = strtrim(edt.Value);
        if isempty(s)
            uialert(fig, "Campo vacío: " + name, 'Input error');
            error("Campo vacío: %s", name);
        end
        v = str2double(s);
        if ~isfinite(v)
            uialert(fig, "Número inválido en " + name + ": '" + s + "'", 'Input error');
            error("Número inválido: %s", name);
        end
    end

    function x = clamp(x, lo, hi)
        x = min(max(x, lo), hi);
    end

    function onClose(~,~)
        try
            stop(tmr); delete(tmr);
        catch
        end
        try
            if S.isConnected && ~isempty(S.sp)
                flush(S.sp);
                delete(S.sp);
            end
        catch
        end
        delete(fig);
    end
end
