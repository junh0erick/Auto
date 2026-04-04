function pendulo_gui2()
%PENDULO_GUI2  Two-plant control GUI for Inverted Pendulum PSoC.
%
%  Protocol (UARTP2):
%    'r' -> reset                     -> PSoC: 'K'
%    '1' -> set mode plant 0 (inner)  -> PSoC: 'R' -> host: 4B  -> PSoC: 'K'
%    '2' -> set mode plant 1 (outer)  -> PSoC: 'R' -> host: 4B  -> PSoC: 'K'
%    'a' -> coefficients plant 0      -> PSoC: 'R' -> host: 100B -> PSoC: 'K'
%    'b' -> coefficients plant 1      -> PSoC: 'R' -> host: 100B -> PSoC: 'K'
%    'i' -> start (ref float32)       -> PSoC: 'R' -> host: 4B  -> PSoC: 'K'
%    's' -> stop                      -> PSoC: 'K'
%  Telemetry: 16 B/frame  [u1_f32  y1_f32  u2_f32  y2_f32]
%
%  Modes: 0=TF  1=SS-Pred-NoI  2=SS-Act-NoI  3=SS-Pred-I  4=SS-Act-I
%         5=OpenLoop  6=Off
%
%  Coefficient layout (25 single-precision floats, 1-indexed MATLAB):
%    TF:  c(1:6)=b  c(7:12)=a  c(15)=N  c(16)=Fs_hz
%    SS:  c(1:4)=A(2x2 row-maj)  c(5:6)=B  c(7:8)=C  c(9)=D
%         c(10:11)=L  c(12:13)=K  c(14)=Kx  c(15)=N  c(16)=Fs_hz

%% ── Constants ────────────────────────────────────────────────────────────────
FRAME_SZ       = 16;    % bytes per telemetry frame
STEP_TMO       = 0.8;   % seconds, per handshake step
MAX_RETRIES    = 50;
MAX_PTS        = 500000;
STREAM_PERIOD  = 0.050; % 50 ms timer

MODE_NAMES = {'TF','SS Pred-NoI','SS Act-NoI','SS Pred-I','SS Act-I','Open-loop','Off'};
MODE_VALS  = [  0,            1,           2,          3,         4,          5,   6];
PLANT_CMD_MODE  = {'1','2'};   % mode command chars
PLANT_CMD_COEF  = {'a','b'};   % coeff command chars

%% ── State ────────────────────────────────────────────────────────────────────
S = struct();
S.sp = []; S.isConnected = false;
S.streamTimer = []; S.rxBuf = uint8([]); S.streamOn = false; S.inTxn = false;

S.nVec  = zeros(0,1);
S.u1Vec = zeros(0,1);  S.y1Vec = zeros(0,1);
S.u2Vec = zeros(0,1);  S.y2Vec = zeros(0,1);
S.framesTotal = 0;
S.autoStopEn  = false; S.autoStopN = 0; S.autoStopArmed = false;

%% ── Figure ───────────────────────────────────────────────────────────────────
FIG_W = 1500;  FIG_H = 860;
fig = uifigure('Name','Péndulo Invertido — Control GUI v2', ...
               'Position',[20 20 FIG_W FIG_H]);
fig.CloseRequestFcn = @onClose;

%% ── Axes (left, 4 stacked) ──────────────────────────────────────────────────
AX_X = 8;  AX_W = 892;
ax_u1 = uiaxes(fig,'Position',[AX_X 648 AX_W 206]);
ax_y1 = uiaxes(fig,'Position',[AX_X 432 AX_W 206]);
ax_u2 = uiaxes(fig,'Position',[AX_X 216 AX_W 206]);
ax_y2 = uiaxes(fig,'Position',[AX_X   5 AX_W 204]);

setupAx(ax_u1,'u₁  [PWM counts]',   'u₁');
setupAx(ax_y1,'y₁ = ω motor [rad/s]','ω [rad/s]');
setupAx(ax_u2,'u₂  [outer output]', 'u₂');
setupAx(ax_y2,'y₂ = θ péndulo [rad]','θ [rad]');

hold(ax_u1,'on'); hU1 = stairs(ax_u1,nan,nan,'b-','LineWidth',1,'DisplayName','u₁'); hold(ax_u1,'off'); legend(ax_u1,'show');
hold(ax_y1,'on'); hY1 = plot(ax_y1,nan,nan,'b-','LineWidth',1,'DisplayName','ω real'); hold(ax_y1,'off'); legend(ax_y1,'show');
hold(ax_u2,'on'); hU2 = stairs(ax_u2,nan,nan,'r-','LineWidth',1,'DisplayName','u₂'); hold(ax_u2,'off'); legend(ax_u2,'show');
hold(ax_y2,'on'); hY2 = plot(ax_y2,nan,nan,'r-','LineWidth',1,'DisplayName','θ real'); hold(ax_y2,'off'); legend(ax_y2,'show');

%% ── Right panels ─────────────────────────────────────────────────────────────
RX = 907;  RW = 586;

% ─ 1. Connection  y=820 h=35 ─────────────────────────────────────────────────
pConn = uipanel(fig,'Title','Conexión','Position',[RX 820 RW 35]);
uilabel(pConn,'Text','Puerto:','Position',[  6 5 45 22]);
edtCom  = uieditfield(pConn,'text','Value','COM4','Position',[53 5 68 22]);
uilabel(pConn,'Text','Baud:',  'Position',[126 5 38 22]);
edtBaud = uieditfield(pConn,'numeric','Value',115200,'Limits',[1200 4e6],'Position',[166 5 90 22]);
btnConn  = uibutton(pConn,'Text','Conectar','Position',[262 4 100 24],...
    'BackgroundColor',[0.2 0.62 0.2],'FontColor','w','ButtonPushedFcn',@onConnToggle);
lblStat = uilabel(pConn,'Text','Desconectado','Position',[368 5 140 22]);
btnRst  = uibutton(pConn,'Text','Reset (r)','Position',[512 4 68 24],'ButtonPushedFcn',@onReset);

% ─ 2 & 3. Plant panels (one per plant) ───────────────────────────────────────
%   Each panel contains: mode row, N/Fs row, button row, + floating TF/SS panel.
%   Floating TF/SS panels sit at the bottom of each plant panel.

PLANT_Y   = [560  302];
PLANT_H   = 255;
PLANT_LBL = {'Planta 0 — Motor/ω  (inner)','Planta 1 — Péndulo/θ  (outer)'};

% Per-plant UI handles stored in arrays
ddMode  = gobjects(2,1);
ddObs   = gobjects(2,1);
cbInt   = gobjects(2,1);
edtN    = gobjects(2,1);
edtFs   = gobjects(2,1);
btnSM   = gobjects(2,1);
btnSC   = gobjects(2,1);
pTF     = gobjects(2,1);
pSS     = gobjects(2,1);

% TF table handles
tfTable    = gobjects(2,1);
edtTFOrd   = gobjects(2,1);

% SS field handles (14 per plant → stored as cell array of gobjects)
ssFields   = cell(2,1);   % each entry is struct with A/B/C/D/L/K/Kx editfields

for pp = 1:2
    pPlant = uipanel(fig,'Title',PLANT_LBL{pp},'Position',[RX PLANT_Y(pp) RW PLANT_H]);

    % ── Mode row (top of panel) ──────────────────────────────────────────────
    y_mode = PLANT_H - 42;
    uilabel(pPlant,'Text','Modo:',    'Position',[ 6 y_mode 40 22]);
    ddMode(pp) = uidropdown(pPlant,'Items',MODE_NAMES,'Value','Off',...
        'Position',[48 y_mode 130 22],...
        'ValueChangedFcn',@(~,~) refreshPlantVis(pp));

    uilabel(pPlant,'Text','Obs:',     'Position',[184 y_mode 28 22]);
    ddObs(pp)  = uidropdown(pPlant,'Items',{'Predictor','Actual'},...
        'Position',[213 y_mode 90 22]);

    cbInt(pp)  = uicheckbox(pPlant,'Text','Integrador',...
        'Position',[308 y_mode 90 22]);

    % ── N / Fs row ──────────────────────────────────────────────────────────
    y_nfs = y_mode - 26;
    uilabel(pPlant,'Text','N:',       'Position',[ 6 y_nfs 20 22]);
    edtN(pp)  = uieditfield(pPlant,'numeric','Value',1,'Limits',[1 1e6],...
        'RoundFractionalValues','on','Position',[28 y_nfs 68 22]);
    uilabel(pPlant,'Text','Fs (Hz):', 'Position',[104 y_nfs 52 22]);
    edtFs(pp) = uieditfield(pPlant,'numeric','Value',200,'Limits',[0.1 10000],...
        'Position',[158 y_nfs 80 22]);

    % ── Buttons ─────────────────────────────────────────────────────────────
    y_btn = y_nfs - 28;
    btnSM(pp) = uibutton(pPlant,'Text',sprintf('Enviar Modo (%s)',PLANT_CMD_MODE{pp}),...
        'Position',[6 y_btn 148 26],...
        'ButtonPushedFcn',@(~,~) onSendMode(pp));
    btnSC(pp) = uibutton(pPlant,'Text',sprintf('Enviar Coefs (%s)',PLANT_CMD_COEF{pp}),...
        'Position',[160 y_btn 148 26],...
        'ButtonPushedFcn',@(~,~) onSendCoeffs(pp));

    % ── TF sub-panel ────────────────────────────────────────────────────────
    TF_H = y_btn - 6;   % fills remaining height
    pTF(pp) = uipanel(pPlant,'Title','TF  (b/a, máx orden 5)',...
        'Position',[3 0 RW-10 TF_H]);

    uilabel(pTF(pp),'Text','k',     'Position',[ 6 TF_H-50 20 18]);
    uilabel(pTF(pp),'Text','b(k)',  'Position',[30 TF_H-50 55 18]);
    uilabel(pTF(pp),'Text','a(k)',  'Position',[130 TF_H-50 55 18]);
    tdata = [zeros(6,1), [1;zeros(5,1)]];
    tfTable(pp) = uitable(pTF(pp),'Data',tdata,'ColumnName',{'b','a'},...
        'RowName',{'0','1','2','3','4','5'},...
        'ColumnEditable',[true true],...
        'Position',[6 TF_H-195 230 140]);
    uilabel(pTF(pp),'Text','Orden (0..5):','Position',[246 TF_H-80 88 18]);
    edtTFOrd(pp) = uieditfield(pTF(pp),'numeric','Value',1,'Limits',[0 5],...
        'RoundFractionalValues','on','Position',[246 TF_H-104 60 22]);

    % ── SS sub-panel ────────────────────────────────────────────────────────
    pSS(pp) = uipanel(pPlant,'Title','SS  (2 estados)',...
        'Position',[3 0 RW-10 TF_H]);

    % Layout: 2 columns of fields, narrow labels
    FW = 58;  FH = 22;  xA = 6; xB = xA+FW+4; xC = xB+FW+4; xD = xC+FW+4;
    yR = [TF_H-200, TF_H-228, TF_H-256, TF_H-284, TF_H-312];  % row y positions

    uilabel(pSS(pp),'Text','A(2×2)  B(2×1)  C(1×2)  D','Position',[xA yR(1)+22 280 18],'FontSize',10);
    f11=ssEdt(pSS(pp),xA, yR(1), '1'); f12=ssEdt(pSS(pp),xB, yR(1), '0');
    f21=ssEdt(pSS(pp),xA, yR(2), '0'); f22=ssEdt(pSS(pp),xB, yR(2), '1');
    fB1=ssEdt(pSS(pp),xC, yR(1), '0'); fB2=ssEdt(pSS(pp),xC, yR(2), '0');
    fC1=ssEdt(pSS(pp),xD, yR(1), '0'); fC2=ssEdt(pSS(pp),xD, yR(2), '0');

    uilabel(pSS(pp),'Text','D:',          'Position',[xA yR(3)+22 20 18]);
    fD =ssEdt(pSS(pp),xA+22, yR(3), '0');

    uilabel(pSS(pp),'Text','L(2×1)  K(1×2)  Kx','Position',[xA yR(4)+22 200 18],'FontSize',10);
    fL1=ssEdt(pSS(pp),xA, yR(4), '0'); fL2=ssEdt(pSS(pp),xA, yR(5), '0');
    fK1=ssEdt(pSS(pp),xB, yR(4), '0'); fK2=ssEdt(pSS(pp),xB, yR(5), '0');
    uilabel(pSS(pp),'Text','Kx:',         'Position',[xC yR(4)+22 24 18]);
    fKx=ssEdt(pSS(pp),xC+26, yR(4), '0');

    ssFields{pp} = struct('A11',f11,'A12',f12,'A21',f21,'A22',f22,...
        'B1',fB1,'B2',fB2,'C1',fC1,'C2',fC2,'D',fD,...
        'L1',fL1,'L2',fL2,'K1',fK1,'K2',fK2,'Kx',fKx);

    refreshPlantVis(pp);  % initial visibility
end

% ─ 4. Control  y=222 h=75 ────────────────────────────────────────────────────
pCtrl = uipanel(fig,'Title','Control','Position',[RX 222 RW 75]);
uilabel(pCtrl,'Text','Ref inicial:','Position',[6 38 70 22]);
edtRef = uieditfield(pCtrl,'numeric','Value',0,'Limits',[-1e6 1e6],'Position',[78 38 80 22]);
btnStart = uibutton(pCtrl,'Text','▶  Start (i)','Position',[164 36 120 26],...
    'BackgroundColor',[0.12 0.62 0.12],'FontColor','w','FontWeight','bold',...
    'ButtonPushedFcn',@onStart);
btnStop  = uibutton(pCtrl,'Text','■  Stop (s)','Position',[290 36 110 26],...
    'BackgroundColor',[0.78 0.1 0.1],'FontColor','w','FontWeight','bold',...
    'ButtonPushedFcn',@onStop);
lblRunSt = uilabel(pCtrl,'Text','Detenido','Position',[408 36 120 26],'FontWeight','bold');

cbAutoStop = uicheckbox(pCtrl,'Text','Auto-stop en frames:','Value',false,...
    'Position',[6 8 148 22],'ValueChangedFcn',@onAutoStopToggle);
edtAutoN   = uieditfield(pCtrl,'numeric','Value',2000,'Limits',[1 1e9],...
    'RoundFractionalValues','on','Position',[158 8 80 22]);
lblDataInf = uilabel(pCtrl,'Text','n=0','Position',[244 8 120 22]);

% ─ 5. Datos  y=138 h=79 ──────────────────────────────────────────────────────
pData = uipanel(fig,'Title','Datos','Position',[RX 138 RW 79]);
btnExp   = uibutton(pData,'Text','Exportar .mat','Position',[  6 10 130 28],'ButtonPushedFcn',@onExport);
btnClear = uibutton(pData,'Text','Borrar datos', 'Position',[142 10 120 28],'ButtonPushedFcn',@onClear);
uilabel(pData,'Text','u x:','Position',[272 14 30 18]);
edtSU1 = uieditfield(pData,'numeric','Value',1,'Limits',[-1e9 1e9],'Position',[304 10 62 22],...
    'ValueChangedFcn',@updatePlots);
uilabel(pData,'Text','ux2:','Position',[370 14 30 18]);
edtSU2 = uieditfield(pData,'numeric','Value',1,'Limits',[-1e9 1e9],'Position',[402 10 62 22],...
    'ValueChangedFcn',@updatePlots);

% ─ 6. Log  y=5 h=128 ─────────────────────────────────────────────────────────
txtLog = uitextarea(fig,'Editable','off','Position',[RX 5 RW 128],'FontSize',9);
txtLog.Value = strings(0,1);

%% ── Helpers UI ───────────────────────────────────────────────────────────────
    function logMsg(msg)
        ts = string(datestr(now,'HH:MM:SS.FFF'));
        ln = "[" + ts + "] " + string(msg);
        v  = txtLog.Value; if ~isstring(v), v = string(v); end
        v(end+1,1) = ln;
        if numel(v) > 400, v = v(end-400:end); end
        txtLog.Value = v;
        drawnow limitrate;
    end

    function ok = reqConn()
        ok = S.isConnected && ~isempty(S.sp);
        if ~ok, logMsg("⚠ No conectado."); end
    end

    function setupAx(ax, ttl, yl)
        ax.XGrid = 'on'; ax.YGrid = 'on';
        ax.Title.String  = ttl;
        ax.XLabel.String = 'frame';
        ax.YLabel.String = yl;
    end

    function ef = ssEdt(parent, x, y, val)
        ef = uieditfield(parent,'text','Value',val,'Position',[x y 58 22]);
    end

    function v = ssVal(ef)
        v = str2double(strtrim(string(ef.Value)));
        if ~isfinite(v), v = 0; end
    end

    function refreshPlantVis(pp)
        modeName = ddMode(pp).Value;
        isTF  = strcmp(modeName,'TF');
        isSS  = contains(modeName,'SS');
        pTF(pp).Visible = isTF;
        pSS(pp).Visible = isSS;
        ddObs(pp).Enable  = isSS;
        cbInt(pp).Enable  = isSS;
    end

    function updatePlots(~,~)
        if isempty(S.nVec), return; end
        n = S.nVec;
        su1 = edtSU1.Value; su2 = edtSU2.Value;
        if ~isfinite(su1)||su1==0, su1=1; end
        if ~isfinite(su2)||su2==0, su2=1; end
        set(hU1,'XData',n,'YData',S.u1Vec*su1);
        set(hY1,'XData',n,'YData',S.y1Vec);
        set(hU2,'XData',n,'YData',S.u2Vec*su2);
        set(hY2,'XData',n,'YData',S.y2Vec);
    end

    function updateDataInfo()
        try
            n = numel(S.nVec);
            lblDataInf.Text = sprintf('n=%d', n);
        catch, end
    end

%% ── UARTP2 Low-level ─────────────────────────────────────────────────────────
    function ll_flush()
        if ~isempty(S.sp), try flush(S.sp); catch, end; end
    end

    function b = ll_readexact(n, tmo)
        if nargin < 2, tmo = STEP_TMO; end
        t0 = tic; buf = zeros(1,n,'uint8'); k = 0;
        while k < n
            if toc(t0) > tmo, error('Timeout (%d/%d bytes)',k,n); end
            av = S.sp.NumBytesAvailable;
            if av > 0
                m = min(av,n-k);
                tmp = read(S.sp,m,"uint8");
                buf(k+1:k+m) = tmp(:);
                k = k+m;
            else, pause(0.001); end
        end
        b = buf(:);
    end

    function rsp = ll_cmd_wait(cmd, tmo)
        if nargin < 2, tmo = STEP_TMO; end
        valid = uint8(['R','K','!']);
        write(S.sp, uint8(cmd), "uint8");
        t0 = tic;
        while true
            if toc(t0) > tmo, error("Timeout cmd '%s'", cmd); end
            if S.sp.NumBytesAvailable > 0
                b = read(S.sp,1,"uint8"); b=b(1);
                if any(b==valid), rsp=b; return; end
            else, pause(0.001); end
        end
    end

    function ll_send_payload(data_u8)
        data_u8 = uint8(data_u8(:));
        n    = numel(data_u8);
        idx  = 1;
        while idx <= n
            w    = uint8([0;0;0;0]);
            take = min(4, n-idx+1);
            w(1:take) = data_u8(idx:idx+take-1);

            tries = 0;
            while true
                tries = tries+1;
                if tries > MAX_RETRIES, error('Max retries en idx=%d',idx); end
                write(S.sp, w, "uint8");
                echo = ll_readexact(4); match = isequal(echo(:),w(:));
                if match, write(S.sp,uint8('A'),"uint8");
                else,     write(S.sp,uint8('N'),"uint8"); end
                conf = ll_readexact(1); conf = conf(1);
                if match && conf == uint8('A'), break; end
            end
            idx = idx + take;
        end
    end

%% ── UARTP2 High-level commands ───────────────────────────────────────────────
    function uartp2_reset()
        ll_flush();
        rsp = ll_cmd_wait('r');
        if rsp ~= uint8('K'), error("reset: rsp=%c",char(rsp)); end
    end

    function uartp2_setmode(plant_idx, mode_val)
        cmd_char = PLANT_CMD_MODE{plant_idx};
        ll_flush();
        rsp = ll_cmd_wait(cmd_char);
        if rsp ~= uint8('R'), error("mode(%d): rsp=%c",plant_idx,char(rsp)); end
        ll_send_payload(uint8([mode_val; 0; 0; 0]));
        fin = ll_readexact(1); fin = fin(1);
        if fin ~= uint8('K'), error("mode(%d): fin=%c",plant_idx,char(fin)); end
    end

    function uartp2_sendcoeffs(plant_idx, coeffs25)
        cmd_char = PLANT_CMD_COEF{plant_idx};
        payload  = typecast(single(coeffs25(:)), 'uint8');
        if numel(payload) ~= 100
            error("coeffs25 debe tener 25 floats (100 bytes), tiene %d bytes",numel(payload));
        end
        ll_flush();
        rsp = ll_cmd_wait(cmd_char);
        if rsp ~= uint8('R'), error("coeff(%d): rsp=%c",plant_idx,char(rsp)); end
        ll_send_payload(payload);
        fin = ll_readexact(1); fin = fin(1);
        if fin ~= uint8('K'), error("coeff(%d): fin=%c",plant_idx,char(fin)); end
    end

    function uartp2_start(ref_val)
        ll_flush();
        rsp = ll_cmd_wait('i');
        if rsp ~= uint8('R'), error("start: rsp=%c",char(rsp)); end
        ll_send_payload(typecast(single(ref_val),'uint8'));
        fin = ll_readexact(1); fin = fin(1);
        if fin ~= uint8('K'), error("start: fin=%c",char(fin)); end
    end

    function uartp2_stop()
        try, write(S.sp, uint8('s'), "uint8"); catch, end
        % Scan for 'K' (discarding telemetry bytes)
        t0 = tic;
        while toc(t0) < 1.5
            if S.sp.NumBytesAvailable > 0
                b = read(S.sp,1,"uint8"); b=b(1);
                if b == uint8('K'), return; end
            else, pause(0.002); end
        end
    end

%% ── Coefficient builders ─────────────────────────────────────────────────────
    function c = make_tf(pp)
        D  = double(tfTable(pp).Data);
        if iscell(D)
            D2 = zeros(size(D));
            for r=1:size(D,1)
                for cc=1:size(D,2)
                    v=str2double(string(D{r,cc})); if ~isfinite(v), v=0; end; D2(r,cc)=v;
                end
            end
            D = D2;
        end
        b = D(:,1)'; a = D(:,2)';

        c = single(zeros(25,1));
        nb = min(6,numel(b)); na = min(6,numel(a));
        c(1:nb)   = single(b(1:nb));
        c(7:6+na) = single(a(1:na));
        c(15) = single(round(edtN(pp).Value));
        c(16) = single(edtFs(pp).Value);
    end

    function c = make_ss(pp)
        sf = ssFields{pp};
        c = single(zeros(25,1));
        c(1)  = single(ssVal(sf.A11)); c(2)  = single(ssVal(sf.A12));
        c(3)  = single(ssVal(sf.A21)); c(4)  = single(ssVal(sf.A22));
        c(5)  = single(ssVal(sf.B1));  c(6)  = single(ssVal(sf.B2));
        c(7)  = single(ssVal(sf.C1));  c(8)  = single(ssVal(sf.C2));
        c(9)  = single(ssVal(sf.D));
        c(10) = single(ssVal(sf.L1));  c(11) = single(ssVal(sf.L2));
        c(12) = single(ssVal(sf.K1));  c(13) = single(ssVal(sf.K2));
        c(14) = single(ssVal(sf.Kx));
        c(15) = single(round(edtN(pp).Value));
        c(16) = single(edtFs(pp).Value);
    end

    function c = make_ol_off(pp)
        c = single(zeros(25,1));
        c(15) = single(round(edtN(pp).Value));
        c(16) = single(edtFs(pp).Value);
    end

    function mode_val = get_mode_val(pp)
        mn  = ddMode(pp).Value;
        idx = find(strcmp(MODE_NAMES, mn), 1);
        if isempty(idx), mode_val = 6; return; end
        mode_val = MODE_VALS(idx);
        % For SS: adjust for observer + integrator
        if mode_val >= 1 && mode_val <= 4
            isAct = strcmp(ddObs(pp).Value,'Actual');
            hasI  = cbInt(pp).Value;
            if ~isAct && ~hasI, mode_val = 1;
            elseif isAct && ~hasI, mode_val = 2;
            elseif ~isAct && hasI, mode_val = 3;
            else,                  mode_val = 4; end
        end
    end

    function c = build_coeffs(pp)
        mn = ddMode(pp).Value;
        if strcmp(mn,'TF')
            c = make_tf(pp);
        elseif contains(mn,'SS')
            c = make_ss(pp);
        else
            c = make_ol_off(pp);
        end
    end

%% ── Actions ──────────────────────────────────────────────────────────────────
    function onConnToggle(~,~)
        if ~S.isConnected
            com = strtrim(string(edtCom.Value));
            if com == "", logMsg("Puerto vacío."); return; end
            try
                S.sp = serialport(com, edtBaud.Value);
                S.sp.Timeout = 0.1;
                flush(S.sp); pause(0.05); ll_flush();
                S.isConnected = true;
                btnConn.Text = 'Desconectar';
                btnConn.BackgroundColor = [0.65 0.15 0.15];
                lblStat.Text = "CONECTADO: " + com;
                logMsg("Conectado: " + com + " @ " + edtBaud.Value);
                startStream();
            catch e
                logMsg("Error conexión: " + string(e.message));
                S.sp = []; S.isConnected = false;
            end
        else
            stopStream();
            try, if ~isempty(S.sp), flush(S.sp); delete(S.sp); end, catch, end
            S.sp = []; S.isConnected = false;
            btnConn.Text = 'Conectar';
            btnConn.BackgroundColor = [0.2 0.62 0.2];
            lblStat.Text = 'Desconectado';
            logMsg("Desconectado.");
        end
    end

    function onReset(~,~)
        if ~reqConn(), return; end
        try
            S.inTxn = true; stopStream(); ll_flush();
            uartp2_reset();
            logMsg("Reset OK.");
        catch e
            logMsg("Reset FAIL: " + string(e.message));
        end
        S.inTxn = false; startStream();
    end

    function onSendMode(pp)
        if ~reqConn(), return; end
        try
            S.inTxn = true; stopStream(); ll_flush();
            mv = get_mode_val(pp);
            uartp2_setmode(pp, mv);
            logMsg(sprintf("Planta %d: modo %d (%s) enviado.", pp-1, mv, ddMode(pp).Value));
        catch e
            logMsg(sprintf("SendMode planta %d FAIL: %s", pp-1, string(e.message)));
        end
        S.inTxn = false; startStream();
    end

    function onSendCoeffs(pp)
        if ~reqConn(), return; end
        try
            S.inTxn = true; stopStream(); ll_flush();
            c = build_coeffs(pp);
            uartp2_sendcoeffs(pp, c);
            logMsg(sprintf("Planta %d: coeficientes enviados. N=%d  Fs=%.1f Hz", ...
                pp-1, round(c(15)), c(16)));
        catch e
            logMsg(sprintf("SendCoeffs planta %d FAIL: %s", pp-1, string(e.message)));
        end
        S.inTxn = false; startStream();
    end

    function onStart(~,~)
        if ~reqConn(), return; end
        ref0 = edtRef.Value;
        try
            S.inTxn = true; stopStream(); ll_flush();
            uartp2_reset();
            % Send both modes and coefficients
            for pp = 1:2
                mv = get_mode_val(pp);
                uartp2_setmode(pp, mv);
                c = build_coeffs(pp);
                uartp2_sendcoeffs(pp, c);
                logMsg(sprintf("Planta %d: modo=%d  N=%d  Fs=%.1f Hz", ...
                    pp-1, mv, round(c(15)), c(16)));
            end
            uartp2_start(ref0);
            S.streamOn = true;
            S.rxBuf = uint8([]);
            if S.autoStopEn && S.autoStopN > 0
                S.autoStopArmed = true;
            else
                S.autoStopArmed = false;
            end
            lblRunSt.Text = '▶ CORRIENDO';
            lblRunSt.FontColor = [0 0.5 0];
            logMsg(sprintf("Control iniciado. ref0=%.4g", ref0));
        catch e
            logMsg("Start FAIL: " + string(e.message));
            S.streamOn = false;
        end
        S.inTxn = false; startStream();
    end

    function onStop(~,~)
        if ~reqConn(), return; end
        try
            S.inTxn = true; S.streamOn = false;
            stopStream(); ll_flush();
            uartp2_stop();
            lblRunSt.Text = 'Detenido';
            lblRunSt.FontColor = [0 0 0];
            logMsg("Control detenido.");
        catch e
            logMsg("Stop FAIL: " + string(e.message));
        end
        S.inTxn = false; startStream();
    end

    function onAutoStopToggle(~,~)
        S.autoStopEn = cbAutoStop.Value;
    end

%% ── Streaming ────────────────────────────────────────────────────────────────
    function startStream()
        stopStream();
        if ~S.isConnected || isempty(S.sp), return; end
        S.streamTimer = timer('ExecutionMode','fixedSpacing','Period',STREAM_PERIOD,...
            'BusyMode','drop','TimerFcn',@onStreamTick);
        start(S.streamTimer);
    end

    function stopStream()
        try
            if ~isempty(S.streamTimer) && isvalid(S.streamTimer)
                stop(S.streamTimer); delete(S.streamTimer);
            end
        catch, end
        S.streamTimer = [];
    end

    function onStreamTick(~,~)
        if S.inTxn || ~S.isConnected || isempty(S.sp), return; end
        try
            % Drain serial port into buffer
            nAv = S.sp.NumBytesAvailable;
            if nAv > 0
                raw = read(S.sp, nAv, "uint8");
                S.rxBuf = [S.rxBuf; uint8(raw(:))];
            end

            if ~S.streamOn, return; end

            % Process all complete frames
            nFrames = floor(numel(S.rxBuf) / FRAME_SZ);
            if nFrames == 0, return; end

            % Skip stale frames if too many accumulated
            if nFrames > 5
                logMsg(sprintf("⚠ Backlog: %d frames acumulados, descartando.", nFrames-1));
                S.rxBuf = S.rxBuf((nFrames-1)*FRAME_SZ+1:end);
                nFrames = 1;
            end

            nProc = 0;
            while floor(numel(S.rxBuf)/FRAME_SZ) > 0
                frame    = S.rxBuf(1:FRAME_SZ);
                S.rxBuf  = S.rxBuf(FRAME_SZ+1:end);
                parseFrame(frame);
                nProc = nProc + 1;
            end

            % Auto-stop check
            if S.autoStopArmed && S.framesTotal >= S.autoStopN
                S.autoStopArmed = false;
                logMsg(sprintf("Auto-stop: %d frames alcanzados.", S.framesTotal));
                onStop();
                return;
            end

            if nProc > 0
                updatePlots();
                updateDataInfo();
                drawnow limitrate;
            end

        catch e
            S.rxBuf = uint8([]);
            logMsg("stream WARN: " + string(e.message));
        end
    end

    function parseFrame(frame)
        u1 = double(typecast(frame(1:4),  'single'));
        y1 = double(typecast(frame(5:8),  'single'));
        u2 = double(typecast(frame(9:12), 'single'));
        y2 = double(typecast(frame(13:16),'single'));

        % Reject clearly invalid (NaN / Inf)
        if ~isfinite(u1), u1=0; end
        if ~isfinite(y1), y1=0; end
        if ~isfinite(u2), u2=0; end
        if ~isfinite(y2), y2=0; end

        n = S.framesTotal + 1;
        S.nVec(end+1,1)  = n;
        S.u1Vec(end+1,1) = u1;
        S.y1Vec(end+1,1) = y1;
        S.u2Vec(end+1,1) = u2;
        S.y2Vec(end+1,1) = y2;
        S.framesTotal    = n;

        % Trim circular buffer
        if numel(S.nVec) > MAX_PTS
            k = numel(S.nVec) - MAX_PTS + 1;
            S.nVec  = S.nVec(k:end);
            S.u1Vec = S.u1Vec(k:end);  S.y1Vec = S.y1Vec(k:end);
            S.u2Vec = S.u2Vec(k:end);  S.y2Vec = S.y2Vec(k:end);
        end
    end

%% ── Data management ──────────────────────────────────────────────────────────
    function onExport(~,~)
        if isempty(S.nVec), uialert(fig,'No hay datos.','Export'); return; end
        [f,p] = uiputfile('*.mat','Exportar datos');
        if isequal(f,0), return; end
        try
            data.n  = S.nVec;
            data.u1 = S.u1Vec;  data.y1 = S.y1Vec;
            data.u2 = S.u2Vec;  data.y2 = S.y2Vec;
            save(fullfile(p,f), '-struct','data');
            logMsg("Exportado: " + string(f));
        catch e, logMsg("Export FAIL: "+string(e.message)); end
    end

    function onClear(~,~)
        S.nVec=[]; S.u1Vec=[]; S.y1Vec=[]; S.u2Vec=[]; S.y2Vec=[];
        S.framesTotal = 0;
        S.rxBuf = uint8([]);
        updatePlots(); updateDataInfo();
        logMsg("Datos borrados.");
    end

%% ── Close ────────────────────────────────────────────────────────────────────
    function onClose(~,~)
        try, stopStream(); catch, end
        try
            if S.isConnected && ~isempty(S.sp)
                try, flush(S.sp); catch, end
                delete(S.sp);
            end
        catch, end
        delete(fig);
    end

end % pendulo_gui2
