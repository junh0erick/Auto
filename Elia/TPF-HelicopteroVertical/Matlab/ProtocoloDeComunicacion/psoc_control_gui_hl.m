function psoc_control_gui_hl()
% GUI basada en UARTP High-Level API.
%
% Requiere en el path:
%   uartp_open, uartp_reset, uartp_setmode, uartp_send_coeffs, uartp_get_coeffs,
%   uartp_init, uartp_stop, uartp_make_tf, uartp_make_ss
%
% Streaming:
%   El PSoC manda frames binarios de 8 bytes: [u(float32) y(float32)] little-endian
%   cada N muestras (según c15).
%
% Nota:
%   El streaming reader es un timer que solo lee múltiplos de 8 bytes y,
%   si encuentra basura, descarta hasta realinear (buffer simple).


    S = struct();
    S.sp = [];
    S.isConnected = false;

    % --- Streaming state ---
    S.streamTimer  = [];
    S.t0           = [];
    S.nVec         = zeros(0,1);
    S.uVec         = zeros(0,1);
    S.yVec         = zeros(0,1);
    S.maxPoints    = 400000;
    S.streamRxBuf  = uint8([]);
    S.framesTotal  = 0;

    % --- Auto-stop state (frames) ---
    S.autoStopEnabled   = false;
    S.autoStopTarget    = 0;
    S.autoStopArmed     = false;
    S.autoStopStartBase = 0;
    S.autoStopPending   = false;
    S.autoStopReason    = "";

    % --- control de parseo ---
    S.streamParseEnabled = true;

    % --------------------
    % UI
    % --------------------
    fig = uifigure( ...
        'Name', sprintf('PSoC Control GUI by Elías Álvarez'), ...
        'Position', [80 80 1280 720] ...
    );
    fig.CloseRequestFcn = @onClose;

    % Plot
    ax = uiaxes(fig, 'Position', [20 220 820 480]);
    ax.XGrid = 'on';
    ax.YGrid = 'on';
    title(ax, 'Streaming u,y');
    xlabel(ax, 'frame index (telemetry frames)');
    ylabel(ax, 'value');

    hold(ax, 'on');
    hU = stairs(ax, nan, nan, 'DisplayName', 'u');
    hY = plot(ax,   nan, nan, 'DisplayName', 'y');
    legend(ax, 'show', 'Location', 'best');
    hold(ax, 'off');

    % Log
    txtLog = uitextarea(fig, 'Editable', 'off', 'Position', [20 20 820 180]);
    txtLog.Value = strings(0,1);

    % --------------------
    % Connection panel
    % --------------------
    pConn = uipanel(fig, 'Title', 'Connection', 'Position', [860 610 400 90]);

    uilabel(pConn, 'Text', 'COM:', 'Position', [10 35 35 22]);
    edtCom = uieditfield(pConn, 'text', 'Value', 'COM9', 'Position', [50 35 90 22]);

    edtBaud = uieditfield( ...
        pConn, 'numeric', 'Value', 921600, 'Limits', [1200 2000000], ...
        'Position', [150 35 90 22] ...
    );
    uilabel(pConn, 'Text', 'baud', 'Position', [245 35 40 22]);

    btnConnect = uibutton( ...
        pConn, 'Text', 'Connect', 'Position', [10 5 90 24], ...
        'ButtonPushedFcn', @onConnectToggle ...
    );

    lblStat = uilabel(pConn, 'Text', 'DISCONNECTED', 'Position', [110 5 280 24]);

    btnReset = uibutton( ...
        pConn, 'Text', 'Reset (r)', 'Position', [310 5 80 24], ...
        'ButtonPushedFcn', @onReset ...
    );

    % --------------------
    % Mode panel
    % --------------------
    pMode = uipanel(fig, 'Title', 'Mode', 'Position', [860 430 400 170]);

    ddType = uidropdown( ...
        pMode, 'Items', {'TF','SS','Open-loop'}, ...
        'Value', 'Open-loop', ...
        'Position', [10 120 120 24], ...
        'ValueChangedFcn', @(~,~) refreshVisibility() ...
    );

    ddObserver = uidropdown( ...
        pMode, 'Items', {'Predictor','Actual'}, ...
        'Position', [150 120 120 24], ...
        'Tooltip', 'Only applies for SS' ...
    );

    cbIntegrator = uicheckbox(pMode, 'Text', 'Integrator', 'Position', [290 120 110 24]);

    % --- N ---
    uilabel(pMode, 'Text', 'N', 'Position', [10 85 20 24]);
    edtN = uieditfield( ...
        pMode, 'numeric', 'Limits', [0 100000], 'RoundFractionalValues', 'on', ...
        'Value', 1, 'Position', [35 85 80 24] ...
    );

    % --- Fs (Hz) ---
    uilabel(pMode, 'Text', 'Fs (Hz)', 'Position', [130 85 50 24]);
    edtFs = uieditfield( ...
        pMode, 'numeric', 'Limits', [0.001 1e9], ...
        'Value', 1000, ...
        'Position', [180 85 90 24], ...
        'ValueChangedFcn', @onFsChanged ...
    );

    % --- Computed Period (solo UI) ---
    lblPeriod = uilabel(pMode, 'Text', 'Period=1500 counts', 'Position', [10 60 360 20]);

    btnSendMode = uibutton( ...
        pMode, 'Text', 'Send Mode (m)', 'Position', [280 85 110 24], ...
        'ButtonPushedFcn', @onSendMode ...
    );

    btnSendCoeffs = uibutton( ...
        pMode, 'Text', 'Send Coeffs (c)', 'Position', [10 25 140 28], ...
        'ButtonPushedFcn', @onSendCoeffs ...
    );

    cbVerify = uicheckbox( ...
        pMode, 'Text', 'Verify (t)', 'Value', true, 'Position', [160 30 120 24] ...
    );

    btnGetCoeffs = uibutton( ...
        pMode, 'Text', 'Get Coeffs (t)', 'Position', [280 25 110 28], ...
        'ButtonPushedFcn', @onGetCoeffs ...
    );

    % --------------------
    % Control panel
    % --------------------
    pRef = uipanel(fig, 'Title', 'Control', 'Position', [860 320 400 100]);

    uilabel(pRef, 'Text', 'u0 / ref', 'Position', [10 45 60 22]);
    edtU0 = uieditfield(pRef, 'numeric', 'Value', 1000, 'Position', [80 40 100 26]);

    btnStart = uibutton( ...
        pRef, 'Text', 'Start (i)', 'Position', [200 40 90 26], ...
        'ButtonPushedFcn', @onStart ...
    );

    btnStop = uibutton( ...
        pRef, 'Text', 'Stop (s)', 'Position', [300 40 80 26], ...
        'ButtonPushedFcn', @onStop ...
    );

    cbWaitBack = uicheckbox( ...
        pRef, 'Text', 'wait back to COMMAND', 'Value', true, ...
        'Position', [10 10 200 24] ...
    );

    % --- Auto-stop ---
    cbAutoStop = uicheckbox( ...
        pRef, 'Text', 'Auto-stop', 'Value', false, ...
        'Position', [220 10 80 24] ...
    );

    uilabel(pRef, 'Text', 'at frames', 'Position', [300 10 55 22]);
    edtAutoStopN = uieditfield( ...
        pRef, 'numeric', ...
        'Limits', [0 1e12], 'RoundFractionalValues', 'on', ...
        'Value', 0, ...
        'Position', [355 10 70 24], ...
        'Tooltip', '0=disabled. Counts TELEMETRY FRAMES.' ...
    );

    % --------------------
    % Data panel
    % --------------------
    pData = uipanel(fig, 'Title', 'Data (Export / Clear)', 'Position', [860 20 400 65]);

    btnExportMAT = uibutton( ...
        pData, 'Text', 'Export .mat', ...
        'Position', [10 10 120 28], ...
        'Tooltip', 'Save current n,u,y vectors to a MAT-file', ...
        'ButtonPushedFcn', @onExportMAT ...
    );

    btnClearData = uibutton( ...
        pData, 'Text', 'Clear data', ...
        'Position', [140 10 120 28], ...
        'Tooltip', 'Clear plotted data (does not affect device)', ...
        'ButtonPushedFcn', @onClearData ...
    );

    lblDataInfo = uilabel( ...
        pData, 'Text', 'n=0', ...
        'Position', [270 45 120 22] ...
    );
        % --- Plot scaling (solo visual) ---
    uilabel(pData, 'Text', 'u x', 'Position', [270 12 30 18]);
    edtScaleU = uieditfield( ...
        pData, 'numeric', 'Value', 0.01, ...
        'Limits', [-1e12 1e12], ...
        'Position', [300 9 70 22], ...
        'Tooltip', 'Escala visual para u (solo plot)', ...
        'ValueChangedFcn', @onScaleChanged ...
    );

    uilabel(pData, 'Text', 'y x', 'Position', [375 12 30 18]);
    edtScaleY = uieditfield( ...
        pData, 'numeric', 'Value', 1, ...
        'Limits', [-1e12 1e12], ...
        'Position', [405 9 70 22], ...
        'Tooltip', 'Escala visual para y (solo plot)', ...
        'ValueChangedFcn', @onScaleChanged ...
    );


    % --------------------
    % TF panel
    % --------------------
    pTF = uipanel(fig, 'Title', 'TF Coeffs', 'Position', [860 95 400 220]);

    uilabel(pTF, 'Text', 'Numerator b0..b5', 'Position', [10 175 200 18]);
    bEdt = gobjects(1,6);
    for i = 1:6
        bEdt(i) = uieditfield(pTF, 'text', 'Value', '0', 'Position', [10+63*(i-1) 150 60 22]);
    end

    uilabel(pTF, 'Text', 'Denominator a0..a5', 'Position', [10 115 200 18]);
    aEdt = gobjects(1,6);
    for i = 1:6
        aEdt(i) = uieditfield(pTF, 'text', 'Value', '0', 'Position', [10+63*(i-1) 90 60 22]);
    end
    aEdt(1).Value = '1';

    uilabel(pTF, 'Text', 'Reserved (c13..c16)', 'Position', [10 55 140 18]);
    edtC13 = uieditfield(pTF, 'numeric', 'Value', 0, 'Position', [10 30 80 22]);
    edtC14 = uieditfield(pTF, 'numeric', 'Value', 0, 'Position', [100 30 80 22]);
    uilabel(pTF, 'Text', 'c15=N, c16=Fs(Hz)', 'Position', [190 30 200 22]);

    % --------------------
    % SS panel
    % --------------------
    pSS = uipanel(fig, 'Title', 'SS Coeffs (2 estados)', 'Position', [860 95 400 220]);

    uilabel(pSS, 'Text', 'A (2x2)', 'Position', [10 175 60 18]);
    edtA11 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [70 172 55 22]);
    edtA12 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [130 172 55 22]);
    edtA21 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [70 145 55 22]);
    edtA22 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [130 145 55 22]);

    uilabel(pSS, 'Text', 'B (2)', 'Position', [200 175 40 18]);
    edtB1 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [240 172 55 22]);
    edtB2 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [300 172 55 22]);

    uilabel(pSS, 'Text', 'C (2)', 'Position', [200 145 40 18]);
    edtC1 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [240 142 55 22]);
    edtC2 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [300 142 55 22]);

    uilabel(pSS, 'Text', 'D', 'Position', [10 115 20 18]);
    edtD = uieditfield(pSS, 'text', 'Value', '0', 'Position', [35 112 55 22]);

    uilabel(pSS, 'Text', 'L (2)', 'Position', [100 115 40 18]);
    edtL1 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [140 112 55 22]);
    edtL2 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [200 112 55 22]);

    uilabel(pSS, 'Text', 'K (2)', 'Position', [10 85 40 18]);
    edtK1 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [50 82 55 22]);
    edtK2 = uieditfield(pSS, 'text', 'Value', '0', 'Position', [110 82 55 22]);

    uilabel(pSS, 'Text', 'Ki', 'Position', [200 85 20 18]);
    edtKi = uieditfield(pSS, 'text', 'Value', '0', 'Position', [225 82 60 22]);

    % init UI state
    onFsChanged();
    refreshVisibility();
    updateDataInfo();

    % =====================================================================
    % UI callbacks (Data)
    % =====================================================================
    function updateDataInfo()
        try
            lblDataInfo.Text = sprintf("n=%d", numel(S.nVec));
        catch
        end
    end

    function onExportMAT(~,~)
        try
            if isempty(S.nVec)
                uialert(fig, "No hay datos para exportar.", "Export");
                return;
            end

            [file, path] = uiputfile("*.mat", "Guardar datos (.mat)", "psoc_stream.mat");
            if isequal(file,0) || isequal(path,0)
                logMsg("Export cancelado.");
                return;
            end

            data = struct();
            data.n = S.nVec;
            data.u = S.uVec;
            data.y = S.yVec;
            data.framesTotal = S.framesTotal;
            
            data.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');

            try
                data.meta = struct();
                data.meta.com = string(edtCom.Value);
                data.meta.baud = double(edtBaud.Value);
                data.meta.type = string(ddType.Value);
                data.meta.mode = computeMode();
                data.meta.N_ui = double(edtN.Value);
                data.meta.Fs_ui = double(edtFs.Value);
                data.meta.u0 = double(edtU0.Value);
            catch
            end

            save(fullfile(path,file), "-struct", "data");
            logMsg("Export OK: " + string(fullfile(path,file)));
        catch e
            logMsg("Export FAIL: " + string(e.message));
            try uialert(fig, e.message, "Export error"); catch, end
        end
    end

    function onClearData(~,~)
        try
            S.nVec = zeros(0,1);
            S.uVec = zeros(0,1);
            S.yVec = zeros(0,1);
            S.streamRxBuf = uint8([]);
            S.framesTotal = 0;
            S.scaleU = 0.01;
            S.scaleY = 1.0;
            try edtScaleU.Value = 1; edtScaleY.Value = 1; catch, end
            applyPlotScaling();


            S.autoStopPending = false;
            S.autoStopReason  = "";
            S.autoStopStartBase = 0;
            S.autoStopArmed = false;

            set(hU, 'XData', nan, 'YData', nan);
            set(hY, 'XData', nan, 'YData', nan);
            drawnow limitrate;

            updateDataInfo();
            logMsg("Data cleared (plot + buffers).");
        catch e
            logMsg("Clear FAIL: " + string(e.message));
        end
    end

    % =====================================================================
    % Existing callbacks
    % =====================================================================
    function refreshVisibility()
        typ = ddType.Value;
        pTF.Visible = strcmp(typ, 'TF');
        pSS.Visible = strcmp(typ, 'SS');

        ddObserver.Enable   = strcmp(typ, 'SS');
        cbIntegrator.Enable = strcmp(typ, 'SS');
        btnSendCoeffs.Enable = true;

        pData.Visible = true;
    end

    function onFsChanged(~,~)
        try
            fs = double(edtFs.Value);
            if ~isfinite(fs) || fs <= 0
                lblPeriod.Text = "Fs inválida";
                return;
            end
            lblPeriod.Text = sprintf("Fs=%.6g Hz (PSoC calcula PeriodCounts)", fs);
        catch
            lblPeriod.Text = "Fs=?";
        end
    end

    function onConnectToggle(~,~)
        if ~S.isConnected
            com = strtrim(string(edtCom.Value));
            if com == ""
                logMsg("COM vacío.");
                return;
            end
            try
                S.sp = uartp_open(com, edtBaud.Value);
                S.isConnected = true;
                btnConnect.Text = "Disconnect";
                lblStat.Text = "CONNECTED: " + com;

                try flush(S.sp); catch, end

                S.streamParseEnabled = true;
                S.streamRxBuf = uint8([]);
                S.autoStopPending = false;
                S.autoStopArmed = false;

                logMsg("Connected to " + com);
                startStreaming();
            catch e
                logMsg("Connect error: " + string(e.message));
                stopStreaming();
            end
        else
            try
                stopStreaming();
                if ~isempty(S.sp)
                    try flush(S.sp); catch, end
                    clear S.sp;
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

    function onReset(~,~)
        if ~requireConn(); return; end
        try
            stopStreaming();
            try flush(S.sp); catch, end
            uartp_reset(S.sp);
            try flush(S.sp); catch, end

            S.streamParseEnabled = false;
            S.streamRxBuf = uint8([]);
            S.autoStopPending = false;
            S.autoStopArmed = false;

            startStreaming();
            logMsg("reset OK (parsing disabled until Start)");
        catch e
            logMsg("reset FAIL: " + string(e.message));
            startStreaming();
        end
    end

    function onSendMode(~,~)
        if ~requireConn(); return; end
        try
            stopStreaming();
            try flush(S.sp); catch, end

            mode = computeMode();
            uartp_setmode(S.sp, mode);

            try flush(S.sp); catch, end

            S.streamParseEnabled = false;
            S.streamRxBuf = uint8([]);
            S.autoStopPending = false;
            S.autoStopArmed = false;

            startStreaming();
            logMsg(sprintf("setmode OK: mode=%d (%s) (parsing disabled until Start)", mode, ddType.Value));
        catch e
            logMsg("setmode FAIL: " + string(e.message));
            startStreaming();
        end
    end

    function onSendCoeffs(~,~)
        if ~requireConn(); return; end
        typ = ddType.Value;

        try
            stopStreaming();
            try flush(S.sp); catch, end

            Nval = single(round(double(edtN.Value)));

            fs = double(edtFs.Value);
            if ~isfinite(fs) || fs <= 0
                error("Fs inválida (Hz).");
            end

            if strcmp(typ, 'TF')
                b = parse6(bEdt, 'b');
                a = parse6(aEdt, 'a');
                coeffs = uartp_make_tf(b, a);

                coeffs(13) = single(edtC13.Value);
                coeffs(14) = single(edtC14.Value);
                coeffs(15) = Nval;
                coeffs(16) = single(fs);

            elseif strcmp(typ, 'SS')
                A = [ ...
                    parse1(edtA11, 'A11'), parse1(edtA12, 'A12'); ...
                    parse1(edtA21, 'A21'), parse1(edtA22, 'A22')  ...
                ];
                B  = [parse1(edtB1, 'B1'); parse1(edtB2, 'B2')];
                C  = [parse1(edtC1, 'C1'); parse1(edtC2, 'C2')].';
                D  = parse1(edtD,  'D');
                L  = [parse1(edtL1, 'L1'); parse1(edtL2, 'L2')];
                K  = [parse1(edtK1, 'K1'); parse1(edtK2, 'K2')];
                Ki = parse1(edtKi, 'Ki');

                coeffs = uartp_make_ss(A, B, C, D, L, K, Ki);
                coeffs(15) = Nval;
                coeffs(16) = single(fs);

            else
                coeffs = zeros(16,1,'single');
                coeffs(15) = Nval;
                coeffs(16) = single(fs);
            end

            uartp_send_coeffs(S.sp, coeffs, cbVerify.Value);

            try flush(S.sp); catch, end

            S.streamParseEnabled = false;
            S.streamRxBuf = uint8([]);
            S.autoStopPending = false;
            S.autoStopArmed = false;

            startStreaming();

            logMsg(sprintf("coeffs OK (%s) N=%g Fs=%.6g Hz (PSoC calcula PeriodCounts) (parsing disabled until Start)", ...
                typ, double(Nval), fs));
        catch e
            logMsg("coeffs FAIL: " + string(e.message));
            startStreaming();
        end
    end

    function onGetCoeffs(~,~)
        if ~requireConn(); return; end

        try
            stopStreaming();
            try flush(S.sp); catch, end

            c = uartp_get_coeffs(S.sp);
            c = single(c(:));
            typ = ddType.Value;

            if strcmp(typ,'TF')
                num6 = c(1:6).';
                den6 = c(7:12).';
                res13 = c(13);
                res14 = c(14);
                N = c(15);
                fs = double(c(16));

                logMsg("t OK (TF)");
                logMsg(" num6 = " + vecfmt(num6));
                logMsg(" den6 = " + vecfmt(den6));
                logMsg(sprintf(" meta: res13=%g res14=%g N=%g Fs=%.6g Hz", ...
                    double(res13), double(res14), double(N), fs));

            elseif strcmp(typ,'SS')
                A = [c(1) c(2); c(3) c(4)];
                B = [c(5); c(6)];
                Cv = [c(7) c(8)];
                D = c(9);
                L = [c(10); c(11)];
                K = [c(12); c(13)];
                Ki = c(14);
                N = c(15);
                fs = double(c(16));

                logMsg("t OK (SS)");
                logMsg(" A = " + matfmt(A));
                logMsg(" B = " + vecfmt(B.'));
                logMsg(" C = " + vecfmt(Cv));
                logMsg(sprintf(" D = %g", double(D)));
                logMsg(" L = " + vecfmt(L.'));
                logMsg(" K = " + vecfmt(K.'));
                logMsg(sprintf(" Ki = %g", double(Ki)));
                logMsg(sprintf(" meta: N=%g Fs=%.6g Hz", double(N), fs));

            else
                N = c(15);
                fs = double(c(16));
                logMsg("t OK (Open-loop / raw).");
                logMsg(sprintf(" meta: N=%g Fs=%.6g Hz", double(N), fs));
            end

            try flush(S.sp); catch, end

            S.streamParseEnabled = false;
            S.streamRxBuf = uint8([]);
            S.autoStopPending = false;
            S.autoStopArmed = false;

            startStreaming();
        catch e
            logMsg("t FAIL: " + string(e.message));
            startStreaming();
        end
    end

    function onStart(~,~)
        if ~requireConn(); return; end
        try
            stopStreaming();
            try flush(S.sp); catch, end

            u0 = double(edtU0.Value);
            uartp_init(S.sp, u0);

            S.autoStopEnabled = logical(cbAutoStop.Value);
            tgt = round(double(edtAutoStopN.Value));
            if ~isfinite(tgt) || tgt < 0, tgt = 0; end

            S.autoStopTarget    = tgt;
            S.autoStopStartBase = S.framesTotal;
            S.autoStopPending   = false;
            S.autoStopReason    = "";

            S.autoStopArmed = (S.autoStopEnabled && S.autoStopTarget > 0);
            if S.autoStopArmed
                logMsg(sprintf("Auto-stop ARMED: stop after %d frames (from Start)", S.autoStopTarget));
            else
                logMsg("Auto-stop OFF");
            end

            S.streamParseEnabled = true;
            S.streamRxBuf = uint8([]);

            try flush(S.sp); catch, end
            startStreaming();

            logMsg(sprintf("init OK: u0/ref=%.6g (CONTROL) (parsing enabled)", u0));
        catch e
            logMsg("init FAIL: " + string(e.message));
            S.streamParseEnabled = true;
            startStreaming();
        end
    end

    function onStop(~,~)
        if ~requireConn(); return; end
        try
            S.autoStopPending = false;
            S.autoStopArmed = false;

            S.streamParseEnabled = false;
            stopStreaming();
            try flush(S.sp); catch, end

            uartp_stop(S.sp, cbWaitBack.Value);

            try flush(S.sp); catch, end
            S.streamRxBuf = uint8([]);

            startStreaming();
            logMsg("stop OK (parsing disabled; UART bytes discarded)");
        catch e
            logMsg("stop FAIL: " + string(e.message));
            S.streamParseEnabled = false;
            stopStreaming();
        end
    end

    % =====================================================================
    % Helpers
    % =====================================================================
        function onScaleChanged(~,~)
        % UI -> estado + redibujo
        su = double(edtScaleU.Value);
        sy = double(edtScaleY.Value);

        if ~isfinite(su) || su == 0, su = 1; end
        if ~isfinite(sy) || sy == 0, sy = 1; end

        S.scaleU = su;
        S.scaleY = sy;

        applyPlotScaling();
        drawnow limitrate;
    end

    function applyPlotScaling()
        % Solo usa S.scaleU / S.scaleY
        su = double(S.scaleU);
        sy = double(S.scaleY);

        if ~isfinite(su) || su == 0, su = 1; end
        if ~isfinite(sy) || sy == 0, sy = 1; end

        if isempty(S.nVec)
            set(hU, 'XData', nan, 'YData', nan);
            set(hY, 'XData', nan, 'YData', nan);
        else
            set(hU, 'XData', S.nVec, 'YData', S.uVec * su);
            set(hY, 'XData', S.nVec, 'YData', S.yVec * sy);
        end
    end

    
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
        if numel(v) > 400
            v = v(end-400:end);
        end

        txtLog.Value = v;
        drawnow limitrate;
    end

    function mode = computeMode()
        typ = ddType.Value;

        if strcmp(typ,'TF'),        mode = 0; return; end
        if strcmp(typ,'Open-loop'), mode = 5; return; end

        isAct = strcmp(ddObserver.Value,'Actual');
        hasI  = cbIntegrator.Value;

        if ~isAct && ~hasI
            mode = 1;
        elseif isAct && ~hasI
            mode = 2;
        elseif ~isAct && hasI
            mode = 3;
        else
            mode = 4;
        end
    end

    function v = parse6(edts, name)
        v = zeros(1,6);
        for kk = 1:6
            v(kk) = parse1(edts(kk), sprintf("%s[%d]", name, kk-1));
        end
    end

    function v = parse1(edt, name)
        s = strtrim(string(edt.Value));
        if s == ""
            uialert(fig, "Campo vacío: " + name, 'Input error');
            error("Campo vacío: %s", name);
        end

        v = str2double(s);
        if ~isfinite(v)
            uialert(fig, "Número inválido en " + name + ": '" + s + "'", 'Input error');
            error("Número inválido: %s", name);
        end
    end

    function s = vecfmt(x)
        x = double(x(:).');
        s = "[" + strjoin(string(x), " ") + "]";
    end

    function s = matfmt(M)
        M = double(M);
        s = sprintf("[[%g %g]; [%g %g]]", M(1,1), M(1,2), M(2,1), M(2,2));
        s = string(s);
    end

    % =====================================================================
    % Streaming (nested)
    % =====================================================================
    function startStreaming()
        stopStreaming();

        if ~S.isConnected || isempty(S.sp)
            return;
        end

        if isempty(S.t0)
            S.t0 = tic; %#ok<NASGU>
        end

        S.streamTimer = timer( ...
            'ExecutionMode', 'fixedSpacing', ...
            'Period', 0.02, ...
            'TimerFcn', @onStreamTick, ...
            'BusyMode', 'drop' ...
        );
        start(S.streamTimer);

        if S.streamParseEnabled
            logMsg("Streaming reader ON (parsing ENABLED) (frames 8B: u,y)");
        else
            logMsg("Streaming reader ON (parsing DISABLED) (discarding UART bytes)");
        end
    end

    function stopStreaming()
        try
            if ~isempty(S.streamTimer) && isvalid(S.streamTimer)
                stop(S.streamTimer);
                delete(S.streamTimer);
            end
        catch
        end
        S.streamTimer = [];
    end

    function onStreamTick(~,~)
        if ~S.isConnected || isempty(S.sp)
            return;
        end
        sp = S.sp;

        try
            if S.autoStopPending
                S.autoStopPending = false;
                doAutoStopNow();
                return;
            end

            nAvail = sp.NumBytesAvailable;
            if nAvail <= 0
                return;
            end

            raw = read(sp, nAvail, "uint8");
            if isempty(raw)
                return;
            end

            if ~S.streamParseEnabled
                S.streamRxBuf = uint8([]);
                return;
            end

            S.streamRxBuf = [S.streamRxBuf; uint8(raw(:))];

            n = numel(S.streamRxBuf);
            nFrames = floor(n/8);
            if nFrames <= 0
                return;
            end

            take = 8*nFrames;
            blk = S.streamRxBuf(1:take);
            S.streamRxBuf = S.streamRxBuf(take+1:end);

            frames = reshape(blk, 8, []).';

            u_u32 = uint32(frames(:,1)) ...
                + bitshift(uint32(frames(:,2)), 8) ...
                + bitshift(uint32(frames(:,3)),16) ...
                + bitshift(uint32(frames(:,4)),24);

            y_u32 = uint32(frames(:,5)) ...
                + bitshift(uint32(frames(:,6)), 8) ...
                + bitshift(uint32(frames(:,7)),16) ...
                + bitshift(uint32(frames(:,8)),24);

            u = typecast(u_u32, 'single');
            y = typecast(y_u32, 'single');

            idx0 = S.framesTotal;
            nIdx = (idx0 + (1:numel(u))).';

            S.nVec = [S.nVec; nIdx];
            S.uVec = [S.uVec; double(u)];
            S.yVec = [S.yVec; double(y)];

            if numel(S.nVec) > S.maxPoints
                k0 = numel(S.nVec) - S.maxPoints + 1;
                S.nVec = S.nVec(k0:end);
                S.uVec = S.uVec(k0:end);
                S.yVec = S.yVec(k0:end);
            end

            applyPlotScaling();


            S.framesTotal = S.framesTotal + numel(u);
            updateDataInfo();

            if S.autoStopArmed
                framesSinceStart = S.framesTotal - S.autoStopStartBase;
                if framesSinceStart >= S.autoStopTarget
                    S.autoStopArmed   = false;
                    S.autoStopPending = true;
                    S.autoStopReason  = sprintf("framesSinceStart=%d >= %d", framesSinceStart, S.autoStopTarget);
                    logMsg("Auto-stop TRIGGER (deferred): " + S.autoStopReason);
                end
            end

            if mod(S.framesTotal, 50) == 0
                logMsg(sprintf("stream: framesTotal=%d (último u=%.4g y=%.4g)", ...
                    S.framesTotal, S.uVec(end), S.yVec(end)));
            end

            drawnow limitrate;

        catch e
            S.streamRxBuf = uint8([]);
            try flush(sp); catch, end
            logMsg("stream WARN: " + string(e.message));
        end
    end

    function doAutoStopNow()
        if ~requireConn()
            return;
        end

        try
            S.streamParseEnabled = false;
            S.streamRxBuf = uint8([]);

            stopStreaming();
            try flush(S.sp); catch, end

            logMsg("Auto-stop: sending STOP... (" + S.autoStopReason + ")");
            uartp_stop(S.sp, cbWaitBack.Value);

            try flush(S.sp); catch, end
            S.streamRxBuf = uint8([]);

            startStreaming();
            logMsg("Auto-stop: stop OK (parsing disabled; draining UART).");

        catch e
            logMsg("Auto-stop: stop FAIL: " + string(e.message));
            S.streamParseEnabled = false;
            try startStreaming(); catch, end
        end
    end

    function onClose(~,~)
        try
            stopStreaming();
            if S.isConnected && ~isempty(S.sp)
                try flush(S.sp); catch, end
                clear S.sp;
            end
        catch
        end
        delete(fig);
    end
end

