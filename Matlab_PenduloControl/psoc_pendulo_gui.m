function psoc_pendulo_gui()
%PSOC_PENDULO_GUI  Visualización y testeo del péndulo invertido.
%
% CÓMO USAR:
%  1. En el workspace de MATLAB diseñá los controladores:
%       C_in  = pid(Kp, Ki, Kd, Ts_inner)     → lazo interno
%       C_out = tf([b0 b1],[a0 a1], Ts_outer)  → lazo externo
%       % o  ss(A,B,C,D, Ts) para ambos
%  2. Abrí la GUI y ARRASTRÁ cada objeto a su panel.
%  3. Para SS: ingresá K, L y Ki en los campos de cada lazo.
%  4. Conectá al PSoC, verificá que N sea entero, enviá Ts y arrancá.
%
% PROTOCOLO UART:
%   PSoC → MATLAB : [theta int32 LE (4B)][delta_omega int16 LE (2B)] = 6 B/ciclo
%   MATLAB → PSoC : [u_pwm int16 LE (2B)] = 2 B/ciclo
%   STOP          : [0x00 0x00 0x73]
%
% SESIONES: se guardan en  ./sesiones/   (LAST_SESSION.mat = auto-save)

%% ── Constantes ──────────────────────────────────────────────────────────────
MOTOR_CPR      = 1040;   % 4 × 260 PPR (QuadDec x4)
FRAME_SZ       = 6;
STEP_TIMEOUT_S = 0.5;
MAX_RETRIES    = 50;
AUTOSAVE_S     = 30;
PWM_MAX        = 1264;
MAX_STATES     = 4;

SESSIONS_DIR = fullfile(fileparts(mfilename('fullpath')), 'sesiones');
if ~isfolder(SESSIONS_DIR), mkdir(SESSIONS_DIR); end

%% ── Estado global ────────────────────────────────────────────────────────────
S = struct();
S.sp = []; S.isConnected = false;
S.streamTimer = []; S.streamEnabled = false; S.inTxn = false;
S.rxBuf = uint8([]);

% Datos registrados
S.t_vec         = zeros(0,1);
S.theta_vec     = zeros(0,1);
S.omega_vec     = zeros(0,1);
S.u_vec         = zeros(0,1);
S.u_unsat_vec   = zeros(0,1);
S.theta_sim_vec = zeros(0,1);
S.omega_sim_vec = zeros(0,1);
S.omega_ref_vec = zeros(0,1);
S.x_hat_mat     = nan(0,MAX_STATES);
S.framesTotal   = 0;
S.maxPoints     = 500000;

% Controladores parseados (struct interno)
S.ctrl_inner = [];   % resultado de parseController()
S.ctrl_outer = [];
S.model_inner = [];  % modelo de sim lazo interno (ss struct)
S.model_outer = [];  % modelo de sim lazo externo (ss struct)
S.model_scale_i = 1.0;  % escala entrada modelo inner (ej: 12/1264 si modelo en V)
S.model_scale_o = 1.0;  % escala entrada modelo outer

% Runtime
S.ctrl_state = [];
S.inner_ctr  = 0;
S.omega_ref  = 0;
S.sim_state_i = [];   % sim lazo interno
S.sim_state_o = [];   % sim lazo externo
S.Ts_inner   = 0;
S.Ts_outer   = 0;
S.N          = 0;
S.innerOnly  = false; % true = solo lazo interno (test motor)

% Parámetros operativos
S.sat_min     = -PWM_MAX;
S.sat_max     =  PWM_MAX;
S.last_u_pwm  = 0;
S.PENDULO_CPR = 10000;
S.u_unit      = 'PWM';

S.lastSessionPath = fullfile(SESSIONS_DIR, 'LAST_SESSION.mat');
S.lastSaveTime    = tic;
S.lastPlotTime    = tic;

%% ── Figura ───────────────────────────────────────────────────────────────────
FIG_W = 1400; FIG_H = 960;
fig = uifigure('Name','Péndulo Invertido — PSoC Control GUI', ...
               'Position',[30 30 FIG_W FIG_H]);
fig.CloseRequestFcn = @onClose;

%% ── Axes (izquierda) ─────────────────────────────────────────────────────────
AX_X = 8; AX_W = 910;
%        [x      y     w      h  ]
ax_th = uiaxes(fig,'Position',[AX_X  653  AX_W  302]);
ax_om = uiaxes(fig,'Position',[AX_X  437  AX_W  208]);
ax_u  = uiaxes(fig,'Position',[AX_X  221  AX_W  208]);
ax_st = uiaxes(fig,'Position',[AX_X    5  AX_W  208]);

setupAxis(ax_th,'θ péndulo [rad]',       'muestra','θ [rad]');
setupAxis(ax_om,'Velocidad motor ω [rad/s]','muestra','ω [rad/s]');
setupAxis(ax_u, 'Esfuerzo u',            'muestra','u');
setupAxis(ax_st,'Estados estimados x̂  (SS)','muestra','valor');

hold(ax_th,'on');
hTh_r = plot(ax_th,nan,nan,'b-', 'LineWidth',1.2,'DisplayName','θ real');
hTh_s = plot(ax_th,nan,nan,'r--','LineWidth',0.9,'DisplayName','θ sim');
legend(ax_th,'show','Location','best'); hold(ax_th,'off');

hold(ax_om,'on');
hOm_r  = plot(ax_om,nan,nan,'b-', 'LineWidth',1.0,'DisplayName','ω real');
hOm_s  = plot(ax_om,nan,nan,'r--','LineWidth',0.8,'DisplayName','ω sim');
hOm_rf = plot(ax_om,nan,nan,'g--','LineWidth',0.8,'DisplayName','ω ref');
legend(ax_om,'show','Location','best'); hold(ax_om,'off');

hold(ax_u,'on');
hU_s   = stairs(ax_u,nan,nan,'k-',  'LineWidth',1.1,'DisplayName','u aplicado');
hU_us  = stairs(ax_u,nan,nan,':',   'Color',[0.5 0.5 0.5 0.35],'LineWidth',0.8,'DisplayName','u sin sat');
hSatHi = yline(ax_u, S.sat_max,'r--','LineWidth',0.7,'DisplayName','sat max');
hSatLo = yline(ax_u, S.sat_min,'r--','LineWidth',0.7,'DisplayName','sat min');
legend(ax_u,'show','Location','best'); hold(ax_u,'off');

hold(ax_st,'on');
ST_CLR = {'b','r',[0 0.6 0],'m'};
hSt = gobjects(MAX_STATES,1);
for ii = 1:MAX_STATES
    hSt(ii) = plot(ax_st,nan,nan,'-','Color',ST_CLR{ii},'LineWidth',0.8,...
                   'DisplayName',sprintf('x̂_%d',ii));
end
legend(ax_st,'show','Location','best'); hold(ax_st,'off');

%% ── Paneles derecha ──────────────────────────────────────────────────────────
RX = 930; RW = 460;
% y calculada desde arriba hacia abajo con gap de 4px entre paneles.

% ─ 1. Conexión  y=866, h=89 ─────────────────────────────────────────────────
pConn = uipanel(fig,'Title','Conexión','Position',[RX 866 RW 89]);
uilabel(pConn,'Text','Puerto:',   'Position',[  8 52 48 22]);
edtCom  = uieditfield(pConn,'text','Value','COM4','Position',[58 52 82 22]);
uilabel(pConn,'Text','Baud:',     'Position',[148 52 40 22]);
edtBaud = uieditfield(pConn,'numeric','Value',115200,'Limits',[1200 4e6],'Position',[190 52 105 22]);
btnConn  = uibutton(pConn,'Text','Conectar','Position',[8 18 100 28],...
    'BackgroundColor',[0.2 0.62 0.2],'FontColor','w','ButtonPushedFcn',@onConnectToggle);
lblStat  = uilabel(pConn,'Text','Desconectado','Position',[115 18 200 28]);
btnReset = uibutton(pConn,'Text','Reset (r)','Position',[320 18 90 28],'ButtonPushedFcn',@onReset);

% ─ 2. Lazo Interno  y=687, h=175 ────────────────────────────────────────────
pInner = uipanel(fig,'Title','Lazo Interno  (motor)','Position',[RX 687 RW 175]);

lblInfI     = uilabel(pInner,'Text','controlador: —', ...
    'Position',[8 148 RW-20 20],'FontWeight','bold');
lblModInfoI = uilabel(pInner,'Text','modelo sim:   —', ...
    'Position',[8 128 200 18],'FontColor',[0.45 0.45 0.45],'FontSize',10);
uilabel(pInner,'Text','escala u→mdl:','Position',[210 128 82 18],'FontSize',10);
edtMdlScaleI = uieditfield(pInner,'numeric','Value',1.0,'Limits',[1e-12 1e12], ...
    'Position',[294 128 82 18],'FontSize',10, ...
    'Tooltip','Multiplicador del esfuerzo antes de entrar al modelo de sim. Si modelo en V: 12/1264 ≈ 0.0095', ...
    'ValueChangedFcn',@(~,~) onMdlScaleChanged('i'));
btnLdI  = uibutton(pInner,'Text','Cargar  Controlador_inner  …', ...
    'Position',[8 92 RW-20 30],'ButtonPushedFcn',@(~,~) loadByName('inner'));
btnLdMI = uibutton(pInner,'Text','Cargar  Modelo_inner  …', ...
    'Position',[8 56 RW-20 30],'ButtonPushedFcn',@(~,~) loadByName('inner_model'));

uilabel(pInner,'Text','ω ref [rad/s]:', ...
    'Position',[8 24 95 22]);
edtOmegaRef = uieditfield(pInner,'numeric','Value',0, ...
    'Limits',[-1e4 1e4],'Position',[106 24 90 22]);
btnSetRef = uibutton(pInner,'Text','Confirmar ω ref', ...
    'Position',[202 22 RW-210 26], ...
    'ButtonPushedFcn',@(~,~) setfield_omegaref(edtOmegaRef.Value));

% ─ 3. Lazo Externo  y=508, h=175 ────────────────────────────────────────────
pOuter = uipanel(fig,'Title','Lazo Externo  (péndulo)','Position',[RX 508 RW 175]);

cbOuterEn = uicheckbox(pOuter,'Text','Habilitado  (desmarcar = solo lazo interno)', ...
    'Value',true,'Position',[8 148 RW-20 20], ...
    'ValueChangedFcn',@onOuterEnChanged);
lblInfO     = uilabel(pOuter,'Text','controlador: —', ...
    'Position',[8 122 RW-20 20],'FontWeight','bold');
lblModInfoO = uilabel(pOuter,'Text','modelo sim:   —', ...
    'Position',[8 100 200 18],'FontColor',[0.45 0.45 0.45],'FontSize',10);
uilabel(pOuter,'Text','escala u→mdl:','Position',[210 100 82 18],'FontSize',10);
edtMdlScaleO = uieditfield(pOuter,'numeric','Value',1.0,'Limits',[1e-12 1e12], ...
    'Position',[294 100 82 18],'FontSize',10, ...
    'Tooltip','Multiplicador del esfuerzo antes de entrar al modelo de sim. Si modelo en V: 12/1264 ≈ 0.0095', ...
    'ValueChangedFcn',@(~,~) onMdlScaleChanged('o'));
btnLdO  = uibutton(pOuter,'Text','Cargar  Controlador_out  …', ...
    'Position',[8 64 RW-20 30],'ButtonPushedFcn',@(~,~) loadByName('outer'));
btnLdMO = uibutton(pOuter,'Text','Cargar  Modelo_out  …', ...
    'Position',[8 26 RW-20 30],'ButtonPushedFcn',@(~,~) loadByName('outer_model'));

% ─ 4. Muestreo  y=436, h=68 ─────────────────────────────────────────────────
pSamp = uipanel(fig,'Title','Muestreo  (detectado automáticamente)','Position',[RX 436 RW 68]);
lblTsInf = uilabel(pSamp,'Text','Ts_inner: —    Ts_outer: —    N: —','Position',[8 34 320 20]);
lblTsVal = uilabel(pSamp,'Text','','Position',[330 34 120 20],'FontWeight','bold');
btnSendTs = uibutton(pSamp,'Text','Enviar Ts al PSoC','Position',[8 8 155 22],'ButtonPushedFcn',@onSendTs);

% ─ 5. Saturación  y=358, h=74 ───────────────────────────────────────────────
pSat = uipanel(fig,'Title','Saturación del esfuerzo  [PWM: −1264 … +1264]','Position',[RX 358 RW 74]);
uilabel(pSat,'Text','sat_min:','Position',[ 8 34 58 22]);
edtSmin = uieditfield(pSat,'numeric','Value',S.sat_min,'Limits',[-1264 0],...
    'Position',[68 34 90 22],'ValueChangedFcn',@onSatChanged);
uilabel(pSat,'Text','sat_max:','Position',[170 34 58 22]);
edtSmax = uieditfield(pSat,'numeric','Value',S.sat_max,'Limits',[0 1264],...
    'Position',[230 34 90 22],'ValueChangedFcn',@onSatChanged);
uilabel(pSat,'Text','(el PSoC aplica su propia saturación de seguridad adicional)',...
    'Position',[8 8 444 22],'FontColor',[0.55 0.55 0.55],'FontSize',10);

% ─ 6. Control  y=300, h=54 ──────────────────────────────────────────────────
pCtrl = uipanel(fig,'Title','Control','Position',[RX 300 RW 54]);
btnStart = uibutton(pCtrl,'Text','▶  Iniciar','Position',[8 8 130 30],...
    'BackgroundColor',[0.12 0.62 0.12],'FontColor','w','FontSize',13,...
    'FontWeight','bold','ButtonPushedFcn',@onStart);
btnStop  = uibutton(pCtrl,'Text','■  Detener','Position',[146 8 120 30],...
    'BackgroundColor',[0.78 0.1 0.1],'FontColor','w','FontSize',13,...
    'FontWeight','bold','ButtonPushedFcn',@onStop);
lblCtrlSt = uilabel(pCtrl,'Text','Detenido','Position',[274 8 110 30],'FontWeight','bold');
lblDataInf = uilabel(pCtrl,'Text','n=0','Position',[385 8 68 30],'HorizontalAlignment','right');

% ─ 7. Visualización  y=163, h=133 ───────────────────────────────────────────
pVis = uipanel(fig,'Title','Visualización','Position',[RX 163 RW 133]);
uilabel(pVis,'Text','Unidad u:','Position',[8 100 65 22]);
ddUUnit = uidropdown(pVis,'Items',{'PWM (cuentas)','Voltios (aprox)'},'Value','PWM (cuentas)',...
    'Position',[76 100 145 22],'ValueChangedFcn',@onUUnitChanged);
uilabel(pVis,'Text','Mostrar:','Position',[8 76 58 22]);
cbTh_r  = uicheckbox(pVis,'Text','θ real',    'Value',true,'Position',[  8 54 85 20],'ValueChangedFcn',@updateVis);
cbTh_s  = uicheckbox(pVis,'Text','θ sim',     'Value',true,'Position',[ 96 54 80 20],'ValueChangedFcn',@updateVis);
cbOm_r  = uicheckbox(pVis,'Text','ω real',    'Value',true,'Position',[180 54 75 20],'ValueChangedFcn',@updateVis);
cbOm_s  = uicheckbox(pVis,'Text','ω sim',     'Value',true,'Position',[258 54 70 20],'ValueChangedFcn',@updateVis);
cbOm_rf = uicheckbox(pVis,'Text','ω ref',     'Value',true,'Position',[330 54 75 20],'ValueChangedFcn',@updateVis);
cbU_s   = uicheckbox(pVis,'Text','u aplicado','Value',true,'Position',[  8 30 95 20],'ValueChangedFcn',@updateVis);
cbU_us  = uicheckbox(pVis,'Text','u sin sat', 'Value',true,'Position',[ 108 30 90 20],'ValueChangedFcn',@updateVis);
cbSatLn = uicheckbox(pVis,'Text','límites sat','Value',true,'Position',[ 202 30 95 20],'ValueChangedFcn',@updateVis);
cbStates= uicheckbox(pVis,'Text','estados x̂','Value',true,'Position',[  8  8 95 20],'ValueChangedFcn',@updateVis);
updateVis();

% ─ 8. Sesiones  y=81, h=78 ──────────────────────────────────────────────────
pSess = uipanel(fig,'Title','Sesiones y datos','Position',[RX 81 RW 78]);
btnSaveSess = uibutton(pSess,'Text','Guardar sesión…','Position',[  8 42 142 26],'ButtonPushedFcn',@onSaveSess);
btnLoadSess = uibutton(pSess,'Text','Cargar sesión…', 'Position',[156 42 142 26],'ButtonPushedFcn',@onLoadSess);
btnLoadLast = uibutton(pSess,'Text','Última sesión',  'Position',[304 42 148 26],'ButtonPushedFcn',@onLoadLast);
btnExport   = uibutton(pSess,'Text','Exportar .mat', 'Position',[  8 10 135 26],'ButtonPushedFcn',@onExport);
btnClear    = uibutton(pSess,'Text','Borrar datos',  'Position',[150 10 115 26],'ButtonPushedFcn',@onClearData);
lblAutoSave = uilabel(pSess,'Text','Auto-save: —',   'Position',[272 10 180 26],...
    'FontColor',[0.5 0.5 0.5],'FontSize',10);

% ─ 9. Log  y=5, h=72 ────────────────────────────────────────────────────────
txtLog = uitextarea(fig,'Editable','off','Position',[RX 5 RW 72],'FontSize',9);
txtLog.Value = strings(0,1);

%% ── Helpers UI ───────────────────────────────────────────────────────────────
    function logMsg(msg)
        ts   = string(datestr(now,'HH:MM:SS.FFF'));
        line = "[" + ts + "] " + string(msg);
        v = txtLog.Value;
        if ~isstring(v), v = string(v); end
        v(end+1,1) = line;
        if numel(v) > 300, v = v(end-300:end); end
        txtLog.Value = v;
        drawnow limitrate;
    end

    function ok = reqConn()
        ok = S.isConnected && ~isempty(S.sp);
        if ~ok, logMsg("⚠ No conectado."); end
    end

    function setupAxis(ax, ttl, xl, yl)
        ax.XGrid = 'on'; ax.YGrid = 'on';
        ax.Title.String  = ttl;
        ax.XLabel.String = xl;
        ax.YLabel.String = yl;
    end

    function updateVis()
        hTh_r.Visible  = v2s(cbTh_r.Value);
        hTh_s.Visible  = v2s(cbTh_s.Value);
        hOm_r.Visible  = v2s(cbOm_r.Value);
        hOm_s.Visible  = v2s(cbOm_s.Value);
        hOm_rf.Visible = v2s(cbOm_rf.Value);
        hU_s.Visible   = v2s(cbU_s.Value);
        hU_us.Visible  = v2s(cbU_us.Value);
        hSatHi.Visible = v2s(cbSatLn.Value);
        hSatLo.Visible = v2s(cbSatLn.Value);
        for ii = 1:MAX_STATES
            hSt(ii).Visible = v2s(cbStates.Value);
        end
    end
    function s = v2s(b), if b, s='on'; else, s='off'; end, end

    function onSatChanged(~,~)
        S.sat_min  = edtSmin.Value;
        S.sat_max  = edtSmax.Value;
        hSatHi.Value = S.sat_max;
        hSatLo.Value = S.sat_min;
    end

    function onUUnitChanged(~,~)
        if strcmp(ddUUnit.Value,'Voltios (aprox)')
            S.u_unit = 'Volts';
            ax_u.Title.String  = 'Esfuerzo u [V]  (aprox. lineal)';
            ax_u.YLabel.String = 'u [V]';
        else
            S.u_unit = 'PWM';
            ax_u.Title.String  = 'Esfuerzo u [PWM]';
            ax_u.YLabel.String = 'u';
        end
        updatePlots();
    end

    function updateDataInfo()
        try
            n = numel(S.t_vec);
            t = 0; if n>0, t = S.t_vec(end); end
            lblDataInf.Text = sprintf('n=%d  %.1fs  u=%d', n, t, round(S.last_u_pwm));
        catch, end
    end

    % ── Ts validation ─────────────────────────────────────────────────────────
    function updateTsDisplay()
        if isempty(S.ctrl_inner)
            lblTsInf.Text = 'Ts_inner: —    Ts_outer: —    N: —';
            lblTsVal.Text = ''; return;
        end
        ti = S.ctrl_inner.Ts;
        if ti <= 0
            lblTsInf.Text = 'Ts_inner inválido';
            lblTsVal.Text = '✗'; lblTsVal.FontColor = [0.8 0 0]; return;
        end
        if S.innerOnly
            S.Ts_inner = ti; S.Ts_outer = ti; S.N = 1;
            lblTsInf.Text = sprintf('Ts_inner: %.1f ms    [solo lazo interno]', ti*1000);
            lblTsVal.Text = '✓  OK'; lblTsVal.FontColor = [0 0.55 0]; return;
        end
        if isempty(S.ctrl_outer)
            lblTsInf.Text = sprintf('Ts_inner: %.1f ms    Ts_outer: —    N: —', ti*1000);
            lblTsVal.Text = ''; return;
        end
        to  = S.ctrl_outer.Ts;
        if to <= 0
            lblTsInf.Text = 'Ts_outer inválido';
            lblTsVal.Text = '✗'; lblTsVal.FontColor = [0.8 0 0]; return;
        end
        N_f = to / ti;
        N_i = round(N_f);
        err = abs(N_f - N_i) / N_f;
        lblTsInf.Text = sprintf('Ts_inner: %.1f ms    Ts_outer: %.1f ms    N: %d', ...
                                ti*1000, to*1000, N_i);
        if err < 0.005 && N_i >= 1
            S.Ts_inner = ti; S.Ts_outer = to; S.N = N_i;
            lblTsVal.Text = '✓  OK'; lblTsVal.FontColor = [0 0.55 0];
        else
            lblTsVal.Text = sprintf('✗  N=%.2f (no entero)', N_f);
            lblTsVal.FontColor = [0.8 0 0];
        end
    end

%% ── Carga de controladores ───────────────────────────────────────────────────
    % Checkbox lazo externo
    function onOuterEnChanged(~,~)
        S.innerOnly = ~cbOuterEn.Value;
        en = cbOuterEn.Value;
        btnLdO.Enable      = en;
        btnLdMO.Enable     = en;
        edtOmegaRef.Enable = ~en;   % solo editable en modo solo-interno
        if en, S.omega_ref = 0; edtOmegaRef.Value = 0; end
        updateTsDisplay();
        logMsg(sprintf("Lazo externo %s.", iif(en,'habilitado','deshabilitado')));
    end

    function setfield_omegaref(val)
        S.omega_ref = val;
        logMsg(sprintf("ω ref = %.3f rad/s", val));
    end

    function onMdlScaleChanged(which)
        if which == 'i'
            S.model_scale_i = edtMdlScaleI.Value;
            logMsg(sprintf("Escala modelo inner = %.6g", S.model_scale_i));
        else
            S.model_scale_o = edtMdlScaleO.Value;
            logMsg(sprintf("Escala modelo outer = %.6g", S.model_scale_o));
        end
    end
    function s = iif(c,a,b), if c, s=a; else, s=b; end, end

    % Carga por nombre de variable en workspace
    function loadByName(which)
        switch which
            case 'inner',       prompt = 'Controlador_inner  (pid | tf | struct SS):';
            case 'inner_model', prompt = 'Modelo_inner  (tf | ss):';
            case 'outer',       prompt = 'Controlador_out  (pid | tf | struct SS):';
            case 'outer_model', prompt = 'Modelo_out  (tf | ss):';
            otherwise,          prompt = sprintf('Variable para  %s:', which);
        end
        answer = inputdlg(prompt, 'Cargar desde workspace', 1, {which});
        if isempty(answer) || strtrim(answer{1}) == "", return; end
        try
            obj = evalin('base', strtrim(answer{1}));
            loadController(obj, which);
        catch e
            logMsg("Cargar FAIL [" + which + "]: " + string(e.message));
        end
    end

    function loadController(obj, which)
        % ── Modelos de simulación ──────────────────────────────────────────
        if strcmp(which,'inner_model') || strcmp(which,'outer_model')
            m = parseModel(obj);
            if strcmp(which,'inner_model')
                S.model_inner = m;
                lblModInfoI.Text = sprintf('modelo sim:   %s  %dx%d  Ts=%.3g s', ...
                    m.orig_type, size(m.A,1), size(m.B,2), m.Ts);
                logMsg(sprintf("Modelo inner cargado: %s  Ts=%.4g s", m.orig_type, m.Ts));
            else
                S.model_outer = m;
                lblModInfoO.Text = sprintf('modelo sim:   %s  %dx%d  Ts=%.3g s', ...
                    m.orig_type, size(m.A,1), size(m.B,2), m.Ts);
                logMsg(sprintf("Modelo outer cargado: %s  Ts=%.4g s", m.orig_type, m.Ts));
            end
            return;
        end

        % ── Controladores ──────────────────────────────────────────────────
        ctrl = parseController(obj);
        if strcmp(which,'inner')
            S.ctrl_inner = ctrl;
            lblInfI.Text = sprintf('controlador: %s    Ts: %.4g s  (%.1f ms)', ...
                ctrl.type, ctrl.Ts, ctrl.Ts*1000);
            logMsg(sprintf("Controlador inner cargado: %s  Ts=%.4g s", ctrl.type, ctrl.Ts));
        else
            S.ctrl_outer = ctrl;
            lblInfO.Text = sprintf('controlador: %s    Ts: %.4g s  (%.1f ms)', ...
                ctrl.type, ctrl.Ts, ctrl.Ts*1000);
            logMsg(sprintf("Controlador outer cargado: %s  Ts=%.4g s", ctrl.type, ctrl.Ts));
        end
        updateTsDisplay();
    end

    % Parsear controlador: pid() | pidstd() | tf() | struct SS
    function ctrl = parseController(obj)
        if isa(obj, 'pid')
            Ts = obj.Ts;
            validateDiscrete(Ts, 'pid');
            ctrl.type = 'PID';
            ctrl.Kp   = obj.Kp;
            ctrl.Ki   = obj.Ki * Ts;   % continuo → acumulador discreto
            ctrl.Kd   = obj.Kd;
            ctrl.Ts   = Ts;

        elseif isa(obj, 'pidstd')
            % Forma estándar: u = Kp*(e + 1/Ti*∫e + Td*ė)
            % Ganancias paralelas equiv.: Ki_c = Kp/Ti,  Kd = Kp*Td
            Ts = obj.Ts;
            validateDiscrete(Ts, 'pidstd');
            Kp   = obj.Kp;
            Ti   = obj.Ti;
            Td   = obj.Td;
            Ki_c = 0;
            if isfinite(Ti) && Ti > 0, Ki_c = Kp / Ti; end
            ctrl.type = 'PID';
            ctrl.Kp   = Kp;
            ctrl.Ki   = Ki_c * Ts;   % continuo → acumulador discreto
            ctrl.Kd   = Kp * Td;
            ctrl.Ts   = Ts;

        elseif isa(obj, 'tf')
            Ts = obj.Ts;
            validateDiscrete(Ts, 'tf');
            [b, a] = tfdata(obj, 'v');
            ctrl.type = 'TF';
            ctrl.b    = b(:)';
            ctrl.a    = a(:)';
            ctrl.Ts   = Ts;

        elseif isstruct(obj) && isfield(obj,'sys') && isa(obj.sys,'ss')
            % struct SS: .sys(ss)  .K  .L(opt)  .Ki(opt)  .mode(opt)
            sys = obj.sys;
            Ts  = sys.Ts;
            validateDiscrete(Ts, 'ss-struct');
            if ~isfield(obj,'K')
                error("Struct SS: falta campo .K (ganancia de realimentación de estado).");
            end
            ctrl.type = 'SS';
            ctrl.A    = sys.A;
            ctrl.B    = sys.B;
            ctrl.C    = sys.C;
            ctrl.D    = sys.D;
            ctrl.Ts   = Ts;
            ctrl.K    = obj.K(:)';
            ctrl.L    = zeros(size(sys.A,1),1);
            if isfield(obj,'L'),    ctrl.L    = obj.L(:);      end
            ctrl.Ki   = 0;
            if isfield(obj,'Ki'),   ctrl.Ki   = obj.Ki;        end
            ctrl.mode = 'Actual';
            if isfield(obj,'mode'), ctrl.mode = obj.mode;      end

        else
            error("Tipo no soportado: %s.\nControlador: pid()|tf()|struct{.sys .K [.L .Ki .mode]}", class(obj));
        end
    end

    % Parsear modelo de simulación: tf() | ss()  → struct con matrices ss
    function m = parseModel(obj)
        if isa(obj,'tf')
            validateDiscrete(obj.Ts, 'tf-modelo');
            sys = ss(obj);
        elseif isa(obj,'ss')
            validateDiscrete(obj.Ts, 'ss-modelo');
            sys = obj;
        else
            error("Modelo: se espera tf() o ss() discretos. Recibido: %s", class(obj));
        end
        m.A = sys.A; m.B = sys.B; m.C = sys.C; m.D = sys.D;
        m.Ts = sys.Ts;
        m.orig_type = class(obj);
    end

    function validateDiscrete(Ts, tipo)
        if Ts <= 0
            error("%s es continuo (Ts=%.4g). Usá un objeto discreto con Ts > 0.", tipo, Ts);
        end
    end

%% ── UARTP Low-Level ──────────────────────────────────────────────────────────
    function ll_flush()
        if ~isempty(S.sp), try flush(S.sp); catch, end; end
    end

    function b = ll_readexact(n, tmo)
        if nargin < 2, tmo = STEP_TIMEOUT_S; end
        t0 = tic; buf = zeros(1,n,'uint8'); k = 0;
        while k < n
            if toc(t0) > tmo, error('Timeout (%d/%d bytes)', k, n); end
            av = S.sp.NumBytesAvailable;
            if av > 0
                m = min(av, n-k);
                tmp = read(S.sp, m, "uint8");
                buf(k+1:k+m) = tmp(:);
                k = k + m;
            else
                pause(0.001);
            end
        end
        b = buf(:);
    end

    function rsp = ll_cmd_wait(cmd, tmo)
        if nargin < 2, tmo = STEP_TIMEOUT_S; end
        valid = uint8(['R','K','S','!']);
        write(S.sp, uint8(cmd), "uint8");
        t0 = tic;
        while true
            if toc(t0) > tmo, error("Timeout cmd '%s'", cmd); end
            if S.sp.NumBytesAvailable > 0
                bb = read(S.sp,1,"uint8"); bb=bb(1);
                if any(bb == valid), rsp = bb; return; end
            else
                pause(0.001);
            end
        end
    end

    function ll_send_word4(w4)
        w4 = uint8(w4(:)); assert(numel(w4)==4);
        tries = 0;
        while true
            tries = tries+1;
            if tries > MAX_RETRIES, error('Max retries word4'); end
            write(S.sp, w4, "uint8");
            echo = ll_readexact(4);
            if isequal(echo(:), w4)
                write(S.sp, uint8('A'), "uint8");
            else
                write(S.sp, uint8('N'), "uint8");
            end
            conf = ll_readexact(1); conf = conf(1);
            if conf == uint8('A') && isequal(echo(:), w4), return; end
        end
    end

%% ── Protocolo HL ─────────────────────────────────────────────────────────────
    function uartp_reset()
        % Si el PSoC está en modo CONTROL, primero enviamos stop [0,0,'s']
        % para que vuelva a COMMAND antes de procesar 'r'.
        try
            write(S.sp, uint8([0, 0, uint8('s')]), "uint8");
            pause(0.08);
        catch, end
        ll_flush();
        rsp = ll_cmd_wait('r');
        if rsp ~= uint8('K'), error("reset: rsp=%c", char(rsp)); end
    end

    function uartp_set_fs(fs_hz)
        ll_flush();
        rsp = ll_cmd_wait('f');
        if rsp ~= uint8('R'), error("f: rsp=%c", char(rsp)); end
        ll_send_word4(typecast(single(fs_hz),'uint8'));
        fin = ll_readexact(1); fin = fin(1);
        if fin ~= uint8('K'), error("f: fin=%c", char(fin)); end
    end

    function uartp_start()
        ll_flush();
        rsp = ll_cmd_wait('i');
        if rsp ~= uint8('K'), error("i: rsp=%c", char(rsp)); end
    end

    function uartp_stop_cmd()
        try
            write(S.sp, uint8([0, 0, uint8('s')]), "uint8");
            ll_readexact(1, 1.0);
        catch, end
    end

%% ── Acciones principales ─────────────────────────────────────────────────────
    function onConnectToggle(~,~)
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
                logMsg("Conectado: " + com + " @ " + edtBaud.Value + " baud");
                startStreaming();
            catch e
                logMsg("Error conexión: " + string(e.message));
                S.sp = []; S.isConnected = false;
            end
        else
            stopStreaming();
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
            S.inTxn = true; stopStreaming(); ll_flush();
            uartp_reset();
            logMsg("Reset OK.");
        catch e
            logMsg("Reset FAIL: " + string(e.message));
        end
        S.inTxn = false; startStreaming();
    end

    function onSendTs(~,~)
        if ~reqConn(), return; end
        if S.Ts_inner <= 0
            logMsg("⚠ No hay Ts válido. Cargá ambos controladores primero.");
            return;
        end
        try
            S.inTxn = true; stopStreaming(); ll_flush();
            uartp_reset();
            uartp_set_fs(1.0 / S.Ts_inner);
            logMsg(sprintf("Ts enviado: Ts_inner=%.4f s  Fs_inner=%.2f Hz", ...
                S.Ts_inner, 1/S.Ts_inner));
        catch e
            logMsg("SendTs FAIL: " + string(e.message));
        end
        S.inTxn = false; startStreaming();
    end

    function onStart(~,~)
        if ~reqConn(), return; end
        if isempty(S.ctrl_inner)
            logMsg("⚠ Cargá el Controlador_inner primero."); return;
        end
        if ~S.innerOnly && isempty(S.ctrl_outer)
            logMsg("⚠ Cargá el Controlador_out, o habilitá solo lazo interno."); return;
        end
        if S.Ts_inner <= 0 || S.N < 1
            logMsg("⚠ Ts no válido. Verificá los controladores cargados."); return;
        end
        try
            S.inTxn = true; stopStreaming(); ll_flush();

            % Leer parámetros operativos actuales
            S.sat_min = edtSmin.Value;
            S.sat_max = edtSmax.Value;

            % Inicializar estado del controlador
            initCtrlState();

            % Protocolo: reset → Fs → start
            uartp_reset();
            uartp_set_fs(1.0 / S.Ts_inner);
            uartp_start();

            S.streamEnabled = true;
            S.rxBuf = uint8([]);
            S.inner_ctr = 0;
            S.omega_ref = edtOmegaRef.Value;

            lblCtrlSt.Text = '▶ CORRIENDO';
            lblCtrlSt.FontColor = [0 0.5 0];
            logMsg(sprintf("Control iniciado. Ts_inner=%.3f ms  N=%d  Fs=%.1f Hz", ...
                S.Ts_inner*1000, S.N, 1/S.Ts_inner));

        catch e
            logMsg("Start FAIL: " + string(e.message));
            S.streamEnabled = false;
        end
        S.inTxn = false; startStreaming();
    end

    function onStop(~,~)
        if ~reqConn(), return; end
        try
            S.inTxn = true; S.streamEnabled = false;
            stopStreaming(); ll_flush();
            uartp_stop_cmd();
            lblCtrlSt.Text = 'Detenido';
            lblCtrlSt.FontColor = [0 0 0];
            logMsg("Control detenido.");
            autoSave();
        catch e
            logMsg("Stop FAIL: " + string(e.message));
        end
        S.inTxn = false; startStreaming();
    end

%% ── Streaming ────────────────────────────────────────────────────────────────
    function startStreaming()
        stopStreaming();
        if ~S.isConnected || isempty(S.sp), return; end
        S.streamTimer = timer('ExecutionMode','fixedSpacing','Period',0.010,...
            'TimerFcn',@onStreamTick,'BusyMode','drop');
        start(S.streamTimer);
    end

    function stopStreaming()
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
            nAv = S.sp.NumBytesAvailable;
            if nAv > 0
                raw = read(S.sp, nAv, "uint8");
                S.rxBuf = [S.rxBuf; uint8(raw(:))];
            end

            if ~S.streamEnabled, return; end

            % Descartar frames viejos — procesar solo el más reciente
            nFrames = floor(numel(S.rxBuf) / FRAME_SZ);
            if nFrames == 0, return; end
            if nFrames > 1
                % Saltar al último frame completo
                S.rxBuf = S.rxBuf((nFrames-1)*FRAME_SZ + 1 : end);
            end
            frame   = S.rxBuf(1:FRAME_SZ);
            S.rxBuf = S.rxBuf(FRAME_SZ+1:end);
            processFrame(frame);
            nProc = 1;

            if nProc > 0 && toc(S.lastPlotTime) > 0.05
                S.lastPlotTime = tic;
                updatePlots();
                updateDataInfo();
                drawnow limitrate;
            end

            if toc(S.lastSaveTime) > AUTOSAVE_S
                S.lastSaveTime = tic;
                autoSave();
            end

        catch e
            S.rxBuf = uint8([]);
            logMsg("stream WARN: " + string(e.message));
        end
    end

    function processFrame(frame)
        Ts_i = S.Ts_inner;

        % Parsear
        theta_cnt     = typecast(frame(1:4),'int32');
        delta_om_cnt  = typecast(frame(5:6),'int16');

        % Convertir a físico
        theta_rad  = double(theta_cnt)   * (2*pi / S.PENDULO_CPR);
        omega_rads = double(delta_om_cnt) * (2*pi / MOTOR_CPR) / Ts_i;

        % Lazo interno (cada Ts_inner)
        [u_raw, S.ctrl_state] = runInnerStep(S.ctrl_state, S.omega_ref, omega_rads);

        % Lazo externo (cada N ciclos internos) — solo si habilitado
        if ~S.innerOnly
            S.inner_ctr = S.inner_ctr + 1;
            if S.inner_ctr >= S.N
                S.inner_ctr = 0;
                [omega_new, S.ctrl_state] = runOuterStep(S.ctrl_state, 0, theta_rad);
                S.omega_ref = omega_new;
            end
        end

        % Saturar y cuantizar
        u_sat = max(S.sat_min, min(S.sat_max, u_raw));
        u_pwm = int16(round(u_sat));

        % Enviar al PSoC
        write(S.sp, typecast(u_pwm,'uint8'), "uint8");
        S.last_u_pwm = double(u_pwm);

        % Diagnóstico periódico
        if mod(S.framesTotal, 100) == 0
            logMsg(sprintf("k=%d  ω=%.2f rad/s  ref=%.2f rad/s  u=%d PWM  (%.1f%%)", ...
                S.framesTotal, omega_rads, S.omega_ref, int16(u_pwm), ...
                double(u_pwm)/PWM_MAX*100));
        end

        % Simulación interna (modelo inner → omega_sim)
        omega_sim = nan;
        if ~isempty(S.sim_state_i)
            u_mi = double(u_pwm) * S.model_scale_i;
            [y_i, S.sim_state_i] = simStep(S.sim_state_i, u_mi);
            if ~isempty(y_i), omega_sim = y_i(1); end
        end

        % Simulación interna (modelo outer → theta_sim)
        theta_sim = nan;
        if ~isempty(S.sim_state_o)
            u_mo = double(u_pwm) * S.model_scale_o;
            [y_s, S.sim_state_o] = simStep(S.sim_state_o, u_mo);
            if ~isempty(y_s), theta_sim = y_s(1); end
        end

        % Estados para plot
        x_hat_k = nan(1,MAX_STATES);
        if isfield(S.ctrl_state,'inner') && isfield(S.ctrl_state.inner,'x_hat')
            xh = S.ctrl_state.inner.x_hat;
            nx = min(numel(xh),MAX_STATES);
            x_hat_k(1:nx) = xh(1:nx)';
        end

        % Almacenar
        t_k = S.framesTotal * Ts_i;
        S.t_vec(end+1,1)         = t_k;
        S.theta_vec(end+1,1)     = theta_rad;
        S.omega_vec(end+1,1)     = omega_rads;
        S.u_vec(end+1,1)         = double(u_pwm);
        S.u_unsat_vec(end+1,1)   = u_raw;
        S.theta_sim_vec(end+1,1) = theta_sim;
        S.omega_sim_vec(end+1,1) = omega_sim;
        S.omega_ref_vec(end+1,1) = S.omega_ref;
        S.x_hat_mat(end+1,:)     = x_hat_k;
        S.framesTotal = S.framesTotal + 1;

        if numel(S.t_vec) > S.maxPoints
            k0 = numel(S.t_vec) - S.maxPoints + 1;
            S.t_vec         = S.t_vec(k0:end);
            S.theta_vec     = S.theta_vec(k0:end);
            S.omega_vec     = S.omega_vec(k0:end);
            S.u_vec         = S.u_vec(k0:end);
            S.u_unsat_vec   = S.u_unsat_vec(k0:end);
            S.theta_sim_vec = S.theta_sim_vec(k0:end);
            S.omega_sim_vec = S.omega_sim_vec(k0:end);
            S.omega_ref_vec = S.omega_ref_vec(k0:end);
            S.x_hat_mat     = S.x_hat_mat(k0:end,:);
        end
    end

    function updatePlots()
        if isempty(S.t_vec), return; end
        n = S.t_vec;
        set(hTh_r,'XData',n,'YData',S.theta_vec);
        set(hTh_s,'XData',n,'YData',S.theta_sim_vec);
        set(hOm_r, 'XData',n,'YData',S.omega_vec);
        set(hOm_s, 'XData',n,'YData',S.omega_sim_vec);
        set(hOm_rf,'XData',n,'YData',S.omega_ref_vec);
        ud  = u2disp(S.u_vec);
        uud = u2disp(S.u_unsat_vec);
        set(hU_s, 'XData',n,'YData',ud);
        set(hU_us,'XData',n,'YData',uud);
        for ii = 1:MAX_STATES
            set(hSt(ii),'XData',n,'YData',S.x_hat_mat(:,ii));
        end
    end

    function ud = u2disp(v)
        if strcmp(S.u_unit,'Volts')
            ud = v * 12.0 / PWM_MAX;
        else
            ud = v;
        end
    end

%% ── Funciones STEP del controlador ──────────────────────────────────────────
    function initCtrlState()
        S.ctrl_state.inner = initLoopState(S.ctrl_inner);
        if ~S.innerOnly && ~isempty(S.ctrl_outer)
            S.ctrl_state.outer = initLoopState(S.ctrl_outer);
        end

        S.sim_state_i = [];
        if ~isempty(S.model_inner)
            S.sim_state_i = S.model_inner;
            S.sim_state_i.x = zeros(size(S.model_inner.A,1),1);
        end
        S.sim_state_o = [];
        if ~isempty(S.model_outer)
            S.sim_state_o = S.model_outer;
            S.sim_state_o.x = zeros(size(S.model_outer.A,1),1);
        end
    end

    function st = initLoopState(lp)
        st.type = lp.type;
        switch lp.type
            case 'PID'
                st.acc = 0; st.prev_e = 0;
            case 'TF'
                nb = numel(lp.b); na = numel(lp.a);
                st.w = zeros(max(nb,na)-1, 1);
            case 'SS'
                n = size(lp.A,1);
                st.v_int = 0;
                st.x_hat = zeros(n,1);
                if strcmpi(lp.mode,'Actual')
                    st.z = zeros(n,1);
                else
                    st.x = zeros(n,1);
                end
        end
    end

    function [u, cs] = runInnerStep(cs, ref, meas)
        lp = S.ctrl_inner;
        switch lp.type
            case 'PID', [u, cs.inner] = pidStep(lp, cs.inner, ref, meas);
            case 'TF',  [u, cs.inner] = tfStep(lp, cs.inner, ref-meas);
            case 'SS'
                if strcmpi(lp.mode,'Actual')
                    [u, cs.inner] = ssActual(lp, cs.inner, ref, meas);
                else
                    [u, cs.inner] = ssPredictor(lp, cs.inner, ref, meas);
                end
            otherwise, u = 0;
        end
    end

    function [omega_ref, cs] = runOuterStep(cs, ref, meas)
        lp = S.ctrl_outer;
        switch lp.type
            case 'PID', [omega_ref, cs.outer] = pidStep(lp, cs.outer, ref, meas);
            case 'TF',  [omega_ref, cs.outer] = tfStep(lp, cs.outer, ref-meas);
            case 'SS'
                if strcmpi(lp.mode,'Actual')
                    [omega_ref, cs.outer] = ssActual(lp, cs.outer, ref, meas);
                else
                    [omega_ref, cs.outer] = ssPredictor(lp, cs.outer, ref, meas);
                end
            otherwise, omega_ref = 0;
        end
    end

    % PID — acumulador (Ki ya convertido a escala por muestra)
    function [u, st] = pidStep(p, st, ref, meas)
        e = ref - meas;
        st.acc = st.acc + e;
        d = e - st.prev_e; st.prev_e = e;
        u = p.Kp*e + p.Ki*st.acc + p.Kd*d;
    end

    % TF — Direct Form II transpuesta
    function [y, st] = tfStep(p, st, x)
        b = p.b(:)'; a = p.a(:)';
        a0 = a(1); b = b/a0; a = a/a0;
        nb = numel(b); na = numel(a);
        nw = max(nb,na)-1;
        w = st.w;
        if numel(w) < nw, w(end+1:nw) = 0; end
        if nw == 0, y = b(1)*x; st.w = []; return; end
        y = b(1)*x + w(1);
        for k = 1:nw-1
            bk=0; if k+1<=nb, bk=b(k+1); end
            ak=0; if k+1<=na, ak=a(k+1); end
            w(k) = bk*x - ak*y + w(k+1);
        end
        k=nw; bk=0; if k+1<=nb, bk=b(k+1); end
        ak=0; if k+1<=na, ak=a(k+1); end
        w(k) = bk*x - ak*y;
        st.w = w;
    end

    % SS Estimador Actual: corrección ANTES del control
    function [u, st] = ssActual(p, st, ref, meas)
        inn   = meas - p.C*st.z;
        x_hat = st.z + p.L*inn;
        st.x_hat = x_hat;
        u = -(p.K*x_hat) + p.Ki*st.v_int;
        st.v_int = st.v_int + (ref - meas);
        st.z = p.A*x_hat + p.B*u;
        u = u(1);
    end

    % SS Estimador Predictor: control ANTES de la corrección
    function [u, st] = ssPredictor(p, st, ref, meas)
        st.x_hat = st.x;
        u = -(p.K*st.x) + p.Ki*st.v_int;
        st.v_int = st.v_int + (ref - meas);
        inn  = meas - p.C*st.x;
        st.x = p.A*st.x + p.B*u + p.L*inn;
        u = u(1);
    end

    % Simulación interna
    function [y, st] = simStep(st, u)
        y    = st.C*st.x;
        st.x = st.A*st.x + st.B*u;
    end

%% ── Sesiones ──────────────────────────────────────────────────────────────────
    function autoSave()
        try
            saveSession(S.lastSessionPath);
            lblAutoSave.Text = "Auto-save: " + string(datestr(now,'HH:MM:SS'));
        catch, end
    end

    function onSaveSess(~,~)
        [f,p] = uiputfile('*.mat','Guardar sesión',SESSIONS_DIR);
        if isequal(f,0), return; end
        try, saveSession(fullfile(p,f));
             logMsg("Sesión guardada: " + string(f));
        catch e, logMsg("Save FAIL: "+string(e.message)); end
    end

    function onLoadSess(~,~)
        [f,p] = uigetfile('*.mat','Cargar sesión',SESSIONS_DIR);
        if isequal(f,0), return; end
        try, loadSession(fullfile(p,f));
             logMsg("Sesión cargada: " + string(f));
        catch e, logMsg("Load FAIL: "+string(e.message)); end
    end

    function onLoadLast(~,~)
        if ~isfile(S.lastSessionPath)
            logMsg("No existe LAST_SESSION."); return;
        end
        try, loadSession(S.lastSessionPath);
             logMsg("Última sesión cargada.");
        catch e, logMsg("LoadLast FAIL: "+string(e.message)); end
    end

    function saveSession(fp)
        sess.Ts_inner   = S.Ts_inner;
        sess.Ts_outer   = S.Ts_outer;
        sess.N          = S.N;
        sess.sat_min    = S.sat_min;
        sess.sat_max    = S.sat_max;
        sess.PENDULO_CPR = S.PENDULO_CPR;
        sess.u_unit     = S.u_unit;
        sess.ctrl_inner  = S.ctrl_inner;
        sess.ctrl_outer  = S.ctrl_outer;
        sess.model_inner = S.model_inner;
        sess.model_outer = S.model_outer;
        sess.innerOnly   = S.innerOnly;
        sess.data.t          = S.t_vec;
        sess.data.theta_real = S.theta_vec;
        sess.data.omega_real = S.omega_vec;
        sess.data.u          = S.u_vec;
        sess.data.u_unsat    = S.u_unsat_vec;
        sess.model_scale_i = S.model_scale_i;
        sess.model_scale_o = S.model_scale_o;
        sess.data.theta_sim  = S.theta_sim_vec;
        sess.data.omega_sim  = S.omega_sim_vec;
        sess.data.omega_ref  = S.omega_ref_vec;
        sess.data.x_hat      = S.x_hat_mat;
        sess.timestamp = datestr(now,'yyyy-mm-dd HH:MM:SS');
        save(fp, '-struct','sess');
    end

    function loadSession(fp)
        d = load(fp);
        if isfield(d,'Ts_inner'),    S.Ts_inner    = d.Ts_inner; end
        if isfield(d,'Ts_outer'),    S.Ts_outer    = d.Ts_outer; end
        if isfield(d,'N'),           S.N           = d.N; end
        if isfield(d,'sat_min'),     S.sat_min     = d.sat_min;   edtSmin.Value = d.sat_min; end
        if isfield(d,'sat_max'),     S.sat_max     = d.sat_max;   edtSmax.Value = d.sat_max; end
        % PENDULO_CPR es fijo (10000) — no se restaura de sesión
        if isfield(d,'ctrl_inner') && ~isempty(d.ctrl_inner)
            S.ctrl_inner = d.ctrl_inner;
            lblInfI.Text = sprintf('controlador: %s    Ts: %.4g s', d.ctrl_inner.type, d.ctrl_inner.Ts);
        end
        if isfield(d,'ctrl_outer') && ~isempty(d.ctrl_outer)
            S.ctrl_outer = d.ctrl_outer;
            lblInfO.Text = sprintf('controlador: %s    Ts: %.4g s', d.ctrl_outer.type, d.ctrl_outer.Ts);
        end
        if isfield(d,'model_inner') && ~isempty(d.model_inner)
            S.model_inner = d.model_inner;
            lblModInfoI.Text = sprintf('modelo sim:   ss  %dx%d  Ts=%.3g s', ...
                size(d.model_inner.A,1), size(d.model_inner.B,2), d.model_inner.Ts);
        end
        if isfield(d,'model_outer') && ~isempty(d.model_outer)
            S.model_outer = d.model_outer;
            lblModInfoO.Text = sprintf('modelo sim:   ss  %dx%d  Ts=%.3g s', ...
                size(d.model_outer.A,1), size(d.model_outer.B,2), d.model_outer.Ts);
        end
        if isfield(d,'model_scale_i')
            S.model_scale_i = d.model_scale_i; edtMdlScaleI.Value = d.model_scale_i; end
        if isfield(d,'model_scale_o')
            S.model_scale_o = d.model_scale_o; edtMdlScaleO.Value = d.model_scale_o; end
        if isfield(d,'innerOnly')
            S.innerOnly = d.innerOnly;
            cbOuterEn.Value = ~d.innerOnly;
            onOuterEnChanged();
        end
        if isfield(d,'data')
            dd = d.data;
            S.t_vec         = getf(dd,'t');
            S.theta_vec     = getf(dd,'theta_real');
            S.omega_vec     = getf(dd,'omega_real');
            S.u_vec         = getf(dd,'u');
            S.u_unsat_vec   = getf(dd,'u_unsat');
            S.theta_sim_vec = getf(dd,'theta_sim');
            S.omega_sim_vec = getf(dd,'omega_sim');
            S.omega_ref_vec = getf(dd,'omega_ref');
            xh = getf(dd,'x_hat');
            [r,c] = size(xh);
            if c < MAX_STATES, xh = [xh, nan(r,MAX_STATES-c)]; end
            S.x_hat_mat = xh;
            S.framesTotal = numel(S.t_vec);
        end
        updateTsDisplay();
        updateDataInfo();
        updatePlots();
        drawnow;
    end

    function v = getf(s,f)
        if isfield(s,f), v = s.(f); else, v = zeros(0,1); end
    end

    function onExport(~,~)
        if isempty(S.t_vec), uialert(fig,'No hay datos.','Export'); return; end
        [f,p] = uiputfile('*.mat','Exportar datos',SESSIONS_DIR);
        if isequal(f,0), return; end
        try
            data.t           = S.t_vec;
            data.theta_real  = S.theta_vec;
            data.omega_real  = S.omega_vec;
            data.u           = S.u_vec;
            data.u_unsat     = S.u_unsat_vec;
            data.theta_sim   = S.theta_sim_vec;
            data.omega_sim   = S.omega_sim_vec;
            data.omega_ref   = S.omega_ref_vec;
            data.x_hat       = S.x_hat_mat;
            data.ctrl_inner  = S.ctrl_inner;
            data.ctrl_outer  = S.ctrl_outer;
            data.Ts_inner    = S.Ts_inner;
            data.Ts_outer    = S.Ts_outer;
            data.N           = S.N;
            data.sat_min     = S.sat_min;
            data.sat_max     = S.sat_max;
            data.PENDULO_CPR = S.PENDULO_CPR;
            data.timestamp   = datestr(now,'yyyy-mm-dd HH:MM:SS');
            save(fullfile(p,f),'-struct','data');
            logMsg("Export OK: " + string(f));
        catch e, logMsg("Export FAIL: "+string(e.message)); end
    end

    function onClearData(~,~)
        S.t_vec=[]; S.theta_vec=[]; S.omega_vec=[];
        S.u_vec=[]; S.u_unsat_vec=[]; S.theta_sim_vec=[]; S.omega_sim_vec=[];
        S.omega_ref_vec=[]; S.x_hat_mat=nan(0,MAX_STATES);
        S.rxBuf=uint8([]); S.framesTotal=0;
        updatePlots(); updateDataInfo();
        logMsg("Datos borrados.");
    end

    function onClose(~,~)
        try, stopStreaming(); catch, end
        try
            if S.isConnected && ~isempty(S.sp), flush(S.sp); delete(S.sp); end
        catch, end
        try, autoSave(); catch, end
        delete(fig);
    end

end  % psoc_pendulo_gui
