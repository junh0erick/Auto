function pendulo_gui2()
%PENDULO_GUI2  GUI Péndulo Invertido PSoC 5LP — v4
%  MATLAB calcula todo el control (double precision). PSoC = sensor/actuador.
%
%  Protocolo uartp_pend:
%    CMD: 'r'→K  'f'→R→[Fs f32 handshake]→K  'i'→K  's'→K
%    CTRL: PSoC→[theta int32 LE (4B)][delta_om int16 LE (2B)] 6B
%           MATLAB→[u_pwm int16 LE] 2B  |  [0,0,'s'] para stop
%
%  modos ctrl: 0=TF  1=SS-Pred-NoI  2=SS-Act-NoI  3=SS-Pred-I
%              4=SS-Act-I  5=Open-loop  6=Off

%% ═══ FÍSICA Y PROTOCOLO ══════════════════════════════════════════════════════
FRAME_SZ      = 6;        % bytes por trama PSoC→MATLAB
STREAM_PERIOD = 0.005;    % 5 ms — periodo del timer de lectura
MAX_PTS       = 500000;   % máx puntos en memoria
STEP_TMO      = 0.8;      % timeout por paso handshake [s]
MAX_RETRIES   = 50;       % reintentos handshake

MOTOR_CPR   = 1040.0;     % cuentas/rev encoder motor   (QuadDec x4)
PENDULO_CPR = 10000.0;    % cuentas/rev encoder péndulo (QuadDec x4)

MODE_NAMES = {'TF','SS','Open-loop','Off'};

%% ═══ GEOMETRÍA — EDITAR AQUÍ PARA AJUSTAR LA INTERFAZ ══════════════════════
%  Todos los números importantes están en esta sección.
%  Regla: posiciones y=... son siempre desde el borde INFERIOR del contenedor.

% ── Figura ────────────────────────────────────────────────────────────────────
FIG_W     = 1500;   % ancho figura  [px]
FIG_H     = 720;    % alto figura   [px]

% ── Ejes (columna izquierda) ──────────────────────────────────────────────────
AX_X      = 8;      % margen izquierdo
AX_W      = 892;    % ancho de cada eje
AX_BOT    = 10;     % y del eje más inferior
AX_H      = 170;    % alto de cada eje (los 4 son iguales)
AX_GAP    = 4;      % separación vertical entre ejes consecutivos

% ── Columna derecha: paneles ──────────────────────────────────────────────────
RX        = 908;    % borde izquierdo de la columna derecha
RW        = 585;    % ancho de todos los paneles derechos
RGAP      = 4;      % espacio entre paneles

% ── Alturas de los paneles derechos ──────────────────────────────────────────
H_LOG     = 86;     % textarea de log
H_VIZ     = 80;     % panel visualización (checkboxes señales)
H_DATA    = 79;     % panel datos (export, borrar, scalers)
H_CTRL    = 100;    % panel control (ref, start, stop, sat)
H_PLANT   = 145;    % panel planta (compacto — coefs van en popup)
H_CONN    = 35;     % panel conexión

% ── Posiciones Y de paneles (de abajo hacia arriba, automáticas) ──────────────
Y_LOG     = 5;
Y_VIZ     = Y_LOG   + H_LOG   + RGAP;
Y_DATA    = Y_VIZ   + H_VIZ   + RGAP;
Y_CTRL    = Y_DATA  + H_DATA  + RGAP;
Y_PLANT0  = Y_CTRL  + H_CTRL  + RGAP;
Y_PLANT1  = Y_PLANT0 + H_PLANT + RGAP;
Y_CONN    = Y_PLANT1 + H_PLANT + RGAP;

% ── Padding interno de paneles ────────────────────────────────────────────────
PP_PX     = 8;      % margen horizontal izquierdo dentro del panel
PP_RH     = 24;     % alto de fila estándar
PP_RG     = 4;      % gap entre filas

% Y de filas dentro de pPlant (desde el fondo del panel, 4 filas)
PP_Y1     = 6;                        % fila más baja: botones
PP_Y2     = PP_Y1 + PP_RH + PP_RG;   % N / Fs
PP_Y3     = PP_Y2 + PP_RH + PP_RG;   % Modo / Obs / Integrador
PP_Y4     = PP_Y3 + PP_RH + PP_RG;   % label status

% ── Helper para posición y de los ejes ───────────────────────────────────────
%  k=0 → eje inferior (y2),  k=3 → eje superior (u1)
ax_y = @(k) AX_BOT + k * (AX_H + AX_GAP);

%% ═══ ESTADO PRINCIPAL S ══════════════════════════════════════════════════════
S = struct();
S.sp = []; S.isConnected = false;
S.streamTimer = []; S.rxBuf = uint8([]);
S.streamOn = false; S.inTxn = false;

% ── Vectores de telemetría (todos crecen en sincronía, mismo índice n) ────────
S.nVec        = zeros(0,1);
S.u1Vec       = zeros(0,1);   % u1 saturado
S.u1unsat_Vec = zeros(0,1);   % u1 sin saturar
S.y1Vec       = zeros(0,1);   % ω motor [rad/s]
S.u2Vec       = zeros(0,1);   % R inner = salida outer = ref del lazo inner
S.y2Vec       = zeros(0,1);   % θ péndulo [rad]
S.x1iVec      = zeros(0,1);   % x̂₁ inner
S.x2iVec      = zeros(0,1);   % x̂₂ inner
S.x1oVec      = zeros(0,1);   % x̂₁ outer
S.x2oVec      = zeros(0,1);   % x̂₂ outer
S.ySimVec1    = zeros(0,1);   % ŷ simulado planta inner  (NaN si sim OFF)
S.ySimVec2    = zeros(0,1);   % ŷ simulado planta outer  (NaN si sim OFF)
S.framesTotal = 0;

S.autoStopEn = false; S.autoStopN = 0; S.autoStopArmed = false;
S.ctrl = ctrl_state_default();

% ── Configuraciones de planta (fuente de verdad — save/load usa esto) ─────────
S.cfg(1) = cfg_default();   % inner (Planta 0, motor/ω)
S.cfg(2) = cfg_default();   % outer (Planta 1, péndulo/θ)
S.cfg(1).Fs = 200;          % Fs inner default [Hz]

% ── Handles de ventanas emergentes ────────────────────────────────────────────
S.ctrl_popup_fig = {[], []};
S.sim_popup_fig  = {[], []};

%% ═══ FIGURA ══════════════════════════════════════════════════════════════════
fig = uifigure('Name','Péndulo Invertido — Control v4',...
               'Position',[20 40 FIG_W FIG_H]);
fig.CloseRequestFcn = @onClose;

%% ═══ EJES ════════════════════════════════════════════════════════════════════
ax_y2 = uiaxes(fig,'Position',[AX_X ax_y(0) AX_W AX_H]);
ax_u2 = uiaxes(fig,'Position',[AX_X ax_y(1) AX_W AX_H]);
ax_y1 = uiaxes(fig,'Position',[AX_X ax_y(2) AX_W AX_H]);
ax_u1 = uiaxes(fig,'Position',[AX_X ax_y(3) AX_W AX_H]);

setupAx(ax_u1,'u₁ [PWM]  esfuerzo motor + R referencia','u₁');
setupAx(ax_y1,'y₁ = ω motor [rad/s]  /  x̂ inner  /  ŷ sim','ω');
setupAx(ax_u2,'u₂ = R inner  (salida outer = referencia lazo inner)','u₂');
setupAx(ax_y2,'y₂ = θ péndulo [rad]  /  x̂ outer  /  ŷ sim','θ');

hold(ax_u1,'on');
hU1sat   = stairs(ax_u1,nan,nan,'b-', 'LineWidth',1.5,'DisplayName','u₁ sat');
hU1unsat = stairs(ax_u1,nan,nan,'b--','LineWidth',0.8,'DisplayName','u₁ unsat');
hR1      = stairs(ax_u1,nan,nan,'r-', 'LineWidth',1.2,'DisplayName','R (ref inner)');
hold(ax_u1,'off'); legend(ax_u1,'show');

hold(ax_y1,'on');
hY1    = plot(ax_y1,nan,nan,'b-', 'LineWidth',1.5,'DisplayName','ω meas');
hY1sim = plot(ax_y1,nan,nan,'c--','LineWidth',1.2,'DisplayName','ω sim');
hX1i   = plot(ax_y1,nan,nan,'g-', 'LineWidth',1.0,'DisplayName','x̂₁ i');
hX2i   = plot(ax_y1,nan,nan,'m-', 'LineWidth',1.0,'DisplayName','x̂₂ i');
hold(ax_y1,'off'); legend(ax_y1,'show');

hold(ax_u2,'on');
hU2 = stairs(ax_u2,nan,nan,'r-','LineWidth',1.5,'DisplayName','R inner');
hold(ax_u2,'off'); legend(ax_u2,'show');

hold(ax_y2,'on');
hY2    = plot(ax_y2,nan,nan,'r-', 'LineWidth',1.5,'DisplayName','θ meas');
hY2sim = plot(ax_y2,nan,nan,'c--','LineWidth',1.2,'DisplayName','θ sim');
hX1o   = plot(ax_y2,nan,nan,'g-', 'LineWidth',1.0,'DisplayName','x̂₁ o');
hX2o   = plot(ax_y2,nan,nan,'m-', 'LineWidth',1.0,'DisplayName','x̂₂ o');
hold(ax_y2,'off'); legend(ax_y2,'show');

%% ═══ PANEL DERECHO ═══════════════════════════════════════════════════════════

% ── 1. Conexión ───────────────────────────────────────────────────────────────
pConn = uipanel(fig,'Title','Conexión','Position',[RX Y_CONN RW H_CONN]);
uilabel(pConn,'Text','Puerto:','Position',[ PP_PX    5 45 22]);
edtCom  = uieditfield(pConn,'text','Value','COM4', 'Position',[PP_PX+47  5 64 22]);
uilabel(pConn,'Text','Baud:',  'Position',[PP_PX+116 5 36 22]);
edtBaud = uieditfield(pConn,'numeric','Value',115200,'Limits',[1200 4e6],...
          'Position',[PP_PX+154 5 84 22]);
btnConn = uibutton(pConn,'Text','Conectar','Position',[PP_PX+244 4 98 24],...
          'BackgroundColor',[0.2 0.62 0.2],'FontColor','w',...
          'ButtonPushedFcn',@onConnToggle);
lblStat = uilabel(pConn,'Text','Desconectado','Position',[PP_PX+348 5 108 22]);
btnSave = uibutton(pConn,'Text','Guardar','Position',[PP_PX+460 4 56 24],...
          'ButtonPushedFcn',@onSaveSession);
btnLoad = uibutton(pConn,'Text','Cargar', 'Position',[PP_PX+520 4 52 24],...
          'ButtonPushedFcn',@onLoadSession);

% ── 2 & 3. Paneles de planta (compactos) ──────────────────────────────────────
PLANT_Y   = [Y_PLANT0, Y_PLANT1];
PLANT_LBL = {'Planta 0 — Motor / ω  (inner)','Planta 1 — Péndulo / θ  (outer)'};

ddMode     = gobjects(2,1);
ddObs      = gobjects(2,1);
cbInt      = gobjects(2,1);
edtN       = gobjects(2,1);
edtFs      = gobjects(2,1);
lblCfgStat = gobjects(2,1);
cbSimEn    = gobjects(2,1);

for pp = 1:2
    pP = uipanel(fig,'Title',PLANT_LBL{pp},...
         'Position',[RX PLANT_Y(pp) RW H_PLANT]);

    % Fila 4 (más alta): label estado configuración actual
    lblCfgStat(pp) = uilabel(pP,'Text','— sin configurar —',...
        'Position',[PP_PX PP_Y4 RW-PP_PX*2 PP_RH],...
        'FontColor',[0.35 0.35 0.35],'FontSize',9);

    % Fila 3: Modo / Obs / Integrador
    uilabel(pP,'Text','Modo:','Position',[PP_PX PP_Y3 42 22]);
    ddMode(pp) = uidropdown(pP,'Items',MODE_NAMES,'Value','Off',...
        'Position',[PP_PX+44 PP_Y3 108 22],...
        'ValueChangedFcn',@(~,~) onModeChange(pp));
    uilabel(pP,'Text','Obs:','Position',[PP_PX+158 PP_Y3 30 22]);
    ddObs(pp) = uidropdown(pP,'Items',{'Predictor','Actual'},...
        'Position',[PP_PX+190 PP_Y3 88 22],...
        'ValueChangedFcn',@(~,~) onObsChange(pp));
    cbInt(pp) = uicheckbox(pP,'Text','Integrador',...
        'Position',[PP_PX+286 PP_Y3 90 22],...
        'ValueChangedFcn',@(~,~) onIntChange(pp));

    % Fila 2: N o M, Fs (solo planta 0)
    if pp == 1
        uilabel(pP,'Text','M:','Position',[PP_PX PP_Y2 22 22]);
    else
        uilabel(pP,'Text','N:','Position',[PP_PX PP_Y2 22 22]);
    end
    edtN(pp) = uieditfield(pP,'numeric','Value',1,'Limits',[1 1e6],...
        'RoundFractionalValues','on','Position',[PP_PX+24 PP_Y2 58 22],...
        'ValueChangedFcn',@(~,~) onNChange(pp));
    if pp == 1
        uilabel(pP,'Text','Fs (Hz):','Position',[PP_PX+88 PP_Y2 54 22]);
        edtFs(pp) = uieditfield(pP,'numeric','Value',200,'Limits',[0.1 10000],...
            'Position',[PP_PX+144 PP_Y2 78 22],...
            'ValueChangedFcn',@(~,~) onFsChange());
    else
        uilabel(pP,'Text','(N = decimación entera de Fs_inner)',...
            'Position',[PP_PX+88 PP_Y2 270 22],...
            'FontColor',[0.5 0.5 0.5],'FontSize',9);
        edtFs(pp) = uieditfield(pP,'numeric','Value',0,'Limits',[0 10000],...
            'Visible','off','Position',[PP_PX+144 PP_Y2 78 22]);
    end

    % Fila 1 (inferior): botones popup
    uibutton(pP,'Text','⚙ Cargar controlador',...
        'Position',[PP_PX PP_Y1 158 PP_RH],...
        'ButtonPushedFcn',@(~,~) openCtrlPopup(pp));
    cbSimEn(pp) = uicheckbox(pP,'Text','Simular',...
        'Position',[PP_PX+166 PP_Y1 70 PP_RH],...
        'ValueChangedFcn',@(~,~) onSimToggle(pp));
    uibutton(pP,'Text','📈 Cargar modelo',...
        'Position',[PP_PX+242 PP_Y1 148 PP_RH],...
        'ButtonPushedFcn',@(~,~) openSimPopup(pp));
end
onModeChange(1); onModeChange(2);

% ── 4. Control ────────────────────────────────────────────────────────────────
pCtrl = uipanel(fig,'Title','Control','Position',[RX Y_CTRL RW H_CTRL]);
yC1 = H_CTRL - 20 - PP_RH;      % primera fila desde arriba
yC2 = yC1 - PP_RH - PP_RG;
yC3 = yC2 - PP_RH - PP_RG;
uilabel(pCtrl,'Text','Ref:','Position',[PP_PX yC1 30 22]);
edtRef = uieditfield(pCtrl,'numeric','Value',0,'Limits',[-1e6 1e6],...
         'Position',[PP_PX+32 yC1 66 22]);
uibutton(pCtrl,'Text','Aplicar ref','Position',[PP_PX+104 yC1-1 86 PP_RH],...
         'ButtonPushedFcn',@onApplyRef);
btnStart = uibutton(pCtrl,'Text','▶  Start','Position',[PP_PX+196 yC1-1 98 PP_RH],...
           'BackgroundColor',[0.12 0.62 0.12],'FontColor','w','FontWeight','bold',...
           'ButtonPushedFcn',@onStart);
btnStop  = uibutton(pCtrl,'Text','■  Stop', 'Position',[PP_PX+300 yC1-1 90 PP_RH],...
           'BackgroundColor',[0.78 0.1 0.1],'FontColor','w','FontWeight','bold',...
           'ButtonPushedFcn',@onStop);
lblRunSt = uilabel(pCtrl,'Text','Detenido','Position',[PP_PX+396 yC1 160 26],...
           'FontWeight','bold');
uilabel(pCtrl,'Text','u_min:','Position',[PP_PX yC2 42 22]);
edtSatMin = uieditfield(pCtrl,'numeric','Value',-1264,'Limits',[-1e6 0],...
            'Position',[PP_PX+44 yC2 66 22]);
uilabel(pCtrl,'Text','u_max:','Position',[PP_PX+116 yC2 42 22]);
edtSatMax = uieditfield(pCtrl,'numeric','Value', 1264,'Limits',[0 1e6],...
            'Position',[PP_PX+160 yC2 66 22]);
cbAutoStop = uicheckbox(pCtrl,'Text','Auto-stop en frames:','Value',false,...
             'Position',[PP_PX yC3 148 22],'ValueChangedFcn',@onAutoStopToggle);
edtAutoN   = uieditfield(pCtrl,'numeric','Value',2000,'Limits',[1 1e9],...
             'RoundFractionalValues','on','Position',[PP_PX+152 yC3 78 22]);
lblDataInf = uilabel(pCtrl,'Text','n=0','Position',[PP_PX+236 yC3 130 22]);

% ── 5. Datos ──────────────────────────────────────────────────────────────────
pData = uipanel(fig,'Title','Datos','Position',[RX Y_DATA RW H_DATA]);
yD1 = H_DATA - 20 - PP_RH;
yD2 = yD1 - PP_RH - PP_RG;
uibutton(pData,'Text','Exportar .mat','Position',[PP_PX yD2 128 PP_RH],...
         'ButtonPushedFcn',@onExport);
uibutton(pData,'Text','Borrar datos', 'Position',[PP_PX+134 yD2 118 PP_RH],...
         'ButtonPushedFcn',@onClear);
uilabel(pData,'Text','u1×:','Position',[PP_PX+266 yD1+2 28 18]);
edtSU1 = uieditfield(pData,'numeric','Value',1,'Limits',[-1e9 1e9],...
         'Position',[PP_PX+296 yD1 52 22],'ValueChangedFcn',@updatePlots);
uilabel(pData,'Text','u2×:','Position',[PP_PX+354 yD1+2 28 18]);
edtSU2 = uieditfield(pData,'numeric','Value',1,'Limits',[-1e9 1e9],...
         'Position',[PP_PX+384 yD1 52 22],'ValueChangedFcn',@updatePlots);
uilabel(pData,'Text','y1×:','Position',[PP_PX+266 yD2+2 28 18]);
edtSY1 = uieditfield(pData,'numeric','Value',1,'Limits',[-1e9 1e9],...
         'Position',[PP_PX+296 yD2 52 22],'ValueChangedFcn',@updatePlots);
uilabel(pData,'Text','y2×:','Position',[PP_PX+354 yD2+2 28 18]);
edtSY2 = uieditfield(pData,'numeric','Value',1,'Limits',[-1e9 1e9],...
         'Position',[PP_PX+384 yD2 52 22],'ValueChangedFcn',@updatePlots);

% ── 6. Visualización ──────────────────────────────────────────────────────────
pViz = uipanel(fig,'Title','Visualización','Position',[RX Y_VIZ RW H_VIZ]);
yV1 = H_VIZ - 20 - PP_RH;
yV2 = yV1 - PP_RH - PP_RG;
yV3 = yV2 - PP_RH - PP_RG;
% Señales principales
uilabel(pViz,'Text','Señales:','Position',[PP_PX yV1+2 54 18]);
cbSigU      = uicheckbox(pViz,'Text','u sat',  'Value',true, 'Position',[PP_PX+ 58 yV1 58 22],'ValueChangedFcn',@updatePlots);
cbSigUunsat = uicheckbox(pViz,'Text','u unsat','Value',true, 'Position',[PP_PX+118 yV1 68 22],'ValueChangedFcn',@updatePlots);
cbSigR      = uicheckbox(pViz,'Text','R',      'Value',true, 'Position',[PP_PX+188 yV1 34 22],'ValueChangedFcn',@updatePlots);
cbSigY      = uicheckbox(pViz,'Text','y',      'Value',true, 'Position',[PP_PX+224 yV1 34 22],'ValueChangedFcn',@updatePlots);
cbSigSim    = uicheckbox(pViz,'Text','ŷ sim',  'Value',true, 'Position',[PP_PX+260 yV1 60 22],'ValueChangedFcn',@updatePlots);
% Estados SS
uilabel(pViz,'Text','SS x̂:','Position',[PP_PX yV2+2 44 18]);
cbX1i = uicheckbox(pViz,'Text','x̂₁ᵢ','Value',false,'Position',[PP_PX+ 50 yV2 48 22],'ValueChangedFcn',@updatePlots);
cbX2i = uicheckbox(pViz,'Text','x̂₂ᵢ','Value',false,'Position',[PP_PX+100 yV2 48 22],'ValueChangedFcn',@updatePlots);
cbX1o = uicheckbox(pViz,'Text','x̂₁ₒ','Value',false,'Position',[PP_PX+150 yV2 48 22],'ValueChangedFcn',@updatePlots);
cbX2o = uicheckbox(pViz,'Text','x̂₂ₒ','Value',false,'Position',[PP_PX+200 yV2 48 22],'ValueChangedFcn',@updatePlots);
% Escaladores de estados
uilabel(pViz,'Text','Esc×:','Position',[PP_PX yV3+2 42 18]);
uilabel(pViz,'Text','x1ᵢ','Position',[PP_PX+ 48 yV3+2 24 18]);
edtSX1i = uieditfield(pViz,'numeric','Value',1,'Limits',[-1e9 1e9],'Position',[PP_PX+ 74 yV3 44 22],'ValueChangedFcn',@updatePlots);
uilabel(pViz,'Text','x2ᵢ','Position',[PP_PX+124 yV3+2 24 18]);
edtSX2i = uieditfield(pViz,'numeric','Value',1,'Limits',[-1e9 1e9],'Position',[PP_PX+150 yV3 44 22],'ValueChangedFcn',@updatePlots);
uilabel(pViz,'Text','x1ₒ','Position',[PP_PX+200 yV3+2 24 18]);
edtSX1o = uieditfield(pViz,'numeric','Value',1,'Limits',[-1e9 1e9],'Position',[PP_PX+226 yV3 44 22],'ValueChangedFcn',@updatePlots);
uilabel(pViz,'Text','x2ₒ','Position',[PP_PX+276 yV3+2 24 18]);
edtSX2o = uieditfield(pViz,'numeric','Value',1,'Limits',[-1e9 1e9],'Position',[PP_PX+302 yV3 44 22],'ValueChangedFcn',@updatePlots);

% ── 7. Log ────────────────────────────────────────────────────────────────────
txtLog = uitextarea(fig,'Editable','off','Position',[RX Y_LOG RW H_LOG],...
         'FontSize',9);
txtLog.Value = strings(0,1);

%% ═══ HELPERS UI ══════════════════════════════════════════════════════════════

    function logMsg(msg)
        ts = string(datestr(now,'HH:MM:SS.FFF'));
        v  = txtLog.Value;
        if ~isstring(v), v = string(v); end
        v(end+1,1) = "[" + ts + "] " + string(msg);
        if numel(v) > 400, v = v(end-399:end); end
        txtLog.Value = v;
        drawnow limitrate;
    end

    function ok = reqConn()
        ok = S.isConnected && ~isempty(S.sp);
        if ~ok, logMsg("⚠ No conectado."); end
    end

    function setupAx(ax, ttl, yl)
        ax.XGrid = 'on'; ax.YGrid = 'on';
        ax.Title.String = ttl;
        ax.XLabel.String = 'frame';
        ax.YLabel.String = yl;
        ax.YLimMode = 'auto';
        ax.XLimMode = 'auto';
    end

    % campo de texto para entrar un escalar SS en un popup
    function ef = ssEdt(parent, x, y, val)
        ef = uieditfield(parent,'text','Value',num2str(val),...
             'Position',[x y 62 22]);
    end

    function v = ssV(ef)
        v = str2double(strtrim(string(ef.Value)));
        if ~isfinite(v), v = 0; end
    end

    function s = safeS(v)
        if isfinite(v) && v ~= 0, s = v; else, s = 1; end
    end

%% ═══ CALLBACKS DE PANELES DE PLANTA (actualizan S.cfg) ══════════════════════

    function onModeChange(pp)
        mn = ddMode(pp).Value;
        S.cfg(pp).mode = mn;
        isSS = strcmp(mn,'SS');
        ddObs(pp).Enable = isSS;
        cbInt(pp).Enable = isSS;
        if ~isSS, cbInt(pp).Value = false; S.cfg(pp).has_int = false; end
        updateAxesLayout();
        updateCfgStatus(pp);
    end

    function onObsChange(pp)
        S.cfg(pp).obs = ddObs(pp).Value;
    end

    function onIntChange(pp)
        S.cfg(pp).has_int = cbInt(pp).Value;
    end

    function onNChange(pp)
        S.cfg(pp).N = max(1, round(edtN(pp).Value));
        updateCfgStatus(pp);
    end

    function onFsChange()
        S.cfg(1).Fs = edtFs(1).Value;
    end

    function onSimToggle(pp)
        S.cfg(pp).sim_enabled = cbSimEn(pp).Value;
        if ~cbSimEn(pp).Value
            % apagar sim → reemplazar datos con NaN para que no se grafiquen
            if pp == 1
                S.ySimVec1 = nan(size(S.ySimVec1));
            else
                S.ySimVec2 = nan(size(S.ySimVec2));
            end
            updatePlots();
        end
    end

    function updateCfgStatus(pp)
        c = S.cfg(pp);
        switch c.mode
            case 'SS'
                obs_str = c.obs(1:4);
                int_str = ''; if c.has_int, int_str = '+∫'; end
                lblCfgStat(pp).Text = sprintf('SS [%s%s]  N=%d', obs_str, int_str, c.N);
            case 'TF'
                lblCfgStat(pp).Text = sprintf('TF  ord=%d  N=%d', c.tf_ord, c.N);
            otherwise
                lblCfgStat(pp).Text = sprintf('%s  N=%d', c.mode, c.N);
        end
    end

    function updateAxesLayout()
        inn = ~strcmp(ddMode(1).Value,'Off');
        out = ~strcmp(ddMode(2).Value,'Off');
        AH2 = 2*AX_H + AX_GAP;  % altura doble cuando solo hay una planta
        if inn && out
            ax_u1.Visible='on'; ax_y1.Visible='on';
            ax_u2.Visible='on'; ax_y2.Visible='on';
            ax_u1.Position = [AX_X ax_y(3) AX_W AX_H];
            ax_y1.Position = [AX_X ax_y(2) AX_W AX_H];
            ax_u2.Position = [AX_X ax_y(1) AX_W AX_H];
            ax_y2.Position = [AX_X ax_y(0) AX_W AX_H];
        elseif inn
            ax_u1.Visible='on';  ax_y1.Visible='on';
            ax_u2.Visible='off'; ax_y2.Visible='off';
            ax_u1.Position = [AX_X AX_BOT+AH2+AX_GAP AX_W AH2];
            ax_y1.Position = [AX_X AX_BOT             AX_W AH2];
        elseif out
            ax_u1.Visible='off'; ax_y1.Visible='off';
            ax_u2.Visible='on';  ax_y2.Visible='on';
            ax_u2.Position = [AX_X AX_BOT+AH2+AX_GAP AX_W AH2];
            ax_y2.Position = [AX_X AX_BOT             AX_W AH2];
        else
            ax_u1.Visible='off'; ax_y1.Visible='off';
            ax_u2.Visible='off'; ax_y2.Visible='off';
        end
    end

    function updatePlots(~,~)
        % Limpiar todos los handles
        hAll = [hU1sat, hU1unsat, hR1, hY1, hY1sim, hX1i, hX2i,...
                hU2, hY2, hY2sim, hX1o, hX2o];
        for h = hAll
            set(h,'XData',nan,'YData',nan);
        end
        if isempty(S.nVec), return; end

        n   = S.nVec;
        su1 = safeS(edtSU1.Value); su2 = safeS(edtSU2.Value);
        sy1 = safeS(edtSY1.Value); sy2 = safeS(edtSY2.Value);
        sx1i= safeS(edtSX1i.Value); sx2i= safeS(edtSX2i.Value);
        sx1o= safeS(edtSX1o.Value); sx2o= safeS(edtSX2o.Value);

        inn = ~strcmp(ddMode(1).Value,'Off');
        out = ~strcmp(ddMode(2).Value,'Off');

        if inn
            if cbSigU.Value,      set(hU1sat,  'XData',n,'YData',S.u1Vec*su1);        end
            if cbSigUunsat.Value, set(hU1unsat,'XData',n,'YData',S.u1unsat_Vec*su1);  end
            if cbSigR.Value,      set(hR1,     'XData',n,'YData',S.u2Vec*su2);        end
            if cbSigY.Value,      set(hY1,     'XData',n,'YData',S.y1Vec*sy1);        end
            if cbSigSim.Value && numel(S.ySimVec1)==numel(n)
                set(hY1sim,'XData',n,'YData',S.ySimVec1*sy1);
            end
            if cbX1i.Value && numel(S.x1iVec)==numel(n)
                set(hX1i,'XData',n,'YData',S.x1iVec*sx1i);
            end
            if cbX2i.Value && numel(S.x2iVec)==numel(n)
                set(hX2i,'XData',n,'YData',S.x2iVec*sx2i);
            end
        end
        if out
            if cbSigU.Value,  set(hU2, 'XData',n,'YData',S.u2Vec*su2);  end
            if cbSigY.Value,  set(hY2, 'XData',n,'YData',S.y2Vec*sy2);  end
            if cbSigSim.Value && numel(S.ySimVec2)==numel(n)
                set(hY2sim,'XData',n,'YData',S.ySimVec2*sy2);
            end
            if cbX1o.Value && numel(S.x1oVec)==numel(n)
                set(hX1o,'XData',n,'YData',S.x1oVec*sx1o);
            end
            if cbX2o.Value && numel(S.x2oVec)==numel(n)
                set(hX2o,'XData',n,'YData',S.x2oVec*sx2o);
            end
        end

        % Forzar auto-escala (arregla el bug de ejes fijos tras cambios de modo)
        for axh = [ax_u1, ax_y1, ax_u2, ax_y2]
            if strcmp(axh.Visible,'on')
                axis(axh,'auto');
            end
        end
        drawnow;
    end

    function updateDataInfo()
        try, lblDataInf.Text = sprintf('n=%d', numel(S.nVec)); catch, end
    end

%% ═══ DEFAULTS ════════════════════════════════════════════════════════════════

    function c = cfg_default()
        c = struct(...
            'mode','Off',  'obs','Predictor',  'has_int',false,...
            'N',1,  'Fs',200,...
            'tf_b',zeros(6,1),  'tf_a',[1;zeros(5,1)],  'tf_ord',1,...
            'ss_A',eye(2),  'ss_B',zeros(2,1),  'ss_C',zeros(1,2),...
            'ss_D',0,  'ss_L',zeros(2,1),  'ss_K',zeros(1,2),...
            'ss_Ki',0,  'ss_Nbar',1,...
            'sim_enabled',false,...
            'sim_Ad',[],'sim_Bd',[],'sim_Cd',[],'sim_Dd',0,'sim_x',[]);
    end

    function lp = ctrl_loop_default()
        lp = struct('mode',6,'tf_b',zeros(6,1),'tf_a',[1;zeros(5,1)],...
            'tf_w',zeros(5,1),'A',eye(2),'B',zeros(2,1),...
            'C',zeros(1,2),'D',0,'K',zeros(1,2),'L',zeros(2,1),...
            'Ki',0,'Nbar',1,'xhat',zeros(2,1),'zhat',zeros(2,1),...
            'vint',0,'u_prev',0,'ref',0,'N',1,'cnt',0);
    end

    function cs = ctrl_state_default()
        cs = struct('inner',ctrl_loop_default(),'outer',ctrl_loop_default(),...
            'Ts',0.005,'sat_min',-1264,'sat_max',1264);
    end

    function lp = ctrl_reset_states(lp)
        lp.tf_w = zeros(5,1); lp.xhat = zeros(2,1); lp.zhat = zeros(2,1);
        lp.vint = 0; lp.u_prev = 0; lp.cnt = 0;
    end

%% ═══ ALGORITMOS DE CONTROL (double precision) ════════════════════════════════

    function [y, lp] = tf_step(lp, x)
        b=lp.tf_b; a=lp.tf_a; w=lp.tf_w;
        y    =  b(1)*x + w(1);
        w(1) =  w(2) + b(2)*x - a(2)*y;
        w(2) =  w(3) + b(3)*x - a(3)*y;
        w(3) =  w(4) + b(4)*x - a(4)*y;
        w(4) =  w(5) + b(5)*x - a(5)*y;
        w(5) =         b(6)*x - a(6)*y;
        lp.tf_w = w;
    end

    function [u, lp] = ss_pred_step(lp, y, Ts)
        has_i = (lp.mode==3)||(lp.mode==4);
        up = lp.u_prev;
        if has_i, lp.vint = lp.vint + (lp.ref - y)*Ts; end
        if has_i, extra = lp.Ki*lp.vint; else, extra = lp.Nbar*lp.ref; end
        u = extra - lp.K*lp.xhat;
        innov = y - (lp.C*lp.xhat + lp.D*up);
        lp.xhat = lp.A*lp.xhat + lp.B*u + lp.L*innov;
        lp.u_prev = u;
    end

    function [u, lp] = ss_act_step(lp, y, Ts)
        has_i = (lp.mode==3)||(lp.mode==4);
        up = lp.u_prev;
        innov = y - (lp.C*lp.zhat + lp.D*up);
        lp.xhat = lp.zhat + lp.L*innov;
        if has_i, lp.vint = lp.vint + (lp.ref - y)*Ts; end
        if has_i, extra = lp.Ki*lp.vint; else, extra = lp.Nbar*lp.ref; end
        u = extra - lp.K*lp.xhat;
        lp.zhat = lp.A*lp.xhat + lp.B*u;
        lp.u_prev = u;
    end

    function [u, lp] = loop_step(lp, y, Ts)
        switch lp.mode
            case 0,      [u,lp] = tf_step(lp, lp.ref - y);
            case {1,3},  [u,lp] = ss_pred_step(lp, y, Ts);
            case {2,4},  [u,lp] = ss_act_step(lp, y, Ts);
            case 5,      u = lp.ref;
            otherwise,   u = 0;
        end
        lp.u_prev = u;
    end

    function [u_sat,u_unsat,x1i,x2i,x1o,x2o,ctrl] = run_control(ctrl, y1, y2)
        inner = ctrl.inner; outer = ctrl.outer; Ts = ctrl.Ts;
        if outer.mode ~= 6
            outer.cnt = outer.cnt + 1;
            if outer.cnt >= outer.N
                outer.cnt = 0;
                [u2, outer] = loop_step(outer, y2, Ts);
                if inner.mode ~= 6, inner.ref = u2; end
            end
        end
        u_unsat = 0;
        if inner.mode ~= 6
            [u_unsat, inner] = loop_step(inner, y1, Ts);
        elseif outer.mode ~= 6
            u_unsat = outer.u_prev;
        end
        u_sat = max(ctrl.sat_min, min(ctrl.sat_max, u_unsat));
        x1i = inner.xhat(1); x2i = inner.xhat(2);
        x1o = outer.xhat(1); x2o = outer.xhat(2);
        ctrl.inner = inner; ctrl.outer = outer;
    end

%% ═══ GUI → STRUCT CONTROLADOR ════════════════════════════════════════════════

    function mv = cfg_to_mode_val(c)
        switch c.mode
            case 'TF',       mv = 0;
            case 'SS'
                if     ~strcmp(c.obs,'Actual') && ~c.has_int, mv=1;
                elseif  strcmp(c.obs,'Actual') && ~c.has_int, mv=2;
                elseif ~strcmp(c.obs,'Actual') &&  c.has_int, mv=3;
                else,                                          mv=4; end
            case 'Open-loop', mv = 5;
            otherwise,        mv = 6;
        end
    end

    function lp = cfg_to_ctrl_loop(c)
        lp = ctrl_loop_default();
        lp.mode = cfg_to_mode_val(c);
        lp.N    = max(1, round(c.N));
        lp.tf_b = c.tf_b(:); lp.tf_a = c.tf_a(:);
        lp.A = c.ss_A; lp.B = c.ss_B;
        lp.C = c.ss_C; lp.D = c.ss_D;
        lp.L = c.ss_L; lp.K = c.ss_K;
        lp.Ki = c.ss_Ki; lp.Nbar = c.ss_Nbar;
    end

    function onApplyCtrl(pp)
        c  = S.cfg(pp);
        lp = cfg_to_ctrl_loop(c);
        if pp == 1
            if S.streamOn
                % Preservar estados dinámicos para evitar discontinuidades
                lp.tf_w   = S.ctrl.inner.tf_w;
                lp.xhat   = S.ctrl.inner.xhat;
                lp.zhat   = S.ctrl.inner.zhat;
                lp.vint   = S.ctrl.inner.vint;
                lp.u_prev = S.ctrl.inner.u_prev;
                lp.cnt    = S.ctrl.inner.cnt;
                lp.ref    = S.ctrl.inner.ref;
            else
                lp.ref = edtRef.Value;
            end
            S.ctrl.inner = lp;
        else
            if S.streamOn
                lp.tf_w   = S.ctrl.outer.tf_w;
                lp.xhat   = S.ctrl.outer.xhat;
                lp.zhat   = S.ctrl.outer.zhat;
                lp.vint   = S.ctrl.outer.vint;
                lp.u_prev = S.ctrl.outer.u_prev;
                lp.cnt    = S.ctrl.outer.cnt;
                lp.ref    = S.ctrl.outer.ref;
            else
                lp.ref = 0;
            end
            S.ctrl.outer = lp;
        end
        updateCfgStatus(pp);
        logMsg(sprintf('P%d: ctrl aplicado. modo=%d  N=%d', pp-1, lp.mode, lp.N));
    end

%% ═══ POPUP — CONTROLADOR ════════════════════════════════════════════════════

    function openCtrlPopup(pp)
        if ~isempty(S.ctrl_popup_fig{pp}) && isvalid(S.ctrl_popup_fig{pp})
            figure(S.ctrl_popup_fig{pp}); return;
        end
        PW = 670; PH = 520;
        pf = uifigure('Name',sprintf('Controlador — Planta %d',pp-1),...
             'Position',[200+pp*30 140 PW PH],'Resize','off');
        S.ctrl_popup_fig{pp} = pf;
        pf.DeleteFcn = @(~,~) popCtrlClosed(pp);

        c = S.cfg(pp);   % config actual para pre-poblar

        % ── Fila superior ─────────────────────────────────────────────────────
        TOP  = PH - 48;
        uilabel(pf,'Text','Modo:', 'Position',[8  TOP 42 22]);
        pd_m = uidropdown(pf,'Items',MODE_NAMES,'Value',c.mode,...
               'Position',[52 TOP 110 22],'ValueChangedFcn',@pRefresh);
        uilabel(pf,'Text','Obs:','Position',[170 TOP 30 22]);
        pd_o = uidropdown(pf,'Items',{'Predictor','Actual'},'Value',c.obs,...
               'Position',[202 TOP 100 22]);
        pc_i = uicheckbox(pf,'Text','Integrador','Value',c.has_int,...
               'Position',[312 TOP 90 22],...
               'ValueChangedFcn',@(~,~) pSyncKiNbar());
        if pp == 1
            uilabel(pf,'Text','M:','Position',[412 TOP 20 22]);
        else
            uilabel(pf,'Text','N:','Position',[412 TOP 20 22]);
        end
        pd_n = uieditfield(pf,'numeric','Value',c.N,'Limits',[1 1e6],...
               'RoundFractionalValues','on','Position',[434 TOP 66 22]);

        % ── Dimensiones de paneles de contenido ───────────────────────────────
        BTN_Y  = 10;
        BODY_Y = BTN_Y + 34;                 % panel empieza aquí
        BODY_H = TOP - 8 - BODY_Y;           % altura disponible para paneles
        PTF_W  = 320;
        PSS_W  = PW - PTF_W - 20;

        % ── Sub-panel TF ──────────────────────────────────────────────────────
        p_TF = uipanel(pf,'Title','TF — b(z)/a(z)  orden 0..5 (b(1) = coef z^orden)',...
               'Position',[5 BODY_Y PTF_W BODY_H]);
        tdata = [c.tf_b, c.tf_a];
        p_tbl = uitable(p_TF,'Data',tdata,...
                'ColumnName',{'b','a'},'RowName',{'0','1','2','3','4','5'},...
                'ColumnEditable',[true true],...
                'Position',[6 34 250 BODY_H-60]);
        uilabel(p_TF,'Text','Orden (0..5):','Position',[6 10 88 18]);
        p_ord = uieditfield(p_TF,'numeric','Value',c.tf_ord,'Limits',[0 5],...
                'RoundFractionalValues','on','Position',[96 8 50 22]);

        % ── Sub-panel SS ──────────────────────────────────────────────────────
        p_SS = uipanel(pf,'Title','SS — matrices (A 2×2, B 2×1, C 1×2, D escalar)',...
               'Position',[PTF_W+10 BODY_Y PSS_W BODY_H]);

        % Layout interno SS — columnas y filas claramente nombrados
        FW = 60; GAP = 4; GCOL = 12;
        xA  = 8;
        xA2 = xA  + FW + GAP;
        xB  = xA2 + FW + GCOL;
        xC  = xB  + FW + GCOL;
        xKi = xB;
        xNb = xC;

        % Filas desde arriba del panel (y desde fondo, BODY_H-titulo-offset)
        BASE = BODY_H - 22;  % alto disponible dentro del panel
        yHdr1 = BASE - 22;
        yR1   = yHdr1 - 26;
        yR2   = yR1   - 28;
        yR_D  = yR2   - 36;
        yHdr2 = yR_D  - 32;
        yR3   = yHdr2 - 26;
        yR4   = yR3   - 28;

        uilabel(p_SS,'Text','A (2×2)','Position',[xA yHdr1 70 18],'FontWeight','bold');
        uilabel(p_SS,'Text','B (2×1)','Position',[xB yHdr1 60 18],'FontWeight','bold');
        uilabel(p_SS,'Text','C (1×2)','Position',[xC yHdr1 60 18],'FontWeight','bold');

        p_A11 = ssEdt(p_SS,xA, yR1, c.ss_A(1,1));
        p_A12 = ssEdt(p_SS,xA2,yR1, c.ss_A(1,2));
        p_A21 = ssEdt(p_SS,xA, yR2, c.ss_A(2,1));
        p_A22 = ssEdt(p_SS,xA2,yR2, c.ss_A(2,2));
        p_B1  = ssEdt(p_SS,xB, yR1, c.ss_B(1));
        p_B2  = ssEdt(p_SS,xB, yR2, c.ss_B(2));
        p_C1  = ssEdt(p_SS,xC, yR1, c.ss_C(1));
        p_C2  = ssEdt(p_SS,xC, yR2, c.ss_C(2));

        uilabel(p_SS,'Text','D:','Position',[xA yR_D+2 18 18]);
        p_D  = ssEdt(p_SS,xA+20,yR_D, c.ss_D);

        uilabel(p_SS,'Text','L (2×1)','Position',[xA  yHdr2 60 18],'FontWeight','bold');
        uilabel(p_SS,'Text','K (1×2)','Position',[xA2 yHdr2 60 18],'FontWeight','bold');
        uilabel(p_SS,'Text','Ki:',    'Position',[xKi yHdr2 24 18],'FontWeight','bold');
        uilabel(p_SS,'Text','Nbar:',  'Position',[xNb yHdr2 36 18],'FontWeight','bold');

        p_L1  = ssEdt(p_SS,xA, yR3, c.ss_L(1));
        p_L2  = ssEdt(p_SS,xA, yR4, c.ss_L(2));
        p_K1  = ssEdt(p_SS,xA2,yR3, c.ss_K(1));
        p_K2  = ssEdt(p_SS,xA2,yR4, c.ss_K(2));
        p_Ki  = ssEdt(p_SS,xKi,yR3, c.ss_Ki);
        p_Nb  = ssEdt(p_SS,xNb,yR3, c.ss_Nbar);
        p_Nb.ValueChangedFcn = @(~,~) pSyncKiNbar();

        % ── Botones ───────────────────────────────────────────────────────────
        uibutton(pf,'Text','✔  Aplicar y cerrar',...
            'Position',[PW-286 BTN_Y 180 28],...
            'BackgroundColor',[0.12 0.62 0.12],'FontColor','w',...
            'ButtonPushedFcn',@pDoApply);
        uibutton(pf,'Text','✖  Cerrar',...
            'Position',[PW-100 BTN_Y 92 28],...
            'ButtonPushedFcn',@(~,~) delete(pf));

        pRefresh();

        % ── Callbacks internos del popup ──────────────────────────────────────
        function pRefresh(~,~)
            mn = pd_m.Value;
            p_TF.Visible = strcmp(mn,'TF');
            p_SS.Visible = strcmp(mn,'SS');
            pd_o.Enable  = strcmp(mn,'SS');
            pc_i.Enable  = strcmp(mn,'SS');
        end

        function pSyncKiNbar(~,~)
            if ~pc_i.Value
                p_Ki.Value = p_Nb.Value;
            end
        end

        function pDoApply(~,~)
            c2 = S.cfg(pp);
            c2.mode    = pd_m.Value;
            c2.obs     = pd_o.Value;
            c2.has_int = pc_i.Value;
            c2.N       = max(1, round(pd_n.Value));

            % Leer tabla TF
            D = p_tbl.Data;
            if iscell(D)
                D2 = zeros(size(D));
                for ri=1:size(D,1), for ci2=1:size(D,2)
                    v = str2double(string(D{ri,ci2}));
                    if ~isfinite(v), v=0; end
                    D2(ri,ci2) = v;
                end, end
                D = D2;
            end
            D = double(D);
            c2.tf_b  = D(:,1); c2.tf_a = D(:,2);
            c2.tf_ord = p_ord.Value;
            a0 = c2.tf_a(1);
            if a0 ~= 0 && a0 ~= 1
                c2.tf_b = c2.tf_b/a0; c2.tf_a = c2.tf_a/a0; c2.tf_a(1)=1;
            end

            % Leer campos SS
            c2.ss_A   = [ssV(p_A11) ssV(p_A12); ssV(p_A21) ssV(p_A22)];
            c2.ss_B   = [ssV(p_B1); ssV(p_B2)];
            c2.ss_C   = [ssV(p_C1)  ssV(p_C2)];
            c2.ss_D   = ssV(p_D);
            c2.ss_L   = [ssV(p_L1); ssV(p_L2)];
            c2.ss_K   = [ssV(p_K1)  ssV(p_K2)];
            c2.ss_Ki  = ssV(p_Ki);
            c2.ss_Nbar= ssV(p_Nb);

            S.cfg(pp) = c2;

            % Sincronizar panel compacto
            ddMode(pp).Value = c2.mode;
            ddObs(pp).Value  = c2.obs;
            cbInt(pp).Value  = c2.has_int;
            edtN(pp).Value   = c2.N;

            onApplyCtrl(pp);
            delete(pf);
        end

    end % openCtrlPopup

    function popCtrlClosed(pp)
        S.ctrl_popup_fig{pp} = [];
    end

%% ═══ POPUP — MODELO SIMULACIÓN ═══════════════════════════════════════════════

    function openSimPopup(pp)
        if ~isempty(S.sim_popup_fig{pp}) && isvalid(S.sim_popup_fig{pp})
            figure(S.sim_popup_fig{pp}); return;
        end
        SW = 530; SH = 370;
        sf = uifigure('Name',sprintf('Modelo Simulación — Planta %d',pp-1),...
             'Position',[350+pp*20 200 SW SH],'Resize','off');
        S.sim_popup_fig{pp} = sf;
        sf.DeleteFcn = @(~,~) simPopupClosed(pp);

        % Fila superior
        uilabel(sf,'Text','Tipo:',   'Position',[ 8 SH-40  36 22]);
        sd_type = uidropdown(sf,'Items',{'TF','SS'},'Value','TF',...
                  'Position',[46 SH-40 80 22],'ValueChangedFcn',@sRefresh);
        uilabel(sf,'Text','Dominio:','Position',[136 SH-40 54 22]);
        sd_dom  = uidropdown(sf,'Items',{'Continuo','Discreto'},'Value','Discreto',...
                  'Position',[192 SH-40 108 22]);
        uilabel(sf,'Text','Ts (s):','Position',[310 SH-40 44 22]);
        sd_ts   = uieditfield(sf,'numeric','Value',S.ctrl.Ts,...
                  'Limits',[1e-6 10],'Position',[356 SH-40 78 22]);

        % Panel TF
        BTN_Y2 = 10; BODY_Y2 = BTN_Y2+38; BODY_H2 = SH-40-8-BODY_Y2;
        sp_TF = uipanel(sf,'Title','TF — num y den como vectores MATLAB  (ej: [1 0.5])',...
                'Position',[5 BODY_Y2 SW-10 BODY_H2]);
        uilabel(sp_TF,'Text','Num:','Position',[8 BODY_H2-48 36 22]);
        s_num = uieditfield(sp_TF,'text','Value','[1]',...
                'Position',[46 BODY_H2-48 SW-64 22]);
        uilabel(sp_TF,'Text','Den:','Position',[8 BODY_H2-80 36 22]);
        s_den = uieditfield(sp_TF,'text','Value','[1 1]',...
                'Position',[46 BODY_H2-80 SW-64 22]);

        % Panel SS
        sp_SS = uipanel(sf,'Title','SS — A,B,C,D como expresiones MATLAB (ej: [1 0;0 0.9])',...
                'Position',[5 BODY_Y2 SW-10 BODY_H2]);
        slbls = {'A:','B:','C:','D:'};
        sdefs = {'[1 0;0 1]','[0;1]','[1 0]','[0]'};
        s_ss  = cell(4,1);
        for fi=1:4
            uilabel(sp_SS,'Text',slbls{fi},'Position',[8 BODY_H2-44-(fi-1)*34 24 22]);
            s_ss{fi} = uieditfield(sp_SS,'text','Value',sdefs{fi},...
                       'Position',[34 BODY_H2-44-(fi-1)*34 SW-52 22]);
        end

        uilabel(sf,'Text','ŷ (cyan punteado) se compara con y en los gráficos.',...
            'Position',[8 BTN_Y2+4 300 18],'FontSize',9,'FontColor',[0.4 0.4 0.4]);

        uibutton(sf,'Text','✔  Aplicar','Position',[SW-238 BTN_Y2 116 28],...
            'BackgroundColor',[0.12 0.62 0.12],'FontColor','w',...
            'ButtonPushedFcn',@sDoApply);
        uibutton(sf,'Text','✖  Cerrar','Position',[SW-116 BTN_Y2 108 28],...
            'ButtonPushedFcn',@(~,~) delete(sf));

        sRefresh();

        function sRefresh(~,~)
            sp_TF.Visible = strcmp(sd_type.Value,'TF');
            sp_SS.Visible = strcmp(sd_type.Value,'SS');
        end

        function sDoApply(~,~)
            try
                is_cont = strcmp(sd_dom.Value,'Continuo');
                Ts_s    = sd_ts.Value;

                if strcmp(sd_type.Value,'TF')
                    num_v = str2num(strtrim(s_num.Value)); %#ok<ST2NM>
                    den_v = str2num(strtrim(s_den.Value)); %#ok<ST2NM>
                    if isempty(num_v)||isempty(den_v), error('Num/Den inválidos.'); end
                    [Ac,Bc,Cc,Dc] = tf2ss(num_v(:)',den_v(:)');
                else
                    Ac = str2num(strtrim(s_ss{1}.Value)); %#ok<ST2NM>
                    Bc = str2num(strtrim(s_ss{2}.Value)); %#ok<ST2NM>
                    Cc = str2num(strtrim(s_ss{3}.Value)); %#ok<ST2NM>
                    Dc = str2num(strtrim(s_ss{4}.Value)); %#ok<ST2NM>
                    if any(cellfun(@isempty,{Ac,Bc,Cc,Dc}))
                        error('Matrices SS inválidas.'); end
                end

                if is_cont
                    [Ad,Bd] = c2d_zoh(Ac,Bc,Ts_s);
                    Cd = Cc; Dd = Dc;
                else
                    Ad = Ac; Bd = Bc; Cd = Cc; Dd = Dc;
                end

                ns = size(Ad,1);
                if size(Bd,1)~=ns||size(Cd,2)~=ns
                    error('Dimensiones inconsistentes.'); end
                if ~isequal(size(Cd),[1 ns])||~isequal(size(Dd),[1 1])
                    Dd = Dd(1,1);
                    if size(Cd,1)~=1, error('C debe ser fila 1×N.'); end
                end

                S.cfg(pp).sim_Ad = Ad; S.cfg(pp).sim_Bd = Bd;
                S.cfg(pp).sim_Cd = Cd; S.cfg(pp).sim_Dd = Dd(1,1);
                S.cfg(pp).sim_x  = zeros(ns,1);
                S.cfg(pp).sim_enabled = true;
                cbSimEn(pp).Value = true;

                logMsg(sprintf('P%d: modelo sim cargado. %s %s  %d estados.',...
                    pp-1, sd_type.Value, sd_dom.Value, ns));
                delete(sf);
            catch e
                uialert(sf, string(e.message), 'Error — modelo simulación');
            end
        end

    end % openSimPopup

    function simPopupClosed(pp)
        S.sim_popup_fig{pp} = [];
    end

%% ═══ UART BAJO NIVEL ═════════════════════════════════════════════════════════

    function ll_flush()
        if ~isempty(S.sp), try flush(S.sp); catch, end; end
    end

    function b = ll_readexact(n, tmo)
        if nargin<2, tmo=STEP_TMO; end
        t0=tic; buf=zeros(1,n,'uint8'); k=0;
        while k<n
            if toc(t0)>tmo, error('Timeout (%d/%d B)',k,n); end
            av=S.sp.NumBytesAvailable;
            if av>0
                m=min(av,n-k); tmp=read(S.sp,m,"uint8");
                buf(k+1:k+m)=tmp(:)'; k=k+m;
            else, pause(0.001); end
        end
        b=buf(:);
    end

    function rsp = ll_cmd_wait(cmd, tmo)
        if nargin<2, tmo=STEP_TMO; end
        write(S.sp,uint8(cmd),"uint8");
        t0=tic;
        while true
            if toc(t0)>tmo, error("Timeout cmd '%s'",cmd); end
            if S.sp.NumBytesAvailable>0
                b=read(S.sp,1,"uint8"); b=b(1);
                if any(b==uint8(['R','K','!'])), rsp=b; return; end
            else, pause(0.001); end
        end
    end

    function ll_send_payload(data_u8)
        data_u8=uint8(data_u8(:)); n=numel(data_u8); idx=1;
        while idx<=n
            w=uint8([0;0;0;0]); take=min(4,n-idx+1);
            w(1:take)=data_u8(idx:idx+take-1);
            tries=0;
            while true
                tries=tries+1; if tries>MAX_RETRIES, error('Max retries idx=%d',idx); end
                write(S.sp,w,"uint8");
                echo=ll_readexact(4); match=isequal(echo(:),w(:));
                if match, write(S.sp,uint8('A'),"uint8");
                else,     write(S.sp,uint8('N'),"uint8"); end
                conf=ll_readexact(1);
                if match && conf(1)==uint8('A'), break; end
            end
            idx=idx+take;
        end
    end

%% ═══ PROTOCOLO UART ══════════════════════════════════════════════════════════

    function uartp_reset()
        ll_flush(); rsp=ll_cmd_wait('r');
        if rsp~=uint8('K'), error("reset: rsp=%c",char(rsp)); end
    end

    function uartp_set_fs(fs_hz)
        ll_flush(); rsp=ll_cmd_wait('f');
        if rsp~=uint8('R'), error("set_fs: rsp=%c",char(rsp)); end
        ll_send_payload(typecast(single(fs_hz),'uint8'));
        fin=ll_readexact(1);
        if fin(1)~=uint8('K'), error("set_fs fin=%c",char(fin(1))); end
    end

    function uartp_start_run()
        ll_flush(); rsp=ll_cmd_wait('i');
        if rsp~=uint8('K'), error("start: rsp=%c",char(rsp)); end
    end

    function uartp_stop()
        try, write(S.sp,uint8([0,0,uint8('s')]),"uint8"); catch, end
        t0=tic;
        while toc(t0)<1.5
            if S.sp.NumBytesAvailable>0
                b=read(S.sp,1,"uint8");
                if b(1)==uint8('K'), return; end
            else, pause(0.002); end
        end
    end

    function send_u(u_int16)
        write(S.sp, typecast(int16(u_int16),'uint8'), "uint8");
    end

%% ═══ ACCIONES ════════════════════════════════════════════════════════════════

    function onConnToggle(~,~)
        if ~S.isConnected
            com=strtrim(string(edtCom.Value));
            if com=="", logMsg("Puerto vacío."); return; end
            try
                S.sp=serialport(com,edtBaud.Value); S.sp.Timeout=0.1;
                flush(S.sp); pause(0.05); ll_flush();
                S.isConnected=true;
                btnConn.Text='Desconectar'; btnConn.BackgroundColor=[0.65 0.15 0.15];
                lblStat.Text="CONECTADO: "+com;
                logMsg("Conectado: "+com+" @ "+edtBaud.Value);
                startStream();
            catch e
                logMsg("Error: "+string(e.message));
                S.sp=[]; S.isConnected=false;
            end
        else
            stopStream();
            try, if ~isempty(S.sp), flush(S.sp); delete(S.sp); end, catch, end
            S.sp=[]; S.isConnected=false;
            btnConn.Text='Conectar'; btnConn.BackgroundColor=[0.2 0.62 0.2];
            lblStat.Text='Desconectado'; logMsg("Desconectado.");
        end
    end

    function onSaveSession(~,~)
        [f,p]=uiputfile('*.mat','Guardar sesión'); if isequal(f,0), return; end
        try
            sess = struct();
            sess.cfg1 = S.cfg(1); sess.cfg1.sim_x = [];
            sess.cfg2 = S.cfg(2); sess.cfg2.sim_x = [];
            sess.Fs     = edtFs(1).Value;
            sess.SatMin = edtSatMin.Value; sess.SatMax = edtSatMax.Value;
            sess.SU1=edtSU1.Value; sess.SU2=edtSU2.Value;
            sess.SY1=edtSY1.Value; sess.SY2=edtSY2.Value;
            save(fullfile(p,f),'-struct','sess');
            logMsg("Sesión guardada: "+string(f));
        catch e, logMsg("Guardar FAIL: "+string(e.message)); end
    end

    function onLoadSession(~,~)
        [f,p]=uigetfile('*.mat','Cargar sesión'); if isequal(f,0), return; end
        try
            sess=load(fullfile(p,f));
            if isfield(sess,'cfg1')
                S.cfg(1)=sess.cfg1; S.cfg(1).sim_x=[];
                ddMode(1).Value=S.cfg(1).mode; ddObs(1).Value=S.cfg(1).obs;
                cbInt(1).Value=S.cfg(1).has_int; edtN(1).Value=S.cfg(1).N;
                cbSimEn(1).Value=S.cfg(1).sim_enabled;
            end
            if isfield(sess,'cfg2')
                S.cfg(2)=sess.cfg2; S.cfg(2).sim_x=[];
                ddMode(2).Value=S.cfg(2).mode; ddObs(2).Value=S.cfg(2).obs;
                cbInt(2).Value=S.cfg(2).has_int; edtN(2).Value=S.cfg(2).N;
                cbSimEn(2).Value=S.cfg(2).sim_enabled;
            end
            if isfield(sess,'Fs'),     edtFs(1).Value=sess.Fs; end
            if isfield(sess,'SatMin'), edtSatMin.Value=sess.SatMin; end
            if isfield(sess,'SatMax'), edtSatMax.Value=sess.SatMax; end
            if isfield(sess,'SU1'),    edtSU1.Value=sess.SU1; end
            if isfield(sess,'SU2'),    edtSU2.Value=sess.SU2; end
            if isfield(sess,'SY1'),    edtSY1.Value=sess.SY1; end
            if isfield(sess,'SY2'),    edtSY2.Value=sess.SY2; end
            for pp2=1:2, onModeChange(pp2); updateCfgStatus(pp2); end
            logMsg("Sesión cargada: "+string(f));
        catch e, logMsg("Cargar FAIL: "+string(e.message)); end
    end

    function onApplyRef(~,~)
        S.ctrl.inner.ref = edtRef.Value;
        logMsg(sprintf("Ref inner → %.4g", edtRef.Value));
    end

    function onStart(~,~)
        if ~reqConn(), return; end
        for pp2=1:2, onApplyCtrl(pp2); end
        S.ctrl.Ts      = 1/edtFs(1).Value;
        S.ctrl.sat_min = edtSatMin.Value;
        S.ctrl.sat_max = edtSatMax.Value;
        ref0 = edtRef.Value;
        S.ctrl.inner = ctrl_reset_states(S.ctrl.inner); S.ctrl.inner.ref = ref0;
        S.ctrl.outer = ctrl_reset_states(S.ctrl.outer); S.ctrl.outer.ref = 0;
        % Reset estados de simulación
        for pp2=1:2
            if ~isempty(S.cfg(pp2).sim_Ad)
                S.cfg(pp2).sim_x = zeros(size(S.cfg(pp2).sim_Ad,1),1);
            end
        end
        updateAxesLayout();
        try
            S.inTxn=true; stopStream(); ll_flush();
            uartp_reset();
            uartp_set_fs(edtFs(1).Value);
            uartp_start_run();
            S.streamOn=true; S.rxBuf=uint8([]);
            S.autoStopArmed=S.autoStopEn && S.autoStopN>0;
            lblRunSt.Text='▶ CORRIENDO'; lblRunSt.FontColor=[0 0.5 0];
            logMsg(sprintf("Iniciado. Fs=%.1fHz  ref=%.4g  sat=[%.0f,%.0f]",...
                edtFs(1).Value,ref0,S.ctrl.sat_min,S.ctrl.sat_max));
        catch e
            logMsg("Start FAIL: "+string(e.message)); S.streamOn=false;
        end
        S.inTxn=false; startStream();
    end

    function onStop(~,~)
        if ~reqConn(), return; end
        try
            S.streamOn=false; S.inTxn=true;
            stopStream(); ll_flush();
            uartp_stop();
            lblRunSt.Text='Detenido'; lblRunSt.FontColor=[0 0 0];
            logMsg("Control detenido.");
        catch e, logMsg("Stop FAIL: "+string(e.message)); end
        S.inTxn=false; startStream();
    end

    function onAutoStopToggle(~,~)
        S.autoStopEn=cbAutoStop.Value; S.autoStopN=edtAutoN.Value;
    end

%% ═══ STREAMING ═══════════════════════════════════════════════════════════════

    function startStream()
        stopStream();
        if ~S.isConnected||isempty(S.sp), return; end
        S.streamTimer=timer('ExecutionMode','fixedSpacing',...
            'Period',STREAM_PERIOD,'BusyMode','drop','TimerFcn',@onStreamTick);
        start(S.streamTimer);
    end

    function stopStream()
        try
            if ~isempty(S.streamTimer) && isvalid(S.streamTimer)
                stop(S.streamTimer); delete(S.streamTimer);
            end
        catch, end
        S.streamTimer=[];
    end

    function onStreamTick(~,~)
        if S.inTxn||~S.isConnected||isempty(S.sp), return; end
        try
            nAv=S.sp.NumBytesAvailable;
            if nAv>0
                raw=read(S.sp,nAv,"uint8");
                S.rxBuf=[S.rxBuf; uint8(raw(:))];
            end
            if ~S.streamOn, return; end
            nProc=0;
            while numel(S.rxBuf)>=FRAME_SZ
                frame=S.rxBuf(1:FRAME_SZ); S.rxBuf=S.rxBuf(FRAME_SZ+1:end);
                processControlFrame(frame);
                nProc=nProc+1;
                if nProc>=8, break; end
            end
            if S.autoStopArmed && S.framesTotal>=S.autoStopN
                S.autoStopArmed=false;
                logMsg(sprintf("Auto-stop: %d frames.",S.framesTotal));
                onStop(); return;
            end
            if nProc>0
                updatePlots(); updateDataInfo();
            end
        catch e
            S.rxBuf=uint8([]);
            logMsg("stream WARN: "+string(e.message));
        end
    end

    function processControlFrame(frame)
        % Decodificar trama PSoC
        theta_cnt  = double(typecast(uint8(frame(1:4)),'int32'));
        delta_om_c = double(typecast(uint8(frame(5:6)),'int16'));
        Ts = S.ctrl.Ts;
        y2 = theta_cnt  * (2*pi / PENDULO_CPR);        % θ [rad]
        y1 = delta_om_c * (2*pi / MOTOR_CPR) / Ts;     % ω [rad/s]

        % Ejecutar control (double precision)
        [u_sat,u_unsat,x1i,x2i,x1o,x2o,ctrl_new] = run_control(S.ctrl,y1,y2);
        S.ctrl = ctrl_new;

        % Enviar esfuerzo al PSoC (int16 LE)
        u_clamped = max(-32768, min(32767, round(u_sat)));
        send_u(u_clamped);

        % ── Simulación planta inner (input = u_sat → output ≈ ω) ─────────────
        ysim1 = nan;
        c1 = S.cfg(1);
        if c1.sim_enabled && ~isempty(c1.sim_Ad)
            ysim1 = c1.sim_Cd * c1.sim_x + c1.sim_Dd * u_sat;
            S.cfg(1).sim_x = c1.sim_Ad * c1.sim_x + c1.sim_Bd * u_sat;
        end

        % ── Simulación planta outer (input = inner.ref → output ≈ θ) ─────────
        ysim2 = nan;
        c2 = S.cfg(2);
        if c2.sim_enabled && ~isempty(c2.sim_Ad)
            u2_in = S.ctrl.inner.ref;   % salida outer = referencia inner
            ysim2 = c2.sim_Cd * c2.sim_x + c2.sim_Dd * u2_in;
            S.cfg(2).sim_x = c2.sim_Ad * c2.sim_x + c2.sim_Bd * u2_in;
        end

        % ── Guardar telemetría — todos en el mismo índice n (sincronizados) ───
        n = S.framesTotal + 1;
        S.nVec(end+1,1)        = n;
        S.u1Vec(end+1,1)       = u_sat;
        S.u1unsat_Vec(end+1,1) = u_unsat;
        S.y1Vec(end+1,1)       = y1;
        S.u2Vec(end+1,1)       = S.ctrl.inner.ref;   % R inner capturado post-control
        S.y2Vec(end+1,1)       = y2;
        S.x1iVec(end+1,1)      = x1i;
        S.x2iVec(end+1,1)      = x2i;
        S.x1oVec(end+1,1)      = x1o;
        S.x2oVec(end+1,1)      = x2o;
        S.ySimVec1(end+1,1)    = ysim1;
        S.ySimVec2(end+1,1)    = ysim2;
        S.framesTotal = n;

        if numel(S.nVec)>MAX_PTS
            k=numel(S.nVec)-MAX_PTS+1;
            S.nVec=S.nVec(k:end);
            S.u1Vec=S.u1Vec(k:end);           S.u1unsat_Vec=S.u1unsat_Vec(k:end);
            S.y1Vec=S.y1Vec(k:end);           S.u2Vec=S.u2Vec(k:end);
            S.y2Vec=S.y2Vec(k:end);
            S.x1iVec=S.x1iVec(k:end);        S.x2iVec=S.x2iVec(k:end);
            S.x1oVec=S.x1oVec(k:end);        S.x2oVec=S.x2oVec(k:end);
            S.ySimVec1=S.ySimVec1(k:end);     S.ySimVec2=S.ySimVec2(k:end);
        end
    end

%% ═══ ZOH DISCRETIZACIÓN (sin toolbox) ════════════════════════════════════════

    function [Ad, Bd] = c2d_zoh(Ac, Bc, Ts)
        % Discretización por ZOH usando la exponencial de matriz
        n  = size(Ac,1);
        nu = size(Bc,2);
        M  = expm([Ac, Bc; zeros(nu, n+nu)] * Ts);
        Ad = M(1:n,    1:n);
        Bd = M(1:n, n+1:end);
    end

%% ═══ DATOS ═══════════════════════════════════════════════════════════════════

    function onExport(~,~)
        if isempty(S.nVec), uialert(fig,'No hay datos.','Exportar'); return; end
        [f,p]=uiputfile('*.mat','Exportar datos'); if isequal(f,0), return; end
        try
            data=struct();
            data.n=S.nVec; data.u1=S.u1Vec; data.u1_unsat=S.u1unsat_Vec;
            data.y1=S.y1Vec; data.u2=S.u2Vec; data.y2=S.y2Vec;
            data.x1i=S.x1iVec; data.x2i=S.x2iVec;
            data.x1o=S.x1oVec; data.x2o=S.x2oVec;
            data.ysim1=S.ySimVec1; data.ysim2=S.ySimVec2;
            data.fecha    = datestr(now,'yyyy-mm-dd HH:MM:SS');
            data.Fs_inner = edtFs(1).Value;
            data.M_plant0 = edtN(1).Value;
            data.N_plant1 = edtN(2).Value;
            data.Fs_outer = data.Fs_inner / max(1,data.N_plant1);
            data.sat_min  = edtSatMin.Value;
            data.sat_max  = edtSatMax.Value;
            data.cfg      = S.cfg;
            data.ctrl_final = S.ctrl;
            save(fullfile(p,f),'-struct','data');
            logMsg("Exportado: "+string(f));
        catch e, logMsg("Export FAIL: "+string(e.message)); end
    end

    function onClear(~,~)
        S.nVec=[]; S.u1Vec=[]; S.u1unsat_Vec=[];
        S.y1Vec=[]; S.u2Vec=[]; S.y2Vec=[];
        S.x1iVec=[]; S.x2iVec=[]; S.x1oVec=[]; S.x2oVec=[];
        S.ySimVec1=[]; S.ySimVec2=[];
        S.framesTotal=0; S.rxBuf=uint8([]);
        updatePlots(); updateDataInfo();
        logMsg("Datos borrados.");
    end

%% ═══ CIERRE ══════════════════════════════════════════════════════════════════

    function onClose(~,~)
        try, stopStream(); catch, end
        try
            if S.isConnected && ~isempty(S.sp)
                try, flush(S.sp); catch, end
                delete(S.sp);
            end
        catch, end
        for pp2=1:2
            try
                if ~isempty(S.ctrl_popup_fig{pp2}) && isvalid(S.ctrl_popup_fig{pp2})
                    delete(S.ctrl_popup_fig{pp2});
                end
            catch, end
            try
                if ~isempty(S.sim_popup_fig{pp2}) && isvalid(S.sim_popup_fig{pp2})
                    delete(S.sim_popup_fig{pp2});
                end
            catch, end
        end
        delete(fig);
    end

end % pendulo_gui2
