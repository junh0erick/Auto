function pendulo_gui2()
%PENDULO_GUI2  GUI Péndulo Invertido PSoC 5LP — v6
%  Control en PSoC (ctrl_pend.c). MATLAB = display + configuración.
%  Protocolo v6.2: MATLAB→'p'(216B params)→'v'(eco)→'i'→PSoC streaming 32B/tick
%
%  Planta 1 (inner): Motor / ω → u_pwm
%  Planta 2 (outer): Péndulo / θ → ref_inner (decimado por N)

%% ═══ CONSTANTES FÍSICAS Y PROTOCOLO ═════════════════════════════════════════
TELEM_SZ    = 32;       % bytes por trama PSoC→MATLAB (8×float32: y1 y2 u1 u2 x1i x2i x1o x2o)
MAX_PTS     = 500000;   % máx muestras en memoria
STEP_TMO    = 0.8;      % timeout handshake [s]
MAX_RETRIES = 50;       % reintentos handshake

MOTOR_CPR   = 1040.0;
PENDULO_CPR = 10000.0;

MODE_NAMES = {'TF','SS','Open-loop','Off'};

%% ═══ GEOMETRÍA — editar aquí para ajustar la interfaz ═══════════════════════
FIG_W  = 1500;  FIG_H  = 720;

AX_X   = 8;     AX_W   = 892;
AX_BOT = 10;    AX_H   = 170;   AX_GAP = 4;

RX     = 908;   RW     = 585;   RGAP   = 4;

H_LOG   = 86;
H_VIZ   = 80;
H_DATA  = 44;
H_CTRL  = 100;
H_PLANT = 165;
H_CONN  = 35;

Y_LOG    = 5;
Y_VIZ    = Y_LOG    + H_LOG    + RGAP;
Y_DATA   = Y_VIZ    + H_VIZ    + RGAP;
Y_CTRL   = Y_DATA   + H_DATA   + RGAP;
Y_PLANT0 = Y_CTRL   + H_CTRL   + RGAP;
Y_PLANT1 = Y_PLANT0 + H_PLANT  + RGAP;
Y_CONN   = Y_PLANT1 + H_PLANT  + RGAP;

PP_PX  = 8;     % margen horizontal
PP_RH  = 24;    % alto fila estándar
PP_RG  = 4;     % gap entre filas

PP_Y1  = 6;                           % fila 1: botones popup
PP_Y2  = PP_Y1 + PP_RH + PP_RG;      % fila 2: Fs o N
PP_Y3  = PP_Y2 + PP_RH + PP_RG;      % fila 3: Modo/Obs/Int
PP_Y4  = PP_Y3 + PP_RH + PP_RG;      % fila 4: cfg status label
PP_Y5  = PP_Y4 + 18    + PP_RG;      % fila 5: RMS error label

ax_y = @(k) AX_BOT + k * (AX_H + AX_GAP);

%% ═══ ESTADO PRINCIPAL S ═════════════════════════════════════════════════════
S = struct();
S.sp = [];  S.isConnected = false;
S.streamOn = false;  S.inLoop = false;

S.nVec        = zeros(0,1);
S.u1Vec       = zeros(0,1);
S.u1unsat_Vec = zeros(0,1);
S.y1Vec       = zeros(0,1);
S.u2Vec       = zeros(0,1);   % R inner = referencia lazo inner
S.y2Vec       = zeros(0,1);
S.x1iVec      = zeros(0,1);
S.x2iVec      = zeros(0,1);
S.x1oVec      = zeros(0,1);
S.x2oVec      = zeros(0,1);
S.ySimVec1    = zeros(0,1);
S.ySimVec2    = zeros(0,1);
S.framesTotal = 0;

S.rms1_sum2 = 0;  S.rms1_n = 0;   % error RMS Planta 1
S.rms2_sum2 = 0;  S.rms2_n = 0;   % error RMS Planta 2

S.autoStopEn = false;  S.autoStopN = 0;  S.autoStopArmed = false;
S.last_sent_payload = [];   % último payload 216B enviado y verificado al PSoC
S.ctrl = ctrl_state_default();

S.scalers = struct('su1',1,'su2',1,'sy1',1,'sy2',1,...
                   'sysim1',1,'sysim2',1,'sr',1,...
                   'sx1i',1,'sx2i',1,'sx1o',1,'sx2o',1);

S.cfg(1) = cfg_default();  % inner (Planta 1)
S.cfg(2) = cfg_default();  % outer (Planta 2)
S.cfg(1).Fs = 200;

S.ctrl_popup_fig   = {[], []};
S.sim_popup_fig    = {[], []};
S.scaler_popup_fig = [];

%% ═══ FIGURA ══════════════════════════════════════════════════════════════════
fig = uifigure('Name','Péndulo Invertido — Control v6 (PSoC ctrl)',...
               'Position',[20 40 FIG_W FIG_H]);
fig.CloseRequestFcn = @onClose;

%% ═══ EJES ════════════════════════════════════════════════════════════════════
ax_y2 = uiaxes(fig,'Position',[AX_X ax_y(0) AX_W AX_H]);
ax_u2 = uiaxes(fig,'Position',[AX_X ax_y(1) AX_W AX_H]);
ax_y1 = uiaxes(fig,'Position',[AX_X ax_y(2) AX_W AX_H]);
ax_u1 = uiaxes(fig,'Position',[AX_X ax_y(3) AX_W AX_H]);

setupAx(ax_u1,'u₁ [PWM]  esfuerzo motor + R referencia','u₁');
setupAx(ax_y1,'y₁ = ω motor [rad/s]  /  x̂ inner  /  ŷ sim','ω');
setupAx(ax_u2,'u₂ = R inner  (salida outer → referencia lazo inner)','u₂');
setupAx(ax_y2,'y₂ = θ péndulo [rad]  /  x̂ outer  /  ŷ sim','θ');

hold(ax_u1,'on');
hU1sat   = stairs(ax_u1,nan,nan,'b-', 'LineWidth',1.5,'DisplayName','u₁ sat');
hU1unsat = stairs(ax_u1,nan,nan,'b--','LineWidth',0.8,'DisplayName','u₁ unsat');
hR1      = stairs(ax_u1,nan,nan,'r-', 'LineWidth',1.2,'DisplayName','R (ref inner)');
hold(ax_u1,'off');  legend(ax_u1,'show');

hold(ax_y1,'on');
hY1    = plot(ax_y1,nan,nan,'b-', 'LineWidth',1.5,'DisplayName','ω meas');
hY1sim = plot(ax_y1,nan,nan,'c--','LineWidth',1.2,'DisplayName','ω sim');
hX1i   = plot(ax_y1,nan,nan,'g-', 'LineWidth',1.0,'DisplayName','x̂₁ᵢ');
hX2i   = plot(ax_y1,nan,nan,'m-', 'LineWidth',1.0,'DisplayName','x̂₂ᵢ');
hold(ax_y1,'off');  legend(ax_y1,'show');

hold(ax_u2,'on');
hU2 = stairs(ax_u2,nan,nan,'r-','LineWidth',1.5,'DisplayName','R inner');
hold(ax_u2,'off');  legend(ax_u2,'show');

hold(ax_y2,'on');
hY2    = plot(ax_y2,nan,nan,'r-', 'LineWidth',1.5,'DisplayName','θ meas');
hY2sim = plot(ax_y2,nan,nan,'c--','LineWidth',1.2,'DisplayName','θ sim');
hX1o   = plot(ax_y2,nan,nan,'g-', 'LineWidth',1.0,'DisplayName','x̂₁ₒ');
hX2o   = plot(ax_y2,nan,nan,'m-', 'LineWidth',1.0,'DisplayName','x̂₂ₒ');
hold(ax_y2,'off');  legend(ax_y2,'show');

%% ═══ PANEL DERECHO ═══════════════════════════════════════════════════════════

% ── 1. Conexión ───────────────────────────────────────────────────────────────
pConn = uipanel(fig,'Title','Conexión','Position',[RX Y_CONN RW H_CONN]);
uilabel(pConn,'Text','Puerto:','Position',[PP_PX      5 45 22]);
edtCom  = uieditfield(pConn,'text','Value','COM4','Position',[PP_PX+47  5 64 22]);
uilabel(pConn,'Text','Baud:',  'Position',[PP_PX+116  5 36 22]);
edtBaud = uieditfield(pConn,'numeric','Value',921600,'Limits',[1200 4e6],...
          'Position',[PP_PX+154 5 84 22]);
btnConn = uibutton(pConn,'Text','Conectar','Position',[PP_PX+244 4 98 24],...
          'BackgroundColor',[0.2 0.62 0.2],'FontColor','w',...
          'ButtonPushedFcn',@onConnToggle);
lblStat = uilabel(pConn,'Text','Desconectado','Position',[PP_PX+348 5 108 22]);
uibutton(pConn,'Text','Guardar','Position',[PP_PX+460 4 56 24],...
         'ButtonPushedFcn',@onSaveSession);
uibutton(pConn,'Text','Cargar', 'Position',[PP_PX+520 4 52 24],...
         'ButtonPushedFcn',@onLoadSession);

% ── 2 & 3. Paneles de planta (compactos) ─────────────────────────────────────
PLANT_Y   = [Y_PLANT0, Y_PLANT1];
PLANT_LBL = {'Planta 1 — Motor / ω  (inner)','Planta 2 — Péndulo / θ  (outer)'};

ddMode     = gobjects(2,1);
ddObs      = gobjects(2,1);
cbInt      = gobjects(2,1);
edtN       = gobjects(2,1);   % solo pp=2 visible
edtFs      = gobjects(2,1);   % solo pp=1 visible
lblCfgStat = gobjects(2,1);
lblRMS     = gobjects(2,1);
cbSimEn    = gobjects(2,1);

for pp = 1:2
    pP = uipanel(fig,'Title',PLANT_LBL{pp},...
         'Position',[RX PLANT_Y(pp) RW H_PLANT]);

    % Fila 5: RMS error simulación
    lblRMS(pp) = uilabel(pP,'Text','— sin sim —',...
        'Position',[PP_PX PP_Y5 RW-PP_PX*2 16],...
        'FontColor',[0.15 0.5 0.15],'FontSize',9);

    % Fila 4: resumen configuración
    lblCfgStat(pp) = uilabel(pP,'Text','— sin configurar —',...
        'Position',[PP_PX PP_Y4 RW-PP_PX*2 18],...
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

    % Fila 2: Fs (Planta 1) o N decimación (Planta 2)
    if pp == 1
        uilabel(pP,'Text','Fs (Hz):','Position',[PP_PX PP_Y2 54 22]);
        edtFs(pp) = uieditfield(pP,'numeric','Value',200,'Limits',[0.1 10000],...
            'Position',[PP_PX+56 PP_Y2 84 22],...
            'ValueChangedFcn',@(~,~) onFsChange());
        edtN(pp) = uieditfield(pP,'numeric','Value',1,'Limits',[1 1e6],...
            'RoundFractionalValues','on','Visible','off',...
            'Position',[PP_PX+24 PP_Y2 58 22]);
    else
        uilabel(pP,'Text','N decim:','Position',[PP_PX PP_Y2 58 22]);
        edtN(pp) = uieditfield(pP,'numeric','Value',1,'Limits',[1 1e6],...
            'RoundFractionalValues','on','Position',[PP_PX+62 PP_Y2 62 22],...
            'ValueChangedFcn',@(~,~) onNChange(pp));
        edtFs(pp) = uieditfield(pP,'numeric','Value',0,'Limits',[0 10000],...
            'Visible','off','Position',[PP_PX+144 PP_Y2 78 22]);
    end

    % Fila 1: botones popup
    uibutton(pP,'Text','⚙ Cargar controlador',...
        'Position',[PP_PX PP_Y1 158 PP_RH],...
        'ButtonPushedFcn',@(~,~) openCtrlPopup(pp));
    cbSimEn(pp) = uicheckbox(pP,'Text','Simular',...
        'Position',[PP_PX+166 PP_Y1 68 PP_RH],...
        'ValueChangedFcn',@(~,~) onSimToggle(pp));
    uibutton(pP,'Text','📈 Cargar modelo',...
        'Position',[PP_PX+240 PP_Y1 148 PP_RH],...
        'ButtonPushedFcn',@(~,~) openSimPopup(pp));
end
onModeChange(1);  onModeChange(2);

% ── 4. Control ────────────────────────────────────────────────────────────────
pCtrl = uipanel(fig,'Title','Control','Position',[RX Y_CTRL RW H_CTRL]);
yC1 = H_CTRL - 20 - PP_RH;
yC2 = yC1 - PP_RH - PP_RG;
yC3 = yC2 - PP_RH - PP_RG;

uilabel(pCtrl,'Text','Ref:','Position',[PP_PX yC1 30 22]);
edtRef = uieditfield(pCtrl,'numeric','Value',0,'Limits',[-1e6 1e6],...
         'Position',[PP_PX+32 yC1 66 22]);
btnStart = uibutton(pCtrl,'Text','▶  Start','Position',[PP_PX+104 yC1-1 98 PP_RH],...
           'BackgroundColor',[0.12 0.62 0.12],'FontColor','w','FontWeight','bold',...
           'ButtonPushedFcn',@onStart);
btnStop  = uibutton(pCtrl,'Text','■  Stop', 'Position',[PP_PX+208 yC1-1 88 PP_RH],...
           'BackgroundColor',[0.78 0.1 0.1],'FontColor','w','FontWeight','bold',...
           'ButtonPushedFcn',@onStop);
lblRunSt = uilabel(pCtrl,'Text','Detenido','Position',[PP_PX+302 yC1 120 26],...
           'FontWeight','bold');
% (btnSend removed — Start handles all param sending)

uilabel(pCtrl,'Text','u_min:','Position',[PP_PX yC2 42 22]);
edtSatMin = uieditfield(pCtrl,'numeric','Value',-1264,'Limits',[-1e6 0],...
            'Position',[PP_PX+44 yC2 66 22]);
uilabel(pCtrl,'Text','u_max:','Position',[PP_PX+116 yC2 42 22]);
edtSatMax = uieditfield(pCtrl,'numeric','Value', 1264,'Limits',[0 1e6],...
            'Position',[PP_PX+160 yC2 66 22]);
lblRTT = uilabel(pCtrl,'Text','RTT — ms','Position',[PP_PX+240 yC2 300 22],...
         'FontColor',[0.25 0.25 0.7]);

cbAutoStop = uicheckbox(pCtrl,'Text','Auto-stop en frames:','Value',false,...
             'Position',[PP_PX yC3 148 22],'ValueChangedFcn',@onAutoStopToggle);
edtAutoN   = uieditfield(pCtrl,'numeric','Value',2000,'Limits',[1 1e9],...
             'RoundFractionalValues','on','Position',[PP_PX+152 yC3 78 22]);
lblDataInf = uilabel(pCtrl,'Text','n=0','Position',[PP_PX+236 yC3 140 22]);
uilabel(pCtrl,'Text','Tipo num:','Position',[PP_PX+382 yC3 66 22]);
ddNumType  = uidropdown(pCtrl,...
             'Items',{'f32','f64','f16','q31','q15','q7'},...
             'Value','f32',...
             'Position',[PP_PX+450 yC3 120 22]);

% ── 5. Datos ──────────────────────────────────────────────────────────────────
pData = uipanel(fig,'Title','Datos','Position',[RX Y_DATA RW H_DATA]);
yD1 = H_DATA - 20 - PP_RH;
uibutton(pData,'Text','Exportar .mat', 'Position',[PP_PX      yD1 130 PP_RH],...
         'ButtonPushedFcn',@onExport);
uibutton(pData,'Text','Borrar datos',  'Position',[PP_PX+136  yD1 118 PP_RH],...
         'ButtonPushedFcn',@onClear);
uibutton(pData,'Text','⚙ Escaladores','Position',[PP_PX+260  yD1 142 PP_RH],...
         'ButtonPushedFcn',@(~,~) openScalerPopup());

% ── 6. Visualización ──────────────────────────────────────────────────────────
pViz = uipanel(fig,'Title','Visualización','Position',[RX Y_VIZ RW H_VIZ]);
yV1 = H_VIZ - 20 - PP_RH;
yV2 = yV1 - PP_RH - PP_RG;

% Fila 1: señales
uilabel(pViz,'Text','Señales:','Position',[PP_PX yV1+2 52 18]);
cbSigU     = uicheckbox(pViz,'Text','u sat',  'Value',true, 'Position',[PP_PX+ 56 yV1 58 22],'ValueChangedFcn',@updatePlots);
cbSigUnsat = uicheckbox(pViz,'Text','u unsat','Value',true, 'Position',[PP_PX+116 yV1 70 22],'ValueChangedFcn',@updatePlots);
cbSigR     = uicheckbox(pViz,'Text','R',      'Value',true, 'Position',[PP_PX+190 yV1 34 22],'ValueChangedFcn',@updatePlots);
cbSigY1    = uicheckbox(pViz,'Text','y₁',    'Value',true, 'Position',[PP_PX+226 yV1 40 22],'ValueChangedFcn',@updatePlots);
cbSigY2    = uicheckbox(pViz,'Text','y₂',    'Value',true, 'Position',[PP_PX+268 yV1 40 22],'ValueChangedFcn',@updatePlots);
cbSigSim1  = uicheckbox(pViz,'Text','ŷ₁',   'Value',true,'Visible',false, 'Position',[PP_PX+310 yV1 44 22],'ValueChangedFcn',@updatePlots);
cbSigSim2  = uicheckbox(pViz,'Text','ŷ₂',   'Value',true,'Visible',false, 'Position',[PP_PX+356 yV1 44 22],'ValueChangedFcn',@updatePlots);

% Fila 2: SS estados
uilabel(pViz,'Text','SS x̂:','Position',[PP_PX yV2+2 44 18]);
cbX1i = uicheckbox(pViz,'Text','x̂₁ᵢ','Value',false,'Position',[PP_PX+ 50 yV2 52 22],'ValueChangedFcn',@updatePlots);
cbX2i = uicheckbox(pViz,'Text','x̂₂ᵢ','Value',false,'Position',[PP_PX+104 yV2 52 22],'ValueChangedFcn',@updatePlots);
cbX1o = uicheckbox(pViz,'Text','x̂₁ₒ','Value',false,'Position',[PP_PX+158 yV2 52 22],'ValueChangedFcn',@updatePlots);
cbX2o = uicheckbox(pViz,'Text','x̂₂ₒ','Value',false,'Position',[PP_PX+212 yV2 52 22],'ValueChangedFcn',@updatePlots);

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
        ax.XGrid = 'on';  ax.YGrid = 'on';
        ax.Title.String  = ttl;
        ax.XLabel.String = 'frame';
        ax.YLabel.String = yl;
        ax.YLimMode = 'auto';
        ax.XLimMode = 'auto';
    end

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

%% ═══ CALLBACKS DE PANELES DE PLANTA ══════════════════════════════════════════

    function onModeChange(pp)
        mn = ddMode(pp).Value;
        S.cfg(pp).mode = mn;
        isSS = strcmp(mn,'SS');
        ddObs(pp).Enable = isSS;
        cbInt(pp).Enable = isSS;
        if ~isSS, cbInt(pp).Value = false;  S.cfg(pp).has_int = false; end
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
        % ŷ checkbox only visible when sim is enabled for that plant
        cbSigSim1.Visible = cbSimEn(1).Value;
        cbSigSim2.Visible = cbSimEn(2).Value;
        if ~cbSimEn(pp).Value
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
                if pp == 2
                    lblCfgStat(pp).Text = sprintf('SS [%s%s]  N=%d', obs_str, int_str, c.N);
                else
                    lblCfgStat(pp).Text = sprintf('SS [%s%s]', obs_str, int_str);
                end
            case 'TF'
                if pp == 2
                    lblCfgStat(pp).Text = sprintf('TF  ord=%d  N=%d', c.tf_ord, c.N);
                else
                    lblCfgStat(pp).Text = sprintf('TF  ord=%d', c.tf_ord);
                end
            otherwise
                if pp == 2
                    lblCfgStat(pp).Text = sprintf('%s  N=%d', c.mode, c.N);
                else
                    lblCfgStat(pp).Text = string(c.mode);
                end
        end
    end

    function updateAxesLayout()
        inn = ~strcmp(ddMode(1).Value,'Off');
        out = ~strcmp(ddMode(2).Value,'Off');
        % Full-height for 2-axes case: fill from AX_BOT to ax_y(3)+AX_H
        AX_TOP = ax_y(3) + AX_H;   % top of tallest 4-axis layout
        AH2    = (AX_TOP - AX_BOT - AX_GAP) / 2;  % each tall axis
        if inn && out
            ax_u1.Visible='on';  ax_y1.Visible='on';
            ax_u2.Visible='on';  ax_y2.Visible='on';
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
            % Clear all lines so legends don't persist when axes are shown again
            hAll = [hU1sat,hU1unsat,hR1,hY1,hY1sim,hX1i,hX2i,hU2,hY2,hY2sim,hX1o,hX2o];
            for h = hAll, set(h,'XData',nan,'YData',nan); end
        end
    end

    function updatePlots(~,~)
        hAll = [hU1sat,hU1unsat,hR1,hY1,hY1sim,hX1i,hX2i,hU2,hY2,hY2sim,hX1o,hX2o];
        for h = hAll, set(h,'XData',nan,'YData',nan); end
        if isempty(S.nVec), return; end

        n    = S.nVec;
        su1  = safeS(S.scalers.su1);   su2  = safeS(S.scalers.su2);
        sy1  = safeS(S.scalers.sy1);   sy2  = safeS(S.scalers.sy2);
        sr   = safeS(S.scalers.sr);
        ss1  = safeS(S.scalers.sysim1);ss2  = safeS(S.scalers.sysim2);
        sx1i = safeS(S.scalers.sx1i);  sx2i = safeS(S.scalers.sx2i);
        sx1o = safeS(S.scalers.sx1o);  sx2o = safeS(S.scalers.sx2o);

        inn = ~strcmp(ddMode(1).Value,'Off');
        out = ~strcmp(ddMode(2).Value,'Off');

        if inn
            if cbSigU.Value,     set(hU1sat,  'XData',n,'YData',S.u1Vec*su1);       end
            if cbSigUnsat.Value, set(hU1unsat,'XData',n,'YData',S.u1unsat_Vec*su1); end
            if cbSigR.Value,     set(hR1,     'XData',n,'YData',S.u2Vec*sr);        end
            if cbSigY1.Value,    set(hY1,     'XData',n,'YData',S.y1Vec*sy1);       end
            if cbSigSim1.Value && numel(S.ySimVec1)==numel(n)
                set(hY1sim,'XData',n,'YData',S.ySimVec1*ss1);
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
            if cbSigY2.Value, set(hY2, 'XData',n,'YData',S.y2Vec*sy2);  end
            if cbSigSim2.Value && numel(S.ySimVec2)==numel(n)
                set(hY2sim,'XData',n,'YData',S.ySimVec2*ss2);
            end
            if cbX1o.Value && numel(S.x1oVec)==numel(n)
                set(hX1o,'XData',n,'YData',S.x1oVec*sx1o);
            end
            if cbX2o.Value && numel(S.x2oVec)==numel(n)
                set(hX2o,'XData',n,'YData',S.x2oVec*sx2o);
            end
        end

        for axh = [ax_u1,ax_y1,ax_u2,ax_y2]
            if strcmp(axh.Visible,'on'), axis(axh,'auto'); end
        end
        drawnow;
    end

    function updateRmsLabels()
        for pp2 = 1:2
            try
                if pp2 == 1
                    n_s = S.rms1_n;  ss = S.rms1_sum2;  yv = S.y1Vec;
                else
                    n_s = S.rms2_n;  ss = S.rms2_sum2;  yv = S.y2Vec;
                end
                if n_s > 0
                    rms_e = sqrt(ss / n_s);
                    yv_ok = yv(isfinite(yv));
                    if ~isempty(yv_ok)
                        ymag = sqrt(mean(yv_ok.^2));
                    else
                        ymag = 0;
                    end
                    if ymag > 1e-9
                        pct = rms_e / ymag * 100;
                        lblRMS(pp2).Text = sprintf('sim err RMS: %.4g  (%.1f%%)', rms_e, pct);
                    else
                        lblRMS(pp2).Text = sprintf('sim err RMS: %.4g', rms_e);
                    end
                else
                    lblRMS(pp2).Text = '— sin sim —';
                end
            catch, end
        end
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
            'q_scale',1.0,...
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
        lp.tf_w = zeros(5,1);  lp.xhat = zeros(2,1);  lp.zhat = zeros(2,1);
        lp.vint = 0;  lp.u_prev = 0;  lp.cnt = 0;
    end

%% ═══ ALGORITMOS DE CONTROL (double precision) ════════════════════════════════

    function [y, lp] = tf_step(lp, x)
        b = lp.tf_b;  a = lp.tf_a;  w = lp.tf_w;
        y    =  b(1)*x + w(1);
        w(1) =  w(2) + b(2)*x - a(2)*y;
        w(2) =  w(3) + b(3)*x - a(3)*y;
        w(3) =  w(4) + b(4)*x - a(4)*y;
        w(4) =  w(5) + b(5)*x - a(5)*y;
        w(5) =         b(6)*x - a(6)*y;
        lp.tf_w = w;
    end

    function [u, lp] = ss_pred_step(lp, y, Ts, sat_min, sat_max) %#ok<INUSL>
        % Ts no se usa en el integrador (convención PSoC/Elia: vint += e sin Ts)
        has_i = (lp.mode==3)||(lp.mode==4);
        up = lp.u_prev;
        if has_i, lp.vint = lp.vint + (lp.ref - y); end
        if has_i, extra = lp.Ki*lp.vint;  else, extra = lp.Nbar*lp.ref; end
        u     = extra - lp.K*lp.xhat;
        % Anti-windup back-calculation
        if has_i && lp.Ki ~= 0
            u_sat = max(sat_min, min(sat_max, u));
            lp.vint = lp.vint + (u_sat - u) / lp.Ki;
            u_for_obs = u_sat;
        else
            u_for_obs = u;
        end
        innov = y - (lp.C*lp.xhat + lp.D*up);
        lp.xhat = lp.A*lp.xhat + lp.B*u_for_obs + lp.L*innov;
        lp.u_prev = u_for_obs;
    end

    function [u, lp] = ss_act_step(lp, y, Ts, sat_min, sat_max) %#ok<INUSL>
        % Ts no se usa en el integrador (convención PSoC/Elia: vint += e sin Ts)
        has_i = (lp.mode==3)||(lp.mode==4);
        up = lp.u_prev;
        innov  = y - (lp.C*lp.zhat + lp.D*up);
        lp.xhat = lp.zhat + lp.L*innov;
        if has_i, lp.vint = lp.vint + (lp.ref - y); end
        if has_i, extra = lp.Ki*lp.vint;  else, extra = lp.Nbar*lp.ref; end
        u = extra - lp.K*lp.xhat;
        % Anti-windup back-calculation
        if has_i && lp.Ki ~= 0
            u_sat = max(sat_min, min(sat_max, u));
            lp.vint = lp.vint + (u_sat - u) / lp.Ki;
            u_for_zhat = u_sat;
        else
            u_for_zhat = u;
        end
        lp.zhat = lp.A*lp.xhat + lp.B*u_for_zhat;
        lp.u_prev = u_for_zhat;
    end

    function [u, lp] = loop_step(lp, y, Ts, sat_min, sat_max)
        % SS step functions set lp.u_prev = u_sat (saturated) internally.
        % TF/OL modes don't use u_prev in the observer so no assignment needed.
        switch lp.mode
            case 0,      [u,lp] = tf_step(lp, lp.ref - y);
            case {1,3},  [u,lp] = ss_pred_step(lp, y, Ts, sat_min, sat_max);
            case {2,4},  [u,lp] = ss_act_step(lp, y, Ts, sat_min, sat_max);
            case 5,      u = lp.ref;
            otherwise,   u = 0;
        end
    end

    function [u_sat,u_unsat,x1i,x2i,x1o,x2o,ctrl] = run_control(ctrl, y1, y2)
        inner = ctrl.inner;  outer = ctrl.outer;  Ts = ctrl.Ts;
        smn = ctrl.sat_min;  smx = ctrl.sat_max;
        if outer.mode ~= 6
            outer.cnt = outer.cnt + 1;
            if outer.cnt >= outer.N
                outer.cnt = 0;
                [u2, outer] = loop_step(outer, y2, Ts, smn, smx);
                if inner.mode ~= 6, inner.ref = u2; end
            end
        end
        u_unsat = 0;
        if inner.mode ~= 6
            [u_unsat, inner] = loop_step(inner, y1, Ts, smn, smx);
        elseif outer.mode ~= 6
            u_unsat = outer.u_prev;
        end
        u_sat = max(smn, min(smx, u_unsat));
        x1i = inner.xhat(1);  x2i = inner.xhat(2);
        x1o = outer.xhat(1);  x2o = outer.xhat(2);
        ctrl.inner = inner;  ctrl.outer = outer;
    end

%% ═══ GUI → STRUCT CONTROLADOR ════════════════════════════════════════════════

    function mv = cfg_to_mode_val(c)
        switch c.mode
            case 'TF',        mv = 0;
            case 'SS'
                if     ~strcmp(c.obs,'Actual') && ~c.has_int, mv = 1;
                elseif  strcmp(c.obs,'Actual') && ~c.has_int, mv = 2;
                elseif ~strcmp(c.obs,'Actual') &&  c.has_int, mv = 3;
                else,                                          mv = 4; end
            case 'Open-loop', mv = 5;
            otherwise,        mv = 6;
        end
    end

    function lp = cfg_to_ctrl_loop(c)
        lp = ctrl_loop_default();
        lp.mode = cfg_to_mode_val(c);
        lp.N    = max(1, round(c.N));
        lp.tf_b = c.tf_b(:);  lp.tf_a = c.tf_a(:);
        lp.A    = c.ss_A;     lp.B    = c.ss_B;
        lp.C    = c.ss_C;     lp.D    = c.ss_D;
        lp.L    = c.ss_L;     lp.K    = c.ss_K;
        lp.Ki   = c.ss_Ki;    lp.Nbar = c.ss_Nbar;
    end

    function onApplyCtrl(pp)
        % v6: solo actualiza el struct MATLAB (el estado real vive en PSoC).
        c  = S.cfg(pp);
        lp = cfg_to_ctrl_loop(c);
        if pp == 1, lp.ref = edtRef.Value; else, lp.ref = 0; end
        if pp == 1
            S.ctrl.inner = lp;
        else
            S.ctrl.outer = lp;
        end
        updateCfgStatus(pp);
        logMsg(sprintf('Planta %d: cfg aplicada. modo=%d  N=%d', pp, lp.mode, lp.N));
    end

%% ═══ POPUP — CONTROLADOR ════════════════════════════════════════════════════

    function openCtrlPopup(pp)
        if ~isempty(S.ctrl_popup_fig{pp}) && isvalid(S.ctrl_popup_fig{pp})
            figure(S.ctrl_popup_fig{pp});  return;
        end

        PW = 740;  PH = 380;
        pf = uifigure('Name',sprintf('Controlador — Planta %d', pp),...
             'Position',[200+pp*30 140 PW PH],'Resize','off');
        S.ctrl_popup_fig{pp} = pf;
        pf.DeleteFcn = @(~,~) popCtrlClosed(pp);

        c     = S.cfg(pp);
        has_N = (pp == 2);
        TOP   = PH - 46;

        % ── Top controls ─────────────────────────────────────────────────
        uilabel(pf,'Text','Modo:','Position',[8 TOP 42 22]);
        pd_m = uidropdown(pf,'Items',MODE_NAMES,'Value',c.mode,...
               'Position',[52 TOP 86 22],'ValueChangedFcn',@pRefresh);
        uilabel(pf,'Text','Obs:','Position',[146 TOP 30 22]);
        pd_o = uidropdown(pf,'Items',{'Predictor','Actual'},'Value',c.obs,...
               'Position',[178 TOP 92 22]);
        pc_i = uicheckbox(pf,'Text','Integrador','Value',c.has_int,...
               'Position',[278 TOP 90 22],...
               'ValueChangedFcn',@(~,~) pSyncKiNbar());
        pd_n = [];
        if has_N
            uilabel(pf,'Text','N:','Position',[376 TOP 20 22]);
            pd_n = uieditfield(pf,'numeric','Value',c.N,'Limits',[1 1e6],...
                   'RoundFractionalValues','on','Position',[398 TOP 60 22]);
        end
        Ts_now = 1 / max(edtFs(1).Value, 0.1);
        uilabel(pf,'Text',sprintf('Ts = %.4g s', Ts_now),...
                'Position',[PW-130 TOP 122 22],'FontColor',[0.4 0.4 0.4],...
                'HorizontalAlignment','right');

        % ── Body layout constants ─────────────────────────────────────────
        BTN_Y  = 10;
        BODY_Y = BTN_Y + 38;
        BODY_H = TOP - 10 - BODY_Y;   % interior panel height

        % Expression row geometry (positions relative to panel interior)
        LBL_X = 8;   LBL_W = 90;
        EF_X  = 108; EF_W  = 424;
        BTN_X = 540; BTN_W = 62;
        STA_X = 610; STA_W = 112;

        ROW_H = 28;  ROW_GAP = 8;
        r1 = BODY_H - 50;
        r2 = r1 - ROW_H - ROW_GAP;
        r3 = r2 - ROW_H - ROW_GAP;
        r4 = r3 - ROW_H - ROW_GAP;
        r5 = r4 - ROW_H - ROW_GAP;

        % ── TF panel ─────────────────────────────────────────────────────
        p_TF = uipanel(pf,...
            'Title','Función de transferencia  — ej: tf([1 0.5],[1 -0.8],Ts) o variable workspace',...
            'Position',[5 BODY_Y PW-10 BODY_H]);

        uilabel(p_TF,'Text','Modelo TF:','FontWeight','bold',...
                'Position',[LBL_X r1 LBL_W 22]);
        % Format default TF expression from stored z^-k coefficients → z-descending
        bv = c.tf_b(1:c.tf_ord+1);  av = c.tf_a(1:c.tf_ord+1);
        tf_init = sprintf('tf(%s,%s,%.6g)',...
            mat2str(fliplr(bv(:)'),4), mat2str(fliplr(av(:)'),4), Ts_now);
        et_tf = uieditfield(p_TF,'text','Value',tf_init,...
                'Position',[EF_X r1 EF_W 22]);
        sl_tf = uilabel(p_TF,'Text','—','FontSize',10,...
                'Position',[STA_X r1 STA_W 22]);
        uibutton(p_TF,'Text','✔ Eval','Position',[BTN_X r1 BTN_W 22],...
                 'ButtonPushedFcn',@(~,~) pEvalTF());

        % ── SS panel ─────────────────────────────────────────────────────
        p_SS = uipanel(pf,...
            'Title','Estado-Espacio  — ej: ss(A,B,C,D,Ts) o variable workspace',...
            'Position',[5 BODY_Y PW-10 BODY_H]);

        % Row 1 — SS model
        uilabel(p_SS,'Text','Modelo SS:','FontWeight','bold',...
                'Position',[LBL_X r1 LBL_W 22]);
        A_s  = mat2str(c.ss_A,5);  B_s  = mat2str(c.ss_B,5);
        Cs_s = mat2str(c.ss_C,5);  D_s  = num2str(c.ss_D,5);
        ss_init = sprintf('ss(%s,%s,%s,%s,%.6g)', A_s, B_s, Cs_s, D_s, Ts_now);
        et_ss = uieditfield(p_SS,'text','Value',ss_init,...
                'Position',[EF_X r1 EF_W 22]);
        sl_ss = uilabel(p_SS,'Text','—','FontSize',10,...
                'Position',[STA_X r1 STA_W 22]);
        uibutton(p_SS,'Text','✔ Eval','Position',[BTN_X r1 BTN_W 22],...
                 'ButtonPushedFcn',@(~,~) pEvalSS());

        % Row 2 — K (1×2)
        uilabel(p_SS,'Text','K (1×2):','FontWeight','bold',...
                'Position',[LBL_X r2 LBL_W 22]);
        et_K  = uieditfield(p_SS,'text','Value',mat2str(c.ss_K,5),...
                'Position',[EF_X r2 EF_W 22]);
        sl_K  = uilabel(p_SS,'Text','—','FontSize',10,...
                'Position',[STA_X r2 STA_W 22]);
        uibutton(p_SS,'Text','✔ Eval','Position',[BTN_X r2 BTN_W 22],...
                 'ButtonPushedFcn',@(~,~) pEvalVec(et_K,sl_K,[1 2]));

        % Row 3 — L (2×1)
        uilabel(p_SS,'Text','L (2×1):','FontWeight','bold',...
                'Position',[LBL_X r3 LBL_W 22]);
        et_L  = uieditfield(p_SS,'text','Value',mat2str(c.ss_L,5),...
                'Position',[EF_X r3 EF_W 22]);
        sl_L  = uilabel(p_SS,'Text','—','FontSize',10,...
                'Position',[STA_X r3 STA_W 22]);
        uibutton(p_SS,'Text','✔ Eval','Position',[BTN_X r3 BTN_W 22],...
                 'ButtonPushedFcn',@(~,~) pEvalVec(et_L,sl_L,[2 1]));

        % Row 4a — Ki (visible when has_int = true)
        lbl_Ki = uilabel(p_SS,'Text','Ki:','FontWeight','bold',...
                 'Position',[LBL_X r4 LBL_W 22]);
        et_Ki  = uieditfield(p_SS,'text','Value',num2str(c.ss_Ki,5),...
                 'Position',[EF_X r4 EF_W 22]);
        sl_Ki  = uilabel(p_SS,'Text','—','FontSize',10,...
                 'Position',[STA_X r4 STA_W 22]);
        btn_Ki = uibutton(p_SS,'Text','✔ Eval','Position',[BTN_X r4 BTN_W 22],...
                 'ButtonPushedFcn',@(~,~) pEvalVec(et_Ki,sl_Ki,[1 1]));

        % Row 4b — Nbar (visible when has_int = false)
        lbl_Nb = uilabel(p_SS,'Text','Nbar:','FontWeight','bold',...
                 'Position',[LBL_X r4 LBL_W 22]);
        et_Nb  = uieditfield(p_SS,'text','Value',num2str(c.ss_Nbar,5),...
                 'Position',[EF_X r4 EF_W 22]);
        sl_Nb  = uilabel(p_SS,'Text','—','FontSize',10,...
                 'Position',[STA_X r4 STA_W 22]);
        btn_Nb = uibutton(p_SS,'Text','✔ Eval','Position',[BTN_X r4 BTN_W 22],...
                 'ButtonPushedFcn',@(~,~) pEvalVec(et_Nb,sl_Nb,[1 1]));

        % Row 5 — q_scale (normalización para paths Q31/Q15/Q7)
        uilabel(p_SS,'Text','q_scale (Q):','FontWeight','bold',...
                'Position',[LBL_X r5 LBL_W 22]);
        qs_init = c.q_scale;  if isempty(qs_init) || qs_init <= 0, qs_init = 1.0; end
        et_qs = uieditfield(p_SS,'numeric','Value',qs_init,'Limits',[1e-9 1e12],...
                'Position',[EF_X r5 120 22],'Tooltip',...
                'Normalización señales para Q31/Q15/Q7. Señales físicas ÷ q_scale ∈ [-1,1). Default: 1.0');
        uilabel(p_SS,'Text','señales ÷ q_scale ∈ [-1,1) para Q-format',...
                'FontSize',10,'FontColor',[0.4 0.4 0.4],...
                'Position',[EF_X+128 r5 STA_W+100 22]);

        % ── Bottom buttons ────────────────────────────────────────────────
        uibutton(pf,'Text','✔  Aplicar y cerrar',...
            'Position',[PW-296 BTN_Y 188 28],...
            'BackgroundColor',[0.12 0.62 0.12],'FontColor','w',...
            'ButtonPushedFcn',@pDoApply);
        uibutton(pf,'Text','✖  Cerrar',...
            'Position',[PW-100 BTN_Y 92 28],...
            'ButtonPushedFcn',@(~,~) delete(pf));

        pRefresh();
        pSyncKiNbar();

        % ════════════════════════════════════════════════════════════════
        %  Helpers
        % ════════════════════════════════════════════════════════════════

        function val = ws_eval(expr)
            expr = strtrim(expr);
            try,  val = evalin('base', expr);
            catch, val = eval(expr); end %#ok<EVLCS>
        end

        function setStat(sl, msg, ok)
            sl.Text = msg(1:min(numel(msg),52));
            if ok, sl.FontColor = [0.05 0.50 0.05];
            else,  sl.FontColor = [0.80 0.10 0.10];  end
        end

        function pEvalTF()
            try
                v = ws_eval(et_tf.Value);
                if isa(v,'zpk'), v = tf(v); end
                if ~isa(v,'tf'), error('Se esperaba tf/zpk'); end
                [~,dv] = tfdata(v,'v');
                ord = length(dv) - 1;
                if ord > 5, error('Orden %d > 5', ord); end
                setStat(sl_tf, sprintf('✓ tf  ord=%d  Ts=%.4g', ord, v.Ts), true);
            catch e
                setStat(sl_tf, ['✗ ' e.message], false);
            end
        end

        function pEvalSS()
            try
                [Av,~,~,~,Tsv] = pExtractSS(ws_eval(et_ss.Value));
                ns = size(Av,1);
                if ns ~= 2, error('PSoC: 2 estados requeridos, SS tiene %d', ns); end
                setStat(sl_ss, sprintf('✓ ss  2 estados  Ts=%.4g', Tsv), true);
            catch e
                setStat(sl_ss, ['✗ ' e.message], false);
            end
        end

        function pEvalVec(ef, sl, expected_sz)
            try
                v = double(ws_eval(ef.Value));
                if ~isnumeric(v), error('Se esperaba numérico'); end
                if ~isequal(size(v), expected_sz)
                    error('Tamaño %s, esperado %s',...
                        mat2str(size(v)), mat2str(expected_sz));
                end
                setStat(sl, sprintf('✓ %s', mat2str(v(:)',4)), true);
            catch e
                setStat(sl, ['✗ ' e.message], false);
            end
        end

        function [Av,Bv,Cv,Dv,Tsv] = pExtractSS(v)
            Ts_cur = 1 / max(edtFs(1).Value, 0.1);
            if isa(v,'ss')
                Av = v.A;  Bv = v.B;  Cv = v.C;  Dv = double(v.D(1,1));
                Tsv = v.Ts;
                if Tsv == 0
                    [Av,Bv] = c2d_zoh(Av, Bv, Ts_cur);
                    Tsv = Ts_cur;
                end
            elseif iscell(v) && numel(v) >= 4
                Av  = double(v{1});  Bv = double(v{2});
                Cv  = double(v{3});  Dv = double(v{4}(1,1));
                Tsv = Ts_cur;
            else
                error('SS: pase ss(A,B,C,D,Ts) o {A,B,C,D}');
            end
        end

        function pRefresh(~,~)
            mn = pd_m.Value;
            p_TF.Visible = strcmp(mn,'TF');
            p_SS.Visible = strcmp(mn,'SS');
            pd_o.Enable  = strcmp(mn,'SS');
            pc_i.Enable  = strcmp(mn,'SS');
        end

        function pSyncKiNbar()
            has_i = pc_i.Value;
            lbl_Ki.Visible = has_i;  et_Ki.Visible = has_i;
            sl_Ki.Visible  = has_i;  btn_Ki.Visible = has_i;
            lbl_Nb.Visible = ~has_i; et_Nb.Visible = ~has_i;
            sl_Nb.Visible  = ~has_i; btn_Nb.Visible = ~has_i;
        end

        function pDoApply(~,~)
            try
                c2 = S.cfg(pp);
                c2.mode    = pd_m.Value;
                c2.obs     = pd_o.Value;
                c2.has_int = pc_i.Value;
                if has_N && ~isempty(pd_n)
                    c2.N = max(1, round(pd_n.Value));
                else
                    c2.N = 1;
                end

                if strcmp(c2.mode, 'TF')
                    % ── Extraer TF ───────────────────────────────────────
                    v = ws_eval(et_tf.Value);
                    if isa(v,'zpk'), v = tf(v); end
                    if ~isa(v,'tf'), error('Expresión TF: se esperaba tf/zpk'); end
                    Ts_cur = 1 / max(edtFs(1).Value, 0.1);
                    if v.Ts == 0
                        [Ac,Bc,Cc,Dc] = ssdata(v);
                        [Ad,Bd] = c2d_zoh(Ac, Bc, Ts_cur);
                        v = tf(ss(Ad,Bd,Cc,Dc,Ts_cur));
                    end
                    [num_v, den_v] = tfdata(v,'v');
                    d0 = den_v(1);
                    num_v = num_v / d0;
                    den_v = den_v / d0;
                    N_ord = length(den_v) - 1;
                    if N_ord > 5, error('Orden %d > 5 (máx PSoC)', N_ord); end
                    M_ord = length(num_v) - 1;
                    % Convert MATLAB z-descending → PSoC z^-k ascending
                    shift = N_ord - M_ord;
                    b_p = zeros(6,1);  a_p = zeros(6,1);
                    b_p(shift+1 : shift+length(num_v)) = num_v(:);
                    a_p(1:length(den_v))               = den_v(:);
                    c2.tf_b   = b_p;
                    c2.tf_a   = a_p;
                    c2.tf_ord = N_ord;

                elseif strcmp(c2.mode, 'SS')
                    % ── Extraer SS ───────────────────────────────────────
                    [Av,Bv,Cv,Dv,~] = pExtractSS(ws_eval(et_ss.Value));
                    ns = size(Av,1);
                    if ns ~= 2,          error('PSoC requiere 2 estados'); end
                    if ~isequal(size(Bv),[2 1]), error('B debe ser 2×1'); end
                    if ~isequal(size(Cv),[1 2]), error('C debe ser 1×2'); end
                    c2.ss_A = Av;  c2.ss_B = Bv;  c2.ss_C = Cv;  c2.ss_D = Dv;

                    Kv = double(ws_eval(et_K.Value));
                    if ~isequal(size(Kv),[1 2]), error('K debe ser 1×2'); end
                    c2.ss_K = Kv;

                    Lv = double(ws_eval(et_L.Value));
                    if ~isequal(size(Lv),[2 1]), error('L debe ser 2×1'); end
                    c2.ss_L = Lv;

                    if c2.has_int
                        Kiv = double(ws_eval(et_Ki.Value));
                        if ~isscalar(Kiv), error('Ki debe ser escalar'); end
                        c2.ss_Ki = Kiv;
                    else
                        Nbv = double(ws_eval(et_Nb.Value));
                        if ~isscalar(Nbv), error('Nbar debe ser escalar'); end
                        c2.ss_Nbar = Nbv;
                    end
                end
                % Open-loop / Off: solo N importa (ya seteado arriba)

                % Leer q_scale (siempre, aplica en modo SS para paths Q)
                c2.q_scale = max(1e-9, et_qs.Value);

                S.cfg(pp) = c2;
                ddMode(pp).Value = c2.mode;
                ddObs(pp).Value  = c2.obs;
                cbInt(pp).Value  = c2.has_int;
                if pp == 2, edtN(pp).Value = c2.N; end

                onApplyCtrl(pp);
                delete(pf);
            catch e
                uialert(pf, string(e.message), 'Error — controlador');
            end
        end

    end % openCtrlPopup

    function popCtrlClosed(pp)
        S.ctrl_popup_fig{pp} = [];
    end

%% ═══ POPUP — MODELO SIMULACIÓN ═══════════════════════════════════════════════

    function openSimPopup(pp)
        if ~isempty(S.sim_popup_fig{pp}) && isvalid(S.sim_popup_fig{pp})
            figure(S.sim_popup_fig{pp});  return;
        end

        SW = 680;  SH = 280;
        sf = uifigure('Name',sprintf('Modelo Simulación — Planta %d', pp),...
             'Position',[350+pp*20 200 SW SH],'Resize','off');
        S.sim_popup_fig{pp} = sf;
        sf.DeleteFcn = @(~,~) simPopupClosed(pp);

        % ── Top controls ───────────────────────────────────────────────
        TOP = SH - 44;
        uilabel(sf,'Text','Tipo:','Position',[8 TOP 36 22]);
        sd_type = uidropdown(sf,'Items',{'TF','SS'},'Value','TF',...
                  'Position',[46 TOP 72 22],'ValueChangedFcn',@sRefresh);

        Ts_cur = 1 / max(edtFs(1).Value, 0.1);
        uilabel(sf,'Text',sprintf('Ts = %.4g s  (desde Fs)',Ts_cur),...
                'Position',[SW-200 TOP 192 22],'FontColor',[0.4 0.4 0.4],...
                'HorizontalAlignment','right');

        % ── Body panels ────────────────────────────────────────────────
        BTN_Y = 10;
        BODY_Y = BTN_Y + 38;
        BODY_H = TOP - 8 - BODY_Y;

        LBL_X = 8;  LBL_W = 90;  EF_X = 108;  EF_W = 394;
        BTN_X = 510; BTN_W = 60;  STA_X = 578;  STA_W = SW-14-STA_X;
        r1 = BODY_H - 44;  r2 = r1 - 36;

        % ── TF panel ───────────────────────────────────────────────────
        sp_TF = uipanel(sf,...
            'Title','Función de transferencia  — ej: tf([1],[1 -0.5],Ts) o variable workspace',...
            'Position',[5 BODY_Y SW-10 BODY_H]);
        uilabel(sp_TF,'Text','Modelo TF:','FontWeight','bold','Position',[LBL_X r1 LBL_W 22]);
        s_tf = uieditfield(sp_TF,'text','Value','tf([1],[1 -1],Ts)',...
               'Position',[EF_X r1 EF_W 22]);
        sl_tf = uilabel(sp_TF,'Text','—','FontSize',10,'Position',[STA_X r1 STA_W 22]);
        uibutton(sp_TF,'Text','✔ Eval','Position',[BTN_X r1 BTN_W 22],...
                 'ButtonPushedFcn',@(~,~) sDemoEval(s_tf,sl_tf,'tf'));

        % ── SS panel ───────────────────────────────────────────────────
        sp_SS = uipanel(sf,...
            'Title','Estado-Espacio  — ej: ss(A,B,C,D,Ts) o variable workspace',...
            'Position',[5 BODY_Y SW-10 BODY_H]);
        uilabel(sp_SS,'Text','Modelo SS:','FontWeight','bold','Position',[LBL_X r1 LBL_W 22]);
        s_ss_ef = uieditfield(sp_SS,'text','Value','ss(A,B,C,D,Ts)',...
                  'Position',[EF_X r1 EF_W 22]);
        sl_ss = uilabel(sp_SS,'Text','—','FontSize',10,'Position',[STA_X r1 STA_W 22]);
        uibutton(sp_SS,'Text','✔ Eval','Position',[BTN_X r1 BTN_W 22],...
                 'ButtonPushedFcn',@(~,~) sDemoEval(s_ss_ef,sl_ss,'ss'));

        % ── Bottom ─────────────────────────────────────────────────────
        uilabel(sf,'Text','ŷ (cyan punteado) se compara con y medida.',...
            'Position',[8 BTN_Y+5 280 18],'FontSize',9,'FontColor',[0.4 0.4 0.4]);
        uibutton(sf,'Text','✔  Aplicar','Position',[SW-226 BTN_Y 108 28],...
            'BackgroundColor',[0.12 0.62 0.12],'FontColor','w',...
            'ButtonPushedFcn',@sDoApply);
        uibutton(sf,'Text','✖  Cerrar','Position',[SW-112 BTN_Y 104 28],...
            'ButtonPushedFcn',@(~,~) delete(sf));

        sRefresh();

        % ── Inner helpers ───────────────────────────────────────────────
        function val = sWsEval(expr)
            expr = strtrim(expr);
            try,  val = evalin('base', expr);
            catch, val = eval(expr); end %#ok<EVLCS>
        end

        function setStat(sl, msg, ok)
            sl.Text = msg(1:min(numel(msg),60));
            if ok, sl.FontColor = [0.05 0.50 0.05];
            else,  sl.FontColor = [0.80 0.10 0.10];  end
        end

        function sDemoEval(ef, sl, typ)
            try
                v = sWsEval(ef.Value);
                if strcmp(typ,'tf')
                    if isa(v,'zpk'), v = tf(v); end
                    if ~isa(v,'tf'), error('Se esperaba tf/zpk'); end
                    [~,dv] = tfdata(v,'v');
                    setStat(sl, sprintf('✓ tf  ord=%d  Ts=%.4g', length(dv)-1, v.Ts), true);
                else
                    if ~isa(v,'ss'), error('Se esperaba ss'); end
                    setStat(sl, sprintf('✓ ss  %d estados  Ts=%.4g', size(v.A,1), v.Ts), true);
                end
            catch e
                setStat(sl, ['✗ ' e.message], false);
            end
        end

        function [Ad,Bd,Cd,Dd] = sExtract(expr)
            Ts_s = 1 / max(edtFs(1).Value, 0.1);
            v = sWsEval(expr);
            if isa(v,'tf') || isa(v,'zpk')
                v = tf(v);
                if v.Ts == 0
                    [Ac,Bc,Cc,Dc] = ssdata(v);
                    [Ad,Bd] = c2d_zoh(Ac,Bc,Ts_s);
                    Cd = Cc;  Dd = Dc;
                else
                    [Ac,Bc,Cc,Dc] = ssdata(v);
                    Ad = Ac;  Bd = Bc;  Cd = Cc;  Dd = Dc;
                end
            elseif isa(v,'ss')
                if v.Ts == 0
                    [Av,Bv] = c2d_zoh(v.A,v.B,Ts_s);
                    Ad = Av;  Bd = Bv;  Cd = v.C;  Dd = v.D;
                else
                    Ad = v.A;  Bd = v.B;  Cd = v.C;  Dd = v.D;
                end
            else
                error('Se esperaba tf, zpk o ss');
            end
        end

        function sRefresh(~,~)
            sp_TF.Visible = strcmp(sd_type.Value,'TF');
            sp_SS.Visible = strcmp(sd_type.Value,'SS');
        end

        function sDoApply(~,~)
            try
                if strcmp(sd_type.Value,'TF')
                    [Ad,Bd,Cd,Dd] = sExtract(s_tf.Value);
                    type_str = 'TF';
                else
                    [Ad,Bd,Cd,Dd] = sExtract(s_ss_ef.Value);
                    type_str = 'SS';
                end
                ns = size(Ad,1);
                if size(Bd,1)~=ns||size(Cd,2)~=ns
                    error('Dimensiones inconsistentes'); end
                if ~isequal(size(Cd),[1 ns])
                    error('C debe ser fila 1×n'); end
                S.cfg(pp).sim_Ad = Ad;  S.cfg(pp).sim_Bd = Bd;
                S.cfg(pp).sim_Cd = Cd;  S.cfg(pp).sim_Dd = double(Dd(1,1));
                S.cfg(pp).sim_x  = zeros(ns,1);
                S.cfg(pp).sim_enabled = true;
                cbSimEn(pp).Value = true;
                onSimToggle(pp);   % update ŷ checkbox visibility
                logMsg(sprintf('Planta %d: modelo sim %s cargado. %d estados.',...
                    pp, type_str, ns));
                delete(sf);
            catch e
                uialert(sf, string(e.message), 'Error — modelo simulación');
            end
        end

    end % openSimPopup

    function simPopupClosed(pp)
        S.sim_popup_fig{pp} = [];
    end

%% ═══ POPUP — ESCALADORES ════════════════════════════════════════════════════

    function openScalerPopup()
        if ~isempty(S.scaler_popup_fig) && isvalid(S.scaler_popup_fig)
            figure(S.scaler_popup_fig);  return;
        end
        SW = 310;
        ROW_H   = 28;  ROW_GAP = 4;  ROW_STEP = ROW_H + ROW_GAP;
        BTN_H   = 28;  BTN_BOT = 10;
        TOP_PAD = 52;  % space above first row
        slabels = {'u₁×','u₂×','y₁×','y₂×','ŷ sim₁×','ŷ sim₂×','R×',...
                   'x̂₁ᵢ×','x̂₂ᵢ×','x̂₁ₒ×','x̂₂ₒ×'};
        sfields = {'su1','su2','sy1','sy2','sysim1','sysim2','sr',...
                   'sx1i','sx2i','sx1o','sx2o'};
        NS = numel(sfields);
        SH = BTN_BOT + BTN_H + ROW_GAP*2 + NS*ROW_STEP + TOP_PAD;
        sf = uifigure('Name','Escaladores de visualización',...
             'Position',[500 200 SW SH],'Resize','off');
        S.scaler_popup_fig = sf;
        sf.DeleteFcn = @(~,~) scalerPopupClosed();

        ef_scl = cell(NS,1);
        for fi = 1:NS
            uilabel(sf,'Text',slabels{fi},...
                'Position',[12 SH-TOP_PAD-(fi-1)*ROW_STEP-ROW_H 82 ROW_H]);
            ef_scl{fi} = uieditfield(sf,'numeric',...
                'Value', S.scalers.(sfields{fi}),'Limits',[-1e9 1e9],...
                'Position',[98 SH-TOP_PAD-(fi-1)*ROW_STEP-ROW_H 116 ROW_H]);
        end

        uibutton(sf,'Text','✔  Aplicar','Position',[SW-232 BTN_BOT 110 BTN_H],...
            'BackgroundColor',[0.12 0.62 0.12],'FontColor','w',...
            'ButtonPushedFcn',@sclApply);
        uibutton(sf,'Text','✖  Cerrar','Position',[SW-116 BTN_BOT 108 BTN_H],...
            'ButtonPushedFcn',@(~,~) delete(sf));

        function sclApply(~,~)
            for fi2 = 1:NS
                S.scalers.(sfields{fi2}) = ef_scl{fi2}.Value;
            end
            updatePlots();
            delete(sf);
        end

    end % openScalerPopup

    function scalerPopupClosed()
        S.scaler_popup_fig = [];
    end

%% ═══ UART BAJO NIVEL ═════════════════════════════════════════════════════════

    function ll_flush()
        if ~isempty(S.sp), try flush(S.sp); catch, end; end
    end

    function b = ll_readexact(n, tmo)
        if nargin < 2, tmo = STEP_TMO; end
        t0 = tic;  buf = zeros(1,n,'uint8');  k = 0;
        while k < n
            if toc(t0) > tmo, error('Timeout (%d/%d B)',k,n); end
            av = S.sp.NumBytesAvailable;
            if av > 0
                m = min(av,n-k);
                tmp = read(S.sp,m,"uint8");
                buf(k+1:k+m) = tmp(:)';  k = k+m;
            else
                pause(0.001);
            end
        end
        b = buf(:);
    end

    function b = ll_readexact_fast(n)
        % Versión sin pause para el loop de control (tight poll)
        t0 = tic;  buf = zeros(1,n,'uint8');  k = 0;
        while k < n
            if toc(t0) > 0.5, error('Timeout ctrl (%d/%d B)',k,n); end
            av = S.sp.NumBytesAvailable;
            if av > 0
                m = min(av,n-k);
                tmp = read(S.sp,m,"uint8");
                buf(k+1:k+m) = tmp(:)';  k = k+m;
            end
        end
        b = buf(:);
    end

    function ll_send_payload(data_u8)
        data_u8 = uint8(data_u8(:));  n = numel(data_u8);  idx = 1;
        while idx <= n
            w = uint8([0;0;0;0]);  take = min(4,n-idx+1);
            w(1:take) = data_u8(idx:idx+take-1);
            tries = 0;
            while true
                tries = tries+1;
                if tries > MAX_RETRIES, error('Max retries idx=%d',idx); end
                write(S.sp,w,"uint8");
                echo = ll_readexact(4);
                match = isequal(echo(:),w(:));
                if match, write(S.sp,uint8('A'),"uint8");
                else,     write(S.sp,uint8('N'),"uint8"); end
                conf = ll_readexact(1);
                if match && conf(1)==uint8('A'), break; end
            end
            idx = idx + take;
        end
    end

%% ═══ PROTOCOLO UART ══════════════════════════════════════════════════════════

    function uartp_reset()
        % FIX: enviar 's' primero para salir de cualquier STREAM/CONTROL mode.
        % Tanto COMMAND como STREAM mode responden 's' con 'K'.
        % Sin esto, si PSoC quedó en STREAM mode, ignora 'r' → timeout.
        try, write(S.sp, uint8('s'), "uint8"); catch, end
        pause(0.15);   % PSoC procesa 's' y envía 'K'
        ll_flush();    % descartar 'K' de 's' + tramas de telemetría pendientes
        rsp = ll_cmd_wait('r');
        if rsp ~= uint8('K'), error("reset: rsp=%c",char(rsp)); end
    end

    function uartp_set_fs(fs_hz)
        ll_flush();  rsp = ll_cmd_wait('f');
        if rsp ~= uint8('R'), error("set_fs: rsp=%c",char(rsp)); end
        ll_send_payload(typecast(single(fs_hz),'uint8'));
        fin = ll_readexact(1);
        if fin(1) ~= uint8('K'), error("set_fs fin=%c",char(fin(1))); end
    end

    function uartp_start_run()
        ll_flush();  rsp = ll_cmd_wait('i');
        if rsp ~= uint8('K'), error("start: rsp=%c",char(rsp)); end
    end

    function uartp_stop()
        if isempty(S.sp), return; end
        try, write(S.sp, uint8('s'), "uint8"); catch, return; end
        t0 = tic;
        while toc(t0) < 1.5
            if isempty(S.sp), return; end
            if S.sp.NumBytesAvailable > 0
                b = read(S.sp,1,"uint8");
                if b(1) == uint8('K'), return; end
            end
        end
    end

    function rsp = ll_cmd_wait(cmd, tmo)
        if nargin < 2, tmo = STEP_TMO; end
        write(S.sp, uint8(cmd), "uint8");
        t0 = tic;
        while true
            if toc(t0) > tmo, error("Timeout cmd '%s'",cmd); end
            if S.sp.NumBytesAvailable > 0
                b = read(S.sp,1,"uint8");  b = b(1);
                if any(b == uint8(['R','K','!'])), rsp = b;  return; end
            else
                pause(0.001);
            end
        end
    end

    function send_u(u_int16)
        write(S.sp, typecast(int16(u_int16),'uint8'), "uint8");
    end

%% ═══ ACCIONES ════════════════════════════════════════════════════════════════

    function onConnToggle(~,~)
        if ~S.isConnected
            com = strtrim(string(edtCom.Value));
            if com == "", logMsg("Puerto vacío."); return; end
            try
                S.sp = serialport(com, edtBaud.Value);  S.sp.Timeout = 0.1;
                flush(S.sp);  pause(0.05);  ll_flush();
                S.isConnected = true;
                btnConn.Text = 'Desconectar';
                btnConn.BackgroundColor = [0.65 0.15 0.15];
                lblStat.Text = "CONECTADO: " + com;
                logMsg("Conectado: " + com + " @ " + edtBaud.Value);
            catch e
                logMsg("Error: " + string(e.message));
                S.sp = [];  S.isConnected = false;
            end
        else
            S.streamOn = false;   % señal para que runControlLoop salga
            try
                if ~isempty(S.sp) && isvalid(S.sp), flush(S.sp); delete(S.sp); end
            catch, end
            S.sp = [];  S.isConnected = false;
            btnConn.Text = 'Conectar';
            btnConn.BackgroundColor = [0.2 0.62 0.2];
            lblStat.Text = 'Desconectado';
            lblRunSt.Text = 'Detenido';  lblRunSt.FontColor = [0 0 0];
            btnStart.Text = '▶  Start';
            btnStart.BackgroundColor = [0.12 0.62 0.12];
            btnStart.FontColor = 'w';
            logMsg("Desconectado.");
        end
    end

    function onSaveSession(~,~)
        [f,p] = uiputfile('*.mat','Guardar sesión');
        if isequal(f,0), return; end
        try
            sess.cfg1     = S.cfg(1);  sess.cfg1.sim_x = [];
            sess.cfg2     = S.cfg(2);  sess.cfg2.sim_x = [];
            sess.Fs       = edtFs(1).Value;
            sess.SatMin   = edtSatMin.Value;
            sess.SatMax   = edtSatMax.Value;
            sess.num_type = ddNumType.Value;
            sess.scalers  = S.scalers;
            save(fullfile(p,f),'-struct','sess');
            logMsg("Sesión guardada: " + string(f));
        catch e, logMsg("Guardar FAIL: " + string(e.message)); end
    end

    function onLoadSession(~,~)
        [f,p] = uigetfile('*.mat','Cargar sesión');
        if isequal(f,0), return; end
        try
            sess = load(fullfile(p,f));
            if isfield(sess,'cfg1')
                S.cfg(1) = sess.cfg1;  S.cfg(1).sim_x = [];
                if ~isfield(S.cfg(1),'q_scale') || S.cfg(1).q_scale <= 0
                    S.cfg(1).q_scale = 1.0;  % retrocompat con sesiones antiguas
                end
                ddMode(1).Value  = S.cfg(1).mode;
                ddObs(1).Value   = S.cfg(1).obs;
                cbInt(1).Value   = S.cfg(1).has_int;
                edtFs(1).Value   = S.cfg(1).Fs;
                cbSimEn(1).Value = S.cfg(1).sim_enabled;
            end
            if isfield(sess,'cfg2')
                S.cfg(2) = sess.cfg2;  S.cfg(2).sim_x = [];
                if ~isfield(S.cfg(2),'q_scale') || S.cfg(2).q_scale <= 0
                    S.cfg(2).q_scale = 1.0;
                end
                ddMode(2).Value  = S.cfg(2).mode;
                ddObs(2).Value   = S.cfg(2).obs;
                cbInt(2).Value   = S.cfg(2).has_int;
                edtN(2).Value    = S.cfg(2).N;
                cbSimEn(2).Value = S.cfg(2).sim_enabled;
            end
            if isfield(sess,'Fs'),       edtFs(1).Value   = sess.Fs; end
            if isfield(sess,'SatMin'),   edtSatMin.Value  = sess.SatMin; end
            if isfield(sess,'SatMax'),   edtSatMax.Value  = sess.SatMax; end
            if isfield(sess,'num_type'), ddNumType.Value  = sess.num_type; end
            if isfield(sess,'scalers')
                S.scalers = sess.scalers;
            elseif isfield(sess,'SU1')
                % compatibilidad con sesiones v4
                S.scalers.su1 = sess.SU1;  S.scalers.su2 = sess.SU2;
                S.scalers.sy1 = sess.SY1;  S.scalers.sy2 = sess.SY2;
            end
            for pp2 = 1:2, onModeChange(pp2);  updateCfgStatus(pp2); end
            for pp2 = 1:2, onSimToggle(pp2); end   % sync ŷ checkbox visibility
            logMsg("Sesión cargada: " + string(f));
        catch e, logMsg("Cargar FAIL: " + string(e.message)); end
    end

    function onStart(~,~)
        if ~reqConn(), return; end
        if S.inLoop
            % ── Ya corriendo: botón actúa como "Send ref" ─────────────────
            new_ref = edtRef.Value;
            try
                uartp_update_ref(new_ref);
                S.ctrl.inner.ref = new_ref;
                logMsg(sprintf("Ref → %.4g (live)", new_ref));
            catch e
                logMsg("Update ref ERR: " + string(e.message));
            end
            return;
        end
        % ── Inicio completo ───────────────────────────────────────────────
        for pp2 = 1:2, onApplyCtrl(pp2); end
        S.ctrl.Ts      = 1 / edtFs(1).Value;
        S.ctrl.sat_min = edtSatMin.Value;
        S.ctrl.sat_max = edtSatMax.Value;
        ref0 = edtRef.Value;
        S.ctrl.inner.ref = ref0;
        S.ctrl.outer.ref = 0;
        for pp2 = 1:2
            if ~isempty(S.cfg(pp2).sim_Ad)
                S.cfg(pp2).sim_x = zeros(size(S.cfg(pp2).sim_Ad,1),1);
            end
        end
        S.rms1_sum2 = 0;  S.rms1_n = 0;
        S.rms2_sum2 = 0;  S.rms2_n = 0;
        updateAxesLayout();
        try
            ll_flush();
            uartp_reset();
            % uartp_send_params(force=false): compara con last_sent_payload.
            % Si params no cambiaron, no reenvía. Si cambiaron, envía y verifica eco.
            uartp_send_params();
            uartp_start_run();
            S.streamOn = true;
            S.autoStopArmed = S.autoStopEn && S.autoStopN > 0;
            lblRunSt.Text = '▶ CORRIENDO';  lblRunSt.FontColor = [0 0.5 0];
            % Cambiar botón Start → "📤 Send ref" (azul) mientras corre
            btnStart.Text = '📤 Send ref';
            btnStart.BackgroundColor = [0.15 0.35 0.75];
            btnStart.FontColor = 'w';
            logMsg(sprintf("✔ Iniciado. Fs=%.1fHz  ref=%.4g  sat=[%.0f,%.0f]",...
                edtFs(1).Value, ref0, S.ctrl.sat_min, S.ctrl.sat_max));
        catch e
            logMsg("Start FAIL: " + string(e.message));
            S.streamOn = false;
            return;
        end
        runStreamLoop();   % loop no-bloqueante hasta S.streamOn = false
    end

    function onStop(~,~)
        S.streamOn = false;   % runControlLoop lo detecta y sale
    end

    function onAutoStopToggle(~,~)
        S.autoStopEn = cbAutoStop.Value;
        S.autoStopN  = edtAutoN.Value;
    end

%% ═══ LOOP DE RECEPCIÓN (v6 — PSoC controla, MATLAB recibe) ══════════════════

    function runStreamLoop()
        % En v6 el PSoC es master de timing: envía 32B cada Ts (timer ISR).
        % MATLAB solo acumula bytes y procesa tramas completas. Sin rate-limit.
        S.inLoop      = true;
        rxBuf         = uint8(zeros(0,1));
        t_last_draw   = tic;
        DRAW_INTERVAL = 0.05;   % 50 ms entre actualizaciones de pantalla
        t_last_frame  = tic;
        last_inter_ms = 0;

        while S.streamOn && S.isConnected && ~isempty(S.sp)
            try
                % 1. Leer todos los bytes disponibles (non-blocking)
                av = S.sp.NumBytesAvailable;
                if av > 0
                    newB = read(S.sp, av, "uint8");
                    rxBuf = [rxBuf; newB(:)];
                end

                % 2. Procesar tramas completas de 32 bytes (8×float32)
                while numel(rxBuf) >= TELEM_SZ
                    processTelemFrame(rxBuf(1:TELEM_SZ));
                    rxBuf = rxBuf(TELEM_SZ+1:end);
                    last_inter_ms = toc(t_last_frame)*1000;
                    t_last_frame  = tic;
                end
            catch e
                logMsg("Stream ERR: " + string(e.message));
                S.streamOn = false;
                break;
            end

            % 3. Actualizar pantalla a ~20 fps
            if toc(t_last_draw) >= DRAW_INTERVAL
                t_last_draw = tic;
                updatePlots();
                updateRmsLabels();
                updateDataInfo();
                try
                    lblRTT.Text = sprintf('Ts_meas=%.2fms  n=%d',...
                        last_inter_ms, S.framesTotal);
                catch, end
                drawnow;
            end

            % 4. Auto-stop
            if S.autoStopArmed && S.framesTotal >= S.autoStopN
                S.autoStopArmed = false;
                S.streamOn = false;
                logMsg(sprintf("Auto-stop: %d frames.", S.framesTotal));
            end

            % 5. Ceder CPU brevemente (GUI responsiva, sin busy-wait)
            if av == 0
                pause(0.001);
            end
        end

        % ── Salida limpia ─────────────────────────────────────────────────────
        try
            S.streamOn = false;
            uartp_stop();
            lblRunSt.Text = 'Detenido';  lblRunSt.FontColor = [0 0 0];
            % Restaurar botón Start (verde)
            btnStart.Text = '▶  Start';
            btnStart.BackgroundColor = [0.12 0.62 0.12];
            btnStart.FontColor = 'w';
            try
                lblRTT.Text = sprintf('Ts_meas=%.2fms  (última)',last_inter_ms);
            catch, end
            logMsg("Streaming detenido.");
            updatePlots();  updateRmsLabels();  updateDataInfo();
            drawnow;
        catch, end
        S.inLoop = false;
    end

%% ═══ TRAMA DE TELEMETRÍA v6 (PSoC → MATLAB) ═════════════════════════════════

    function processTelemFrame(frame)
        % Decodifica 32 bytes = 8 × float32 LE enviados por PSoC.
        % Bytes  0-15: y1 y2 u1 u2  (señales principales)
        % Bytes 16-31: x1i x2i x1o x2o  (estados observador, 0 si modo TF/OL)
        y1     = double(typecast(uint8(frame(1:4)),   'single'));
        y2     = double(typecast(uint8(frame(5:8)),   'single'));
        u1_raw = double(typecast(uint8(frame(9:12)),  'single'));
        u2     = double(typecast(uint8(frame(13:16)), 'single'));
        x1i    = double(typecast(uint8(frame(17:20)), 'single'));
        x2i    = double(typecast(uint8(frame(21:24)), 'single'));
        x1o    = double(typecast(uint8(frame(25:28)), 'single'));
        x2o    = double(typecast(uint8(frame(29:32)), 'single'));

        % Saturar u1 con los límites configurados (PSoC ya lo hace, aquí para display)
        u1_sat   = max(S.ctrl.sat_min, min(S.ctrl.sat_max, u1_raw));
        u1_unsat = u1_raw;

        % Mantener ref del inner sincronizada para simulación
        S.ctrl.inner.ref = u2;

        % ── Simulación — precisión igual a la del PSoC ───────────────────────
        % sim_cast1/2 convierten valores al tipo numérico activo, usando el
        % q_scale de cada planta para normalizar correctamente las señales
        % físicas en los paths Q31/Q15/Q7 (igual que PSoC ctrl_pend.c v6.3).
        numTypeSim = ddNumType.Value;
        qs1 = S.cfg(1).q_scale;  if isempty(qs1) || qs1 <= 0, qs1 = 1; end
        qs2 = S.cfg(2).q_scale;  if isempty(qs2) || qs2 <= 0, qs2 = 1; end
        sim_cast1 = @(x) num_cast(x, numTypeSim, qs1);
        sim_cast2 = @(x) num_cast(x, numTypeSim, qs2);

        ysim1 = nan;
        c1 = S.cfg(1);
        if c1.sim_enabled && ~isempty(c1.sim_Ad)
            u_c  = sim_cast1(u1_sat);
            ysim1 = double(sim_cast1(c1.sim_Cd * c1.sim_x) + c1.sim_Dd * u_c);
            S.cfg(1).sim_x = sim_cast1(c1.sim_Ad * c1.sim_x + c1.sim_Bd * u_c);
        end

        ysim2 = nan;
        c2 = S.cfg(2);
        if c2.sim_enabled && ~isempty(c2.sim_Ad)
            u2_c = sim_cast2(u2);
            ysim2 = double(sim_cast2(c2.sim_Cd * c2.sim_x) + c2.sim_Dd * u2_c);
            S.cfg(2).sim_x = sim_cast2(c2.sim_Ad * c2.sim_x + c2.sim_Bd * u2_c);
        end

        % ── Acumular error RMS ────────────────────────────────────────────────
        if ~isnan(ysim1)
            S.rms1_sum2 = S.rms1_sum2 + (ysim1 - y1)^2;
            S.rms1_n    = S.rms1_n + 1;
        end
        if ~isnan(ysim2)
            S.rms2_sum2 = S.rms2_sum2 + (ysim2 - y2)^2;
            S.rms2_n    = S.rms2_n + 1;
        end

        % ── Guardar datos ─────────────────────────────────────────────────────
        n = S.framesTotal + 1;
        S.nVec(end+1,1)        = n;
        S.u1Vec(end+1,1)       = u1_sat;
        S.u1unsat_Vec(end+1,1) = u1_unsat;
        S.y1Vec(end+1,1)       = y1;
        S.u2Vec(end+1,1)       = u2;
        S.y2Vec(end+1,1)       = y2;
        S.x1iVec(end+1,1)      = x1i;
        S.x2iVec(end+1,1)      = x2i;
        S.x1oVec(end+1,1)      = x1o;
        S.x2oVec(end+1,1)      = x2o;
        S.ySimVec1(end+1,1)    = ysim1;
        S.ySimVec2(end+1,1)    = ysim2;
        S.framesTotal = n;

        if numel(S.nVec) > MAX_PTS
            k = numel(S.nVec) - MAX_PTS + 1;
            S.nVec        = S.nVec(k:end);
            S.u1Vec       = S.u1Vec(k:end);
            S.u1unsat_Vec = S.u1unsat_Vec(k:end);
            S.y1Vec       = S.y1Vec(k:end);
            S.u2Vec       = S.u2Vec(k:end);
            S.y2Vec       = S.y2Vec(k:end);
            S.x1iVec      = S.x1iVec(k:end);
            S.x2iVec      = S.x2iVec(k:end);
            S.x1oVec      = S.x1oVec(k:end);
            S.x2oVec      = S.x2oVec(k:end);
            S.ySimVec1    = S.ySimVec1(k:end);
            S.ySimVec2    = S.ySimVec2(k:end);
        end
    end

%% ═══ SERIALIZACIÓN DE PARÁMETROS PARA PSoC ═══════════════════════════════════

    function c = build_coeffs_for_psoc(cfg, Fs)
        % Construye el vector de 25 floats que espera ctrl_apply_coeffs en PSoC.
        % Layout (0-indexed en C, 1-indexed aquí):
        %   TF:  c(1..6)=b, c(7..12)=a, c(15)=N, c(16)=Fs
        %   SS:  c(1..4)=A(row-major), c(5..6)=B, c(7..8)=C, c(9)=D,
        %        c(10..11)=L, c(12..13)=K, c(14)=Kx, c(15)=N, c(16)=Fs
        c = zeros(25, 1);
        c(15) = double(max(1, round(cfg.N)));   % N decimation
        c(16) = Fs;                              % Fs_inner [Hz]

        mode_v = cfg_to_mode_val(cfg);

        if mode_v == 0  % TF
            b = cfg.tf_b(:);  a = cfg.tf_a(:);
            if numel(b) < 6, b(end+1:6) = 0; end
            if numel(a) < 6, a(end+1:6) = 0; end
            c(1:6)  = b(1:6);
            c(7:12) = a(1:6);
        elseif mode_v >= 1 && mode_v <= 4  % SS
            c(1) = cfg.ss_A(1,1);  c(2) = cfg.ss_A(1,2);
            c(3) = cfg.ss_A(2,1);  c(4) = cfg.ss_A(2,2);
            c(5) = cfg.ss_B(1);    c(6) = cfg.ss_B(2);
            c(7) = cfg.ss_C(1);    c(8) = cfg.ss_C(2);
            c(9) = cfg.ss_D;
            c(10) = cfg.ss_L(1);   c(11) = cfg.ss_L(2);
            c(12) = cfg.ss_K(1);   c(13) = cfg.ss_K(2);
            % c(14) = Kx: Nbar si no tiene integrador, Ki si tiene
            if cfg.has_int
                c(14) = cfg.ss_Ki;
            else
                c(14) = cfg.ss_Nbar;
            end
        end
        % OL (5) / OFF (6): solo N y Fs importan (ya seteados arriba)

        % c(17) = q_scale (v6.3): normalización de señales para paths Q31/Q15/Q7.
        % PSoC: c[16] (0-indexed). 0.0 en PSoC → usa default 1.0.
        qs = cfg.q_scale;
        if isempty(qs) || ~isnumeric(qs) || qs <= 0, qs = 1.0; end
        c(17) = qs;
    end

    function uartp_send_params(force)
        % Serializa y envía parámetros al PSoC, con verificación de eco.
        %
        % Si force=false (default, llamado desde Start):
        %   Compara el payload actual con S.last_sent_payload.
        %   Si son idénticos, no reenvía (PSoC ya tiene los datos correctos).
        %
        % Si force=true: siempre envía (sin comparar con last_sent_payload).
        %
        % Flujo de envío:
        %   1. MATLAB → 'p' + [len 2B] + [216B] + [xsum]
        %   2. PSoC   → 'K'  (checksum OK) o 'E' (error)
        %   3. MATLAB → 'v'
        %   4. PSoC   → [216B eco] + [xsum]  (lo que quedó almacenado)
        %   5. MATLAB verifica eco byte a byte. Error si no coincide.
        %   6. Guarda payload en S.last_sent_payload.
        if nargin < 1, force = false; end

        Fs      = edtFs(1).Value;
        ref0    = single(edtRef.Value);
        sat_mn  = single(edtSatMin.Value);
        sat_mx  = single(edtSatMax.Value);

        mode_inner = uint8(cfg_to_mode_val(S.cfg(1)));
        mode_outer = uint8(cfg_to_mode_val(S.cfg(2)));

        c_inner = single(build_coeffs_for_psoc(S.cfg(1), Fs));
        c_outer = single(build_coeffs_for_psoc(S.cfg(2), Fs));

        % Construir payload de 216 bytes
        payload = zeros(216, 1, 'uint8');
        payload(1) = mode_inner;
        payload(2) = mode_outer;
        NUM_TYPE_NAMES = {'f32','f64','f16','q31','q15','q7'};
        payload(3) = uint8(find(strcmp(NUM_TYPE_NAMES, ddNumType.Value), 1, 'first') - 1);
        payload(5:104)   = typecast(c_inner(:), 'uint8');
        payload(105:204) = typecast(c_outer(:), 'uint8');
        payload(205:208) = typecast(ref0,        'uint8');
        payload(209:212) = typecast(sat_mn,      'uint8');
        payload(213:216) = typecast(sat_mx,      'uint8');

        % ── Comparar con último payload enviado ──────────────────────────────
        if ~force && ~isempty(S.last_sent_payload) && isequal(payload, S.last_sent_payload)
            logMsg("Params sin cambios desde último envío — no se reenvían.");
            return;
        end

        % ── Calcular checksum XOR ─────────────────────────────────────────────
        xsum = uint8(0);
        for bi = 1:216
            xsum = bitxor(xsum, payload(bi));
        end

        % ── Enviar 'p' + len(2B LE) + payload + xsum ─────────────────────────
        len_bytes = typecast(uint16(216), 'uint8');
        packet = [uint8('p'); len_bytes(:); payload(:); xsum];
        write(S.sp, packet, "uint8");

        % ── Esperar 'K' del PSoC ──────────────────────────────────────────────
        resp = ll_readexact(1, STEP_TMO);
        if resp(1) ~= uint8('K')
            error("send_params: PSoC resp=%c (esperado K)", char(resp(1)));
        end

        % ── Verificación de eco: enviar 'v', leer 216B + xsum ────────────────
        write(S.sp, uint8('v'), "uint8");
        echo_raw = ll_readexact(217, STEP_TMO * 2);   % 216B eco + 1B xsum
        echo_payload = echo_raw(1:216);
        echo_xsum    = echo_raw(217);

        % Verificar checksum del eco
        xsum_echo = uint8(0);
        for bi = 1:216
            xsum_echo = bitxor(xsum_echo, echo_payload(bi));
        end
        if xsum_echo ~= echo_xsum
            error("send_params: xsum eco inválido (PSoC=%d, calc=%d)", echo_xsum, xsum_echo);
        end
        % Verificar que el eco coincide con lo enviado
        if ~isequal(echo_payload(:), payload(:))
            n_diff = sum(echo_payload(:) ~= payload(:));
            error("send_params: eco ≠ payload (%d bytes difieren)", n_diff);
        end

        % ── Guardar payload verificado ────────────────────────────────────────
        S.last_sent_payload = payload;

        logMsg(sprintf("✔ Params enviados y verificados. modeI=%d modeO=%d Fs=%.1fHz ref=%.4g numType=%s",...
            mode_inner, mode_outer, Fs, double(ref0), ddNumType.Value));
    end

    function uartp_update_ref(ref_val)
        % Envía 'u' + float32 LE para actualizar ref_inner en PSoC sin parar.
        ref_bytes = typecast(single(ref_val), 'uint8');
        write(S.sp, [uint8('u'), ref_bytes(:)'], "uint8");
    end

%% ═══ CONVERSIÓN NUMÉRICA (match PSoC precision) ══════════════════════════════

    function v = num_cast(x, type_str, q_scale)
        % Convierte x al tipo numérico activo del PSoC y devuelve double.
        % q_scale (opcional, default=1.0): normaliza señales físicas a [-1,1)
        %   antes de cuantizar en Q31/Q15/Q7, igual que PSoC ctrl_pend.c v6.3.
        %   Señales en unidades físicas: q = round(x/q_scale); salida = q*q_scale.
        if nargin < 3 || isempty(q_scale) || q_scale <= 0
            q_scale = 1.0;
        end
        switch type_str
            case 'f64'
                v = double(x);
            case 'f32'
                v = double(single(x));          % round-trip float32
            case 'f16'
                % Emular half-float: recortar mantisa f32 de 23 a 10 bits
                xs = single(x);
                bits = typecast(xs, 'uint32');
                bits = bitand(bits, uint32(0xFFFFFC00));  % trunca 13 LSB
                v = double(typecast(bits, 'single'));
            case 'q31'
                % Normalizar por q_scale → Q31 → des-normalizar.
                % Igual que PSoC: xn=x/qs, qv=round(xn*2^31), out=qv/2^31*qs.
                xn = double(x) / q_scale;
                v  = double(int32(xn * 2147483648.0)) / 2147483648.0 * q_scale;
            case 'q15'
                xn = double(x) / q_scale;
                v  = double(int16(xn * 32768.0)) / 32768.0 * q_scale;
            case 'q7'
                xn = double(x) / q_scale;
                v  = double(int8(xn * 128.0)) / 128.0 * q_scale;
            otherwise
                v = double(x);
        end
    end

%% ═══ ZOH DISCRETIZACIÓN (sin toolbox) ════════════════════════════════════════

    function [Ad, Bd] = c2d_zoh(Ac, Bc, Ts)
        n  = size(Ac,1);
        nu = size(Bc,2);
        M  = expm([Ac, Bc; zeros(nu, n+nu)] * Ts);
        Ad = M(1:n,    1:n);
        Bd = M(1:n, n+1:end);
    end

%% ═══ DATOS ═══════════════════════════════════════════════════════════════════

    function onExport(~,~)
        if isempty(S.nVec), uialert(fig,'No hay datos.','Exportar'); return; end
        [f,p] = uiputfile('*.mat','Exportar datos');
        if isequal(f,0), return; end
        try
            data.n         = S.nVec;
            data.u1        = S.u1Vec;
            data.u1_unsat  = S.u1unsat_Vec;
            data.y1        = S.y1Vec;
            data.u2        = S.u2Vec;   % R inner
            data.y2        = S.y2Vec;
            data.x1i       = S.x1iVec;  data.x2i = S.x2iVec;
            data.x1o       = S.x1oVec;  data.x2o = S.x2oVec;
            data.ysim1     = S.ySimVec1; data.ysim2 = S.ySimVec2;
            data.fecha     = datestr(now,'yyyy-mm-dd HH:MM:SS');
            data.num_type  = ddNumType.Value;
            data.Fs_inner  = edtFs(1).Value;
            data.N_plant2  = S.cfg(2).N;
            data.Fs_outer  = data.Fs_inner / max(1, data.N_plant2);
            data.sat_min   = edtSatMin.Value;
            data.sat_max   = edtSatMax.Value;
            data.scalers   = S.scalers;
            data.cfg       = S.cfg;
            data.ctrl_final = S.ctrl;
            save(fullfile(p,f),'-struct','data');
            logMsg("Exportado: " + string(f));
        catch e, logMsg("Export FAIL: " + string(e.message)); end
    end

    function onClear(~,~)
        S.nVec = [];  S.u1Vec = [];  S.u1unsat_Vec = [];
        S.y1Vec = [];  S.u2Vec = [];  S.y2Vec = [];
        S.x1iVec = [];  S.x2iVec = [];  S.x1oVec = [];  S.x2oVec = [];
        S.ySimVec1 = [];  S.ySimVec2 = [];
        S.framesTotal = 0;
        S.rms1_sum2 = 0;  S.rms1_n = 0;
        S.rms2_sum2 = 0;  S.rms2_n = 0;
        updatePlots();  updateDataInfo();  updateRmsLabels();
        logMsg("Datos borrados.");
    end

%% ═══ CIERRE ══════════════════════════════════════════════════════════════════

    function onClose(~,~)
        S.streamOn = false;
        for pp2 = 1:2
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
        try
            if ~isempty(S.scaler_popup_fig) && isvalid(S.scaler_popup_fig)
                delete(S.scaler_popup_fig);
            end
        catch, end
        try
            if S.isConnected && ~isempty(S.sp) && isvalid(S.sp)
                flush(S.sp);  delete(S.sp);
            end
        catch, end
        delete(fig);
    end

end % pendulo_gui2
