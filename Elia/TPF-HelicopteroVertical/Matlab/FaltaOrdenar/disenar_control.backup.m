function S = disenar_control(G, fs, metodo, varargin)
%DISENAR_CONTROL  Diseña y evalúa controladores digitales SISO (simple y robusto).
%
%   S = disenar_control(G, fs, metodo, Name,Value,...)
%
% Entradas:
%   G      : planta continua SISO (tf/ss/zpk). Si viene discreta, se convierte con d2c(zoh) (warning).
%   fs     : frecuencia de muestreo [Hz]. Ts = 1/fs.
%   metodo : 'zoh' o 'tustin' para discretizar la planta usada en diseño.
%
% Name-Value:
%   'algo'          : 'menu' (default) | 'designer' | 'opt_intra' | 'pp_obs' | 'pp_obs_int' | 'dlqr' | 'lqg'
%   'observer'      : 'actual' (default) | 'predictor'        (aplica en pp_obs/pp_obs_int/lqg)
%   'N'             : muestras de simulación (default 400)
%   'subN'          : submuestras por Ts para intersample (default 30)
%   'u_sat'         : [umin umax] saturación (default [-Inf Inf])
%   'designer_mode' : 'rlocus' (default) | 'default'
%
% Salida struct S:
%   .Ts,.fs,.G,.Gd,.Gd_zoh
%   .Cd (si TF) o .ctrl (si estado-espacio)
%   .step, .ramp  (señales, métricas, intersample)
%
% Notas:
% - Simulación inter-muestra siempre usa ZOH real (independiente de 'metodo').
% - Para state feedback (PP/DLQR/LQG) se calcula Kr para tracking unitario.

% =================== Parse ===================
ip = inputParser;
ip.addParameter('algo','menu',@(s)ischar(s)||isstring(s));
ip.addParameter('observer','actual',@(s)ischar(s)||isstring(s));
ip.addParameter('N',400,@(x)isnumeric(x)&&isscalar(x)&&x>=10);
ip.addParameter('subN',30,@(x)isnumeric(x)&&isscalar(x)&&x>=2);
ip.addParameter('u_sat',[-Inf Inf],@(v)isnumeric(v)&&numel(v)==2&&v(1)<v(2));
ip.addParameter('designer_mode','rlocus',@(s)ischar(s)||isstring(s));
ip.parse(varargin{:});
P = ip.Results;

algo = lower(string(P.algo));
algo = validatestring(algo, {'menu','designer','opt_intra','pp_obs','pp_obs_int','dlqr','lqg'});

obsType = lower(string(P.observer));
obsType = validatestring(obsType, {'actual','predictor'});

designer_mode = lower(string(P.designer_mode));
designer_mode = validatestring(designer_mode, {'rlocus','default'});

N    = P.N;
subN = P.subN;
u_sat = P.u_sat(:).';

if fs <= 0, error('fs debe ser > 0 (Hz).'); end
Ts = 1/fs;

% =================== Normalizar planta ===================
G = normalize_plant_local(G);

if ~is_siso_local(G)
    error('G debe ser SISO.');
end

% Discretizaciones
metodo = lower(string(metodo));
metodo = validatestring(metodo, {'zoh','tustin'});
Gd     = c2d(G, Ts, char(metodo));
Gd_zoh = c2d(G, Ts, 'zoh');   % útil para RLocus y para "actuador real"

% struct salida base
S = struct();
S.fs = fs;
S.Ts = Ts;
S.G  = G;
S.Gd = Gd;
S.Gd_zoh = Gd_zoh;

S.Cd   = [];     % si controlador TF
S.ctrl = [];     % si controlador estado-espacio
S.CL   = [];     % lazo cerrado lineal (cuando se arma)

S.step = [];
S.ramp = [];
S.note = "";

% =================== Menú si corresponde ===================
if algo == "menu"
    k = menu('Elegí acción',...
        'Abrir Control System Designer (planta discreta)',...
        'Óptimo con oscilaciones intramuestra: C = inv(Gd) * 1/(z-1)',...
        'Polos + observador (sin integrador, con Kr)',...
        'Polos + observador (integrador clásico, con Kr opcional)',...
        'DLQR (estado medido, con Kr)',...
        'LQG (DLQR + Kalman, con Kr)');
    map = ["designer","opt_intra","pp_obs","pp_obs_int","dlqr","lqg"];
    algo = map(k);
end

% =================== Ejecutar opción ===================
switch algo
    case "designer"
        sys_open = Gd_zoh; % para RLocus suele tener más sentido con ZOH
        if strcmp(designer_mode, "default")
            controlSystemDesigner(sys_open);
        else
            controlSystemDesigner('rlocus', sys_open);
        end
        S.note = "Control System Designer abierto con Gd_zoh. Exportá el compensador si querés analizarlo luego.";
        return;

    case "opt_intra"
        % C = inv(Gd) * 1/(z-1)
        I = tf([0 1],[1 -1],Ts);  % z^-1/(1-z^-1) equivalente a 1/(z-1)
        Cd = inv(tf(Gd)) * I;

        if ~isproper(Cd)
            warning('Cd salió impropio/no causal. Puede explotar u[k]. Sigo igual (como pediste).');
        end

        S.Cd = Cd;
        S = analizar_y_simular_tf(S, Cd, u_sat, N, subN);

    case "pp_obs"
        % Polos + observador (sin integrador) con Kr
        [Ad,Bd,Cd_,Dd_] = ssdata(ss(Gd));
        n = size(Ad,1);

        polesK = pedir_vector(sprintf('Polos deseados del controlador (n=%d) en z:',n), default_poles(n,0.8,0.25));
        polesL = pedir_vector(sprintf('Polos deseados del observador (n=%d) en z:',n), default_poles(n,0.4,0.4));

        K = acker(Ad,Bd,polesK(:).');
        L = acker(Ad',Cd_',polesL(:).')';

        Kr = calcular_Kr(Ad,Bd,Cd_,Dd_,K,Ts);

        S.ctrl = struct('type','pp_obs','K',K,'L',L,'Kr',Kr,'observer',obsType,'integrator',false);
        S = sim_estado_espacio(S, Ad,Bd,Cd_,Dd_, K,L,Kr, obsType, false, [], u_sat, N, subN);

    case "pp_obs_int"
        % Integrador clásico v[k+1] = v[k] + (r - y)
        % Control: u = Kr*r - K*xhat - Ki*v   (Kr opcional; por default lo calculo igual)
        [Ad,Bd,Cd_,Dd_] = ssdata(ss(Gd));
        n = size(Ad,1);

        polesAug = pedir_vector(sprintf('Polos deseados del sistema aumentado (n+1=%d) en z:',n+1), ...
                                [default_poles(n,0.8,0.25); 0.6]);
        polesL   = pedir_vector(sprintf('Polos deseados del observador (n=%d) en z:',n), default_poles(n,0.4,0.4));
        
        n = max(size(Ad)); %El orden del sistema
        m = 1;            %La cantidad de salidas  

        % Aaug, Baug para integrador de error (SISO)
        Aaug = [Ad Bd;
               zeros(m,m+n)];
        Baug = [zeros(n,m); eye(m)];
        Caug = [Cd_ zeros(1,m)];
        
        Kaug = acker(Aaug,Baug,polesAug(:).');

        Aux = [Ad-eye(size(Ad)) Bd; Cd_*Ad Cd_*Bd];
        
        K2K1 = (Kaug + [zeros(1,max(size(Ad))) eye(m)])*inv(Aux);

        K  = K2K1(1,1:n);
        Ki = K2K1(1,n+1:n+m);


        L = acker(Ad',Cd_',polesL(:).')';


        S.ctrl = struct('type','pp_obs_int','K',K,'Ki',Ki,'L',L,'Kr',1,'observer',obsType,'integrator',true);
        S = sim_estado_espacio(S, Ad,Bd,Cd_,Dd_, K,L,1, obsType, true, Ki, u_sat, N, subN);

    case "dlqr"
        [Ad,Bd,Cd_,Dd_] = ssdata(ss(Gd));
        n = size(Ad,1);

        Q = pedir_matriz(sprintf('Q (nxn=%dx%d) para DLQR:',n,n), eye(n));
        R = pedir_matriz('R (1x1) para DLQR:', 1);

        K = dlqr(Ad,Bd,Q,R);
        Kr = calcular_Kr(Ad,Bd,Cd_,Dd_,K,Ts);

        S.ctrl = struct('type','dlqr','K',K,'Kr',Kr,'Q',Q,'R',R,'observer',"measured",'integrator',false);
        S = sim_estado_espacio_puro(S, Ad,Bd,Cd_,Dd_, K,Kr, u_sat, N, subN);

    case "lqg"
        [Ad,Bd,Cd_,Dd_] = ssdata(ss(Gd));
        n = size(Ad,1);

        Q  = pedir_matriz(sprintf('Q (nxn=%dx%d) para DLQR:',n,n), eye(n));
        R  = pedir_matriz('R (1x1) para DLQR:', 1);
        Qn = pedir_matriz(sprintf('Qn continuo (nxn=%dx%d) ruido de proceso:',n,n), 1e-3*eye(n));
        Rn = pedir_matriz('Rn continuo (1x1) ruido de medición:', 1e-3);

        K = dlqr(Ad,Bd,Q,R);
        Kr = calcular_Kr(Ad,Bd,Cd_,Dd_,K,Ts);

        % Kalman discreto desde covarianzas continuas usando kalmd
        % sys_kf debe tener inputs [u w] donde w es ruido de proceso (dim n)
        Ac = ss(G); % continuo
        [Acs,Bcs,Ccs,Dcs] = ssdata(Ac);

        if size(Bcs,2) ~= 1 || size(Ccs,1) ~= 1
            error('Se esperaba planta continua SISO para LQG (1 input u, 1 output y).');
        end

        sys_kf = ss(Acs, [Bcs eye(n)], Ccs, [Dcs zeros(1,n)]); % u y w como entradas

        if exist('kalmd','file') ~= 2
            error('No encuentro kalmd(). Necesitás Control System Toolbox. (kalmd diseña el estimador discreto desde Qn,Rn continuos).');
        end

        % Sintaxis documentada: [kest,L,P,M,Z] = kalmd(sys,Qn,Rn,Ts) :contentReference[oaicite:3]{index=3}
        [~, Lk] = kalmd(sys_kf, Qn, Rn, Ts);

        S.ctrl = struct('type','lqg','K',K,'Kr',Kr,'L',Lk,'Q',Q,'R',R,'Qn',Qn,'Rn',Rn,'observer',obsType,'integrator',false);
        S = sim_estado_espacio(S, Ad,Bd,Cd_,Dd_, K,Lk,Kr, obsType, false, [], u_sat, N, subN);

    otherwise
        error('algo no reconocido: %s', algo);
end

end % ===== end main =====


% =====================================================================
%                             Helpers
% =====================================================================

function G = normalize_plant_local(G)
    if ~(isa(G,'tf') || isa(G,'ss') || isa(G,'zpk'))
        error('G debe ser tf/ss/zpk.');
    end

    % Si es discreta, la convierto a continua (ZOH) para mantener el contrato "G continua"
    if is_dt_local(G)
        warning('G parece discreta (Ts ~= 0). La convierto a continua con d2c(...,''zoh'').');
        G = d2c(G,'zoh');
    end

    % Forzar representación ss continua
    G = ss(G);
end

function tf_siso = is_siso_local(sys)
    [ny,nu] = size(sys);
    tf_siso = (ny==1)&&(nu==1);
end

function tf_dt = is_dt_local(sys)
    % Usa isdt/isct si existen; si no, cae a Ts
    if exist('isdt','file') == 2
        tf_dt = isdt(sys);
    else
        tf_dt = isprop(sys,'Ts') && sys.Ts ~= 0;
    end
end

function v = pedir_vector(prompt, defaultV)
    def = mat2str(defaultV);
    answ = inputdlg(prompt, 'Entrada', [1 90], {def});
    if isempty(answ), error('Cancelado por el usuario.'); end
    v = str2num(answ{1}); %#ok<ST2NM>
    if isempty(v), error('Vector inválido.'); end
end

function M = pedir_matriz(prompt, defaultM)
    def = mat2str(defaultM);
    answ = inputdlg(prompt, 'Entrada', [4 90], {def});
    if isempty(answ), error('Cancelado por el usuario.'); end
    M = str2num(answ{1}); %#ok<ST2NM>
    if isempty(M), error('Matriz inválida.'); end
end

function poles = default_poles(n, a, b)
    poles = [];
    while numel(poles) < n
        if numel(poles) <= n-2
            poles = [poles; a+1i*b; a-1i*b]; %#ok<AGROW>
        else
            poles = [poles; a]; %#ok<AGROW>
        end
    end
    poles = poles(1:n);
end

function Kr = calcular_Kr(A,B,C,D,K,Ts)
    % Kr = 1 / dcgain( y/u ) del lazo cerrado con u = Kr*r - Kx
    % sys_u2y = ss(A-BK, B, C, D, Ts)
    sys_u2y = ss(A - B*K, B, C, D, Ts);
    g0 = dcgain(sys_u2y);  % DC gain en z=1 :contentReference[oaicite:4]{index=4}
    if isempty(g0) || ~isfinite(g0) || abs(g0) < 1e-12
        warning('dcgain(sys_u2y) inválido o ~0. Pongo Kr=1.');
        Kr = 1;
    else
        Kr = 1/g0;
    end
end


% =====================================================================
%                         TF controller sim
% =====================================================================

function S = analizar_y_simular_tf(S, Cd, u_sat, N, subN)
    Ts = S.Ts;

    % asegurar Cd como ss discreto
    try
        Cd_ss = ss(Cd);
    catch
        error('No se pudo convertir Cd a ss.');
    end
    if Cd_ss.Ts == 0
        Cd_ss = c2d(Cd_ss, Ts, 'tustin');
    end

    % CL lineal (sin saturación)
    try
        S.CL = feedback(series(tf(Cd), tf(S.Gd)), 1);
    catch
        S.CL = [];
    end

    % Root locus (solo si Cd es TF usable)
    try
        figure('Name','Root Locus (TF Controller): L(z)=C(z)G(z)'); grid on;
        rlocus(series(tf(Cd), tf(S.Gd_zoh)));
        title('Lugar de raíces usando Gd\_zoh');
    catch
    end

    % STEP
    r_step = ones(N,1);
    R = sim_loop_tf(ss(S.Gd), Cd_ss, Ts, r_step, u_sat);
    S.step = paquete_step(S.G, Ts, r_step, R, subN);

    % RAMP
    r_ramp = (0:N-1)'*Ts;
    R2 = sim_loop_tf(ss(S.Gd), Cd_ss, Ts, r_ramp, u_sat);
    S.ramp = paquete_ramp(S.G, Ts, r_ramp, R2, subN);
end


% =====================================================================
%                   State-space controllers sim
% =====================================================================

function S = sim_estado_espacio(S, A,B,C,D, K,L,Kr, obsType, useInt, Ki, u_sat, N, subN)
    Ts = S.Ts;

    % Polos (informativo)
    try
        figure('Name','Polos en plano-z'); grid on; hold on;
        p_cl = eig(A - B*K);
        plot(real(p_cl), imag(p_cl), 'x', 'LineWidth', 1.5);
        viscircles([0 0], 1,'LineStyle','--');
        title('Polos del lazo cerrado (A-BK)');
        xlabel('Re'); ylabel('Im');
    catch
    end

    % STEP
    r_step = ones(N,1);
    R = sim_obs(A,B,C,D,K,L,Kr,obsType,useInt,Ki,Ts,r_step,u_sat);
    S.step = paquete_step(S.G, Ts, r_step, R, subN);

    % RAMP
    r_ramp = (0:N-1)'*Ts;
    R2 = sim_obs(A,B,C,D,K,L,Kr,obsType,useInt,Ki,Ts,r_ramp,u_sat);
    S.ramp = paquete_ramp(S.G, Ts, r_ramp, R2, subN);

    % CL lineal aproximado (sin saturación, ignorando dinámica del estimador)
    try
        S.CL = ss(A - B*K, B*Kr, C, D, Ts);
    catch
        S.CL = [];
    end
end

function S = sim_estado_espacio_puro(S, A,B,C,D, K,Kr, u_sat, N, subN)
    Ts = S.Ts;

    % STEP
    r_step = ones(N,1);
    R = sim_puro(A,B,C,D,K,Kr,Ts,r_step,u_sat);
    S.step = paquete_step(S.G, Ts, r_step, R, subN);

    % RAMP
    r_ramp = (0:N-1)'*Ts;
    R2 = sim_puro(A,B,C,D,K,Kr,Ts,r_ramp,u_sat);
    S.ramp = paquete_ramp(S.G, Ts, r_ramp, R2, subN);

    try
        S.CL = ss(A - B*K, B*Kr, C, D, Ts);
    catch
        S.CL = [];
    end
end


% =====================================================================
%                         Result packaging
% =====================================================================

function P = paquete_step(Gc, Ts, r, R, subN)
    t = (0:numel(r)-1)'*Ts;

    y = R.y;
    ess = abs(1 - y(end));

    info = stepinfo(y, t, 1);

    % plots
    figure('Name','Step (discreto)'); tiledlayout(3,1);

    nexttile; grid on; hold on;
    stairs(t, r, 'LineWidth',1.0);
    stairs(t, y, 'LineWidth',1.2);
    legend('r[k]','y[k]','Location','best');
    title('Step: salida discreta');

    nexttile; grid on; hold on;
    stairs(t, R.u, 'LineWidth',1.2);
    title('u[k] (con saturación)'); ylabel('u');

    nexttile; grid on; hold on;
    if ~isempty(R.x)
        plot(t, R.x.', 'LineWidth',1.0);
        if ~isempty(R.xhat)
            plot(t, R.xhat.','--','LineWidth',1.0);
            title('x (línea) y x\_hat (punteada)');
        else
            title('x');
        end
    else
        plot(t, y, 'LineWidth',1.0);
        title('y[k]');
    end

    % intersample desde u[k] (ZOH real)
    Si = ver_intersample_desde_u(Gc, Ts, R.u, 0, subN);

    P = struct();
    P.t = t;
    P.r = r;
    P.y = y;
    P.u = R.u;
    P.x = R.x;
    P.xhat = R.xhat;
    P.v = R.v;
    P.stepinfo = info;
    P.ess = ess;
    P.intersample = Si;
end

function P = paquete_ramp(Gc, Ts, r, R, subN)
    t = (0:numel(r)-1)'*Ts;

    y = R.y;
    e = r - y;
    ess = abs(e(end));

    % plots
    figure('Name','Rampa (discreto)'); tiledlayout(3,1);

    nexttile; grid on; hold on;
    stairs(t, r, 'LineWidth',1.0);
    stairs(t, y, 'LineWidth',1.2);
    legend('r[k]','y[k]','Location','best');
    title('Rampa: salida discreta');

    nexttile; grid on; hold on;
    stairs(t, R.u, 'LineWidth',1.2);
    title('u[k] (con saturación)'); ylabel('u');

    nexttile; grid on; hold on;
    stairs(t, e, 'LineWidth',1.2);
    title('Error e[k]=r[k]-y[k]'); ylabel('e');

    % intersample desde u[k] (ZOH real)
    Si = ver_intersample_desde_u(Gc, Ts, R.u, 0, subN);

    P = struct();
    P.t = t;
    P.r = r;
    P.y = y;
    P.u = R.u;
    P.x = R.x;
    P.xhat = R.xhat;
    P.v = R.v;
    P.stepinfo = [];   % stepinfo no aplica bien en rampa
    P.ess = ess;
    P.intersample = Si;
end


% =====================================================================
%                           Sim cores
% =====================================================================

function R = sim_loop_tf(Gd_ss, Cd_ss, Ts, r, u_sat) %#ok<INUSD>
    [Ap,Bp,Cp,Dp] = ssdata(Gd_ss);
    [Ac,Bc,Cc,Dc] = ssdata(Cd_ss);

    N = numel(r);
    nxp = size(Ap,1);
    nxc = size(Ac,1);

    xp = zeros(nxp, N);
    xc = zeros(nxc, N);

    y = zeros(N,1);
    u = zeros(N,1);

    for k=1:N-1
        y(k) = Cp*xp(:,k) + Dp*u(k);
        e = r(k) - y(k);

        uc = Cc*xc(:,k) + Dc*e;
        xc(:,k+1) = Ac*xc(:,k) + Bc*e;

        u(k) = sat(uc, u_sat);
        xp(:,k+1) = Ap*xp(:,k) + Bp*u(k);
    end
    y(N) = Cp*xp(:,N) + Dp*u(N);

    R = struct('u',u,'y',y,'x',xp,'xhat',[],'v',[]);
end

function R = sim_puro(A,B,C,D,K,Kr,Ts,r,u_sat) %#ok<INUSD>
    N = numel(r);
    nx = size(A,1);
    x = zeros(nx,N);
    y = zeros(N,1);
    u = zeros(N,1);

    for k=1:N-1
        y(k) = C*x(:,k) + D*u(k);

        uc = Kr*r(k) - K*x(:,k);
        u(k) = sat(uc,u_sat);

        x(:,k+1) = A*x(:,k) + B*u(k);
    end
    y(N) = C*x(:,N) + D*u(N);

    R = struct('u',u,'y',y,'x',x,'xhat',x,'v',[]);
end

function R = sim_obs(A,B,C,D,K,L,Kr,obsType,useInt,Ki,Ts,r,u_sat) %#ok<INUSD>
    N = numel(r);
    nx = size(A,1);

    x    = zeros(nx,N);
    xhat = zeros(nx,N);
    y    = zeros(N,1);
    u    = zeros(N,1);
    v    = zeros(1,N); % integrador

    for k=1:N-1
        % salida actual
        y(k) = C*x(:,k) + D*u(k);

        % integrador clásico: v[k+1] = v[k] + (r - y)
        if useInt
            v(k+1) = v(k) + (r(k) - y(k));
        end

        % ley de control (con Kr!)
        if ~useInt
            uc = Kr*r(k) - K*xhat(:,k);
        else
            uc = Kr*r(k) - K*xhat(:,k) - Ki*v(k);
        end

        u(k) = sat(uc, u_sat);

        % evoluciona planta
        x(:,k+1) = A*x(:,k) + B*u(k);

        if strcmp(obsType,"predictor")
            % predictor: usa y(k)
            innov = y(k) - (C*xhat(:,k) + D*u(k));
            xhat(:,k+1) = A*xhat(:,k) + B*u(k) + L*innov;
        else
            % actual: usa y(k+1) real (después de actualizar la planta)
            xpred = A*xhat(:,k) + B*u(k);
            ykp1  = C*x(:,k+1) + D*u(k);
            innov = ykp1 - (C*xpred + D*u(k));
            xhat(:,k+1) = xpred + L*innov;
        end
    end
    y(N) = C*x(:,N) + D*u(N);

    R = struct('u',u,'y',y,'x',x,'xhat',xhat,'v',v);
end

function u = sat(u, lim)
    u = min(max(u, lim(1)), lim(2));
end


% =====================================================================
%                    Intersample (ZOH real)
% =====================================================================

function S = ver_intersample_desde_u(G, Ts, ud, M, N)
    if nargin < 5 || isempty(N), N = 30; end
    if nargin < 4 || isempty(M), M = 0;  end
    if Ts <= 0, error('Ts debe ser > 0.'); end

    uk = ud(:);
    if M > 0
        M = min(M, numel(uk));
        uk = uk(1:M);
    end

    K  = numel(uk) - 1;
    tk = (0:K).' * Ts;

    [tc, uc] = zoh_stretch(uk, Ts, N);
    yc       = lsim(G, uc, tc);

    yk = yc(1:N:end);

    figure('Name','Intersample desde u[k]');
    tiledlayout(2,1);

    nexttile; hold on; grid on; grid minor;
    plot(tc*1e3, yc, 'LineWidth', 1.2);
    stairs(tk*1e3, yk, 'k.', 'LineWidth', 1.0);
    xlabel('t [ms]'); ylabel('y');
    title(sprintf('Salida continua y_c(t) y muestras y[k]  (T_s = %.4g ms)', Ts*1e3));
    legend('y_c(t)','y[k]','Location','best');

    nexttile; hold on; grid on; grid minor;
    stairs(tk*1e3, uk, 'LineWidth', 1.2);
    xlabel('t [ms]'); ylabel('u[k]');
    title('Esfuerzo aplicado');

    S = struct('Ts', Ts, ...
               'tk', tk, 'uk', uk, 'yk', yk, ...
               'tc', tc, 'uc', uc, 'yc', yc);
end

function [t_hi, u_hi] = zoh_stretch(u_k, T, M)
    if size(u_k,2) > 1, u_k = u_k(:); end
    u_hi = repelem(u_k, M);
    t_hi = (0:numel(u_hi)-1).' * (T/M);
end
