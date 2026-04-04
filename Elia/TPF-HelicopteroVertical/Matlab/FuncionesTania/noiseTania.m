close all; clear; clc

%% =========================
% 1) CARGA + DISCRETIZACIÓN
%% =========================
S = load('planta (1).mat');

if isfield(S,'plantaC')
    plantaC = S.plantaC;
elseif isfield(S,'sysC')
    plantaC = S.sysC;
else
    error('No encuentro "plantaC" ni "sysC" dentro de planta (1).mat');
end

Ts   = 1/100;                 % sample time
sysD = c2d(plantaC, Ts, 'zoh');
[A,B,C,D] = ssdata(ss(sysD));
n = size(A,1);

fprintf('Ts=%.9f | n=%d\n', Ts, n);
disp('A='); disp(A); disp('B='); disp(B); disp('C='); disp(C); disp('D='); disp(D);

%% =========================
% 2) PARÁMETROS + PESOS (LQR)
%% =========================
% Observador (z-plane)
p_obs  = [0.8 + 0.25i, 0.8 - 0.25i, 0.9];

% Objetivo práctico: step ~20 y u limitado a ±300
r_step = 20;
u_max  = 300;

% knobs (estos sí son los que tocás)
wy = 20;          % subí => seguís mejor la referencia (más agresivo)
wu = 500;         % subí => menos esfuerzo (más tímido)

% Q y R coherentes (OJO: R escala con u^2)
Q = wy*(C'*C) + 1e-8*eye(n);
R = wu/(u_max);

%% =========================
% 3) GANANCIAS: LQR + OBSERVADOR + Nbar
%% =========================
rc = rank(ctrb(A,B));
if rc < n
    error('El par (A,B) NO es controlable (rank=%d < n=%d).', rc, n);
end

% --- LQR discreto (con fallback) ---
if exist('dlqr','file') == 2
    [K, P, e_cl] = dlqr(A, B, Q, R);
elseif exist('dare','file') == 2
    [P,~,~] = dare(A,B,Q,R);
    K = (R + B'*P*B)\(B'*P*A);
    e_cl = eig(A - B*K);
else
    [P, K, e_cl, info] = dlqr_iter_nolic(A,B,Q,R);
    fprintf('DLQR sin toolbox: iters=%d, err=%.3e\n', info.iters, info.err);
end

fprintf('LQR: eig(A-BK) = \n'); disp(e_cl.');

% --- Observador (place) ---
Ke_pred = place(A', C', p_obs).';            % predictor: A - Ke*C
Ke_act  = place(A', (C*A)', p_obs).';        % "actual": A - Ke*C*A

% --- Nbar (SISO) ---
if size(C,1) ~= 1
    error('Tu C no es SISO (tiene %d salidas). Elegí una fila de C.', size(C,1));
end
[~,~,Nbar] = refi(A, B, C, K);
fprintf('Nbar=%.6g\n', Nbar);

%% =========================
% 4) SIMULACIÓN: SIN RUIDO vs CON RUIDO (double y single)
%% =========================
N        = 200;
ulim_sat = u_max;
ulim_inf = Inf;

% referencia
r = zeros(1,N);
r(2:end) = r_step;

% --------- RUIDO (config) ----------
% Si querés que CADA corrida sea distinta: rng('shuffle')
% Si querés repetible: rng(1)
rng(1);

noise.enable   = true;

% medición (sensor): y_meas = y_true + v_y
noise.sigma_y  = 2;     % unidades de y (si y está en cm, esto es 0.5 cm)

% actuador (jitter): u_app = sat( quant(u_cmd + v_u) )
noise.sigma_u  = 2.0;     % unidades de u (si u está en us/Δus, esto es 2)
noise.q_u      = 1.0;     % cuantización de u (1 unidad). Poné Inf para desactivar

% proceso: x[k+1] = A x + B u + w
noise.sigma_w  = 0.0;     % OJO: no tiene unidades físicas aquí (es “stress test”)

% pre-generado para que double y single usen EXACTAMENTE el mismo ruido
noise.vy = noise.sigma_y * randn(1,N);
noise.vu = noise.sigma_u * randn(1,N);
noise.wx = noise.sigma_w * randn(n,N);

noise_off = noise;
noise_off.enable = false;
noise_off.vy = zeros(1,N);
noise_off.vu = zeros(1,N);
noise_off.wx = zeros(n,N);

% --- IDEAL (sin sat, sin ruido) ---
out_pred_ideal = sim_obs_loop(A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_inf,false,false,noise_off);
out_act_ideal  = sim_obs_loop(A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_inf,true ,false,noise_off);

% --- REAL (sat, sin ruido) ---
out_pred_sat_clean = sim_obs_loop(A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_sat,false,false,noise_off);
out_act_sat_clean  = sim_obs_loop(A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_sat,true ,false,noise_off);

% --- REAL (sat, con ruido) ---
out_pred_sat_noise = sim_obs_loop(A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_sat,false,false,noise);
out_act_sat_noise  = sim_obs_loop(A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_sat,true ,false,noise);

% --- REAL (sat, con ruido) en FLOAT32 ---
out_pred_sat_noise_f = sim_obs_loop(A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_sat,false,true,noise);
out_act_sat_noise_f  = sim_obs_loop(A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_sat,true ,true,noise);

t = (0:N-1)*Ts;

%% =========================
% 5) DIAGNÓSTICOS (para NO autoengañarte)
%% =========================
% Ruido realmente inyectado en y (medición)
eyp = out_pred_sat_noise.y_meas - out_pred_sat_noise.y_true;
eya = out_act_sat_noise.y_meas  - out_act_sat_noise.y_true;

fprintf('\n--- CHECK RUIDO ---\n');
fprintf('sigma_y=%.3g | RMS(y_meas-y_true) Pred=%.3g Act=%.3g\n', noise.sigma_y, rms(eyp), rms(eya));
fprintf('sigma_u=%.3g | q_u=%g\n', noise.sigma_u, noise.q_u);

fprintf('\n--- SATURACIÓN ---\n');
fprintf('Pred clean: max|u|=%.2f sat=%d\n', max(abs(out_pred_sat_clean.u)), sum(abs(out_pred_sat_clean.u) >= ulim_sat-1e-9));
fprintf('Pred noise: max|u|=%.2f sat=%d\n', max(abs(out_pred_sat_noise.u)), sum(abs(out_pred_sat_noise.u) >= ulim_sat-1e-9));
fprintf('Act  clean: max|u|=%.2f sat=%d\n', max(abs(out_act_sat_clean.u)),  sum(abs(out_act_sat_clean.u)  >= ulim_sat-1e-9));
fprintf('Act  noise: max|u|=%.2f sat=%d\n', max(abs(out_act_sat_noise.u)),  sum(abs(out_act_sat_noise.u)  >= ulim_sat-1e-9));

%% =========================
% 6) PLOTS (se ve sí o sí: y_true vs y_meas)
%% =========================
figure('Name','Predictor: y (clean vs noise) [sat]');
plot(t, out_pred_sat_clean.y_true, 'LineWidth',1.6); hold on;
plot(t, out_pred_sat_noise.y_meas, '.-'); 
plot(t, r, 'k--','LineWidth',1.2);
grid on; xlabel('t [s]'); ylabel('y');
legend('y true (clean)','y meas (noise)','r','Location','best');
title('Predictor: salida con y sin ruido (sat)');

figure('Name','Predictor: u (clean vs noise) [sat]');
plot(t, out_pred_sat_clean.u, 'LineWidth',1.6); hold on;
plot(t, out_pred_sat_noise.u, '.-');
yline(+ulim_sat,'k--'); yline(-ulim_sat,'k--');
grid on; xlabel('t [s]'); ylabel('u');
legend('u clean','u noise','Location','best');
title('Predictor: control con y sin ruido (sat)');

figure('Name','Actual: y (clean vs noise) [sat]');
plot(t, out_act_sat_clean.y_true, 'LineWidth',1.6); hold on;
plot(t, out_act_sat_noise.y_meas, '.-');
plot(t, r, 'k--','LineWidth',1.2);
grid on; xlabel('t [s]'); ylabel('y');
legend('y true (clean)','y meas (noise)','r','Location','best');
title('Actual: salida con y sin ruido (sat)');

figure('Name','Actual: u (clean vs noise) [sat]');
plot(t, out_act_sat_clean.u, 'LineWidth',1.6); hold on;
plot(t, out_act_sat_noise.u, '.-');
yline(+ulim_sat,'k--'); yline(-ulim_sat,'k--');
grid on; xlabel('t [s]'); ylabel('u');
legend('u clean','u noise','Location','best');
title('Actual: control con y sin ruido (sat)');

figure('Name','Ruido de medición (y_meas - y_true)');
plot(t, eyp, t, eya);
grid on; xlabel('t [s]'); ylabel('error de medición');
legend('Pred','Act','Location','best');
title('El ruido que realmente se inyectó en la medición');

figure('Name','Float32 - Double (y_meas) [sat+noise]');
plot(t, out_pred_sat_noise.y_meas - out_pred_sat_noise_f.y_meas, ...
     t, out_act_sat_noise.y_meas  - out_act_sat_noise_f.y_meas);
grid on; xlabel('t [s]'); ylabel('double - float32');
legend('Pred','Act','Location','best');
title('Diferencia por precisión float32');

%% =========================
% 7) Z-PLANE (polos planta, CL, obs)
%% =========================
p_ol = eig(A);
p_cl = eig(A - B*K);
p_op = eig(A - Ke_pred*C);
p_oa = eig(A - Ke_act*C*A);

try
    z_plant = tzero(ss(A,B,C,D,Ts));
catch
    z_plant = [];
end

plot_zplane('Z-plane: Predictor', p_ol, p_cl, p_op, z_plant);
plot_zplane('Z-plane: Actual'   , p_ol, p_cl, p_oa, z_plant);

%% ============================================================
% ===================== FUNCIONES LOCALES ======================

function out = sim_obs_loop(A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim,use_actual,use_single,noise)
    if nargin < 15 || isempty(noise)
        noise.enable=false; noise.vy=0; noise.vu=0; noise.wx=0; noise.q_u=Inf;
    end

    if use_single
        A=single(A); B=single(B); C=single(C); D=single(D);
        K=single(K); Nbar=single(Nbar);
        Ke_pred=single(Ke_pred); Ke_act=single(Ke_act);
        r=single(r); Ts=single(Ts); ulim=single(ulim);
    end

    n = size(A,1); N = numel(r);

    x  = zeros(n,N,'like',A);
    xh = zeros(n,N,'like',A);

    y_true = zeros(1,N,'like',A);
    y_meas = zeros(1,N,'like',A);

    u_cmd  = zeros(1,N,'like',A);
    u_app  = zeros(1,N,'like',A);

    % ruido casteado
    vy = zeros(1,N,'like',A);
    vu = zeros(1,N,'like',A);
    wx = zeros(n,N,'like',A);

    if isfield(noise,'enable') && noise.enable
        vy = cast(noise.vy,'like',A);
        vu = cast(noise.vu,'like',A);
        wx = cast(noise.wx,'like',A);
    end

    q_u = Inf;
    if isfield(noise,'q_u'), q_u = noise.q_u; end
    q_u = cast(q_u,'like',A);

    % y(1)
    y_true(1) = C*x(:,1) + D*0;
    y_meas(1) = y_true(1) + vy(1);

    for k=1:N-1
        % control con xhat
        u_unsat  = Nbar*r(k) - K*xh(:,k);
        u_cmd(k) = sat(u_unsat, ulim);

        % actuador (ruido + quant + saturación física final)
        u_app(k) = u_cmd(k) + vu(k);
        if isfinite(double(q_u))
            u_app(k) = round(u_app(k)/q_u)*q_u;
        end
        u_app(k) = sat(u_app(k), ulim);

        % planta (proceso)
        x(:,k+1) = A*x(:,k) + B*u_app(k) + wx(:,k);

        y_true(k)   = C*x(:,k)   + D*u_app(k);
        y_true(k+1) = C*x(:,k+1) + D*u_app(k);

        y_meas(k)   = y_true(k)   + vy(k);
        y_meas(k+1) = y_true(k+1) + vy(k+1);

        if ~use_actual
            % predictor: usa y_meas(k)
            yhat_k = C*xh(:,k) + D*u_app(k);
            xh(:,k+1) = A*xh(:,k) + B*u_app(k) + Ke_pred*( y_meas(k) - yhat_k );
        else
            % actual: usa y_meas(k+1)
            z   = A*xh(:,k) + B*u_app(k);
            yzh = C*z + D*u_app(k);
            xh(:,k+1) = z + Ke_act*( y_meas(k+1) - yzh );
        end
    end

    % último
    u_cmd(N) = sat(Nbar*r(N) - K*xh(:,N), ulim);
    u_app(N) = u_cmd(N) + vu(N);
    if isfinite(double(q_u))
        u_app(N) = round(u_app(N)/q_u)*q_u;
    end
    u_app(N) = sat(u_app(N), ulim);

    y_true(N) = C*x(:,N) + D*u_app(N);
    y_meas(N) = y_true(N) + vy(N);

    out.x = x; out.xh = xh;
    out.y_true = y_true;
    out.y_meas = y_meas;
    out.u_cmd  = u_cmd;
    out.u_app  = u_app;

    % compatibilidad con tu estilo anterior
    out.y = y_meas;
    out.u = u_app;
end

function y = sat(u,lim)
    if isinf(lim)
        y = u;
    else
        y = min(max(u, -lim), lim);
    end
end

function plot_zplane(figName, p_ol, p_cl, p_obs, z_plant)
    figure('Name',figName,'NumberTitle','off');
    hold on; grid on; grid minor; axis equal;
    title('Z-plane');
    xlabel('Re\{z\}'); ylabel('Im\{z\}');

    th = linspace(0,2*pi,400);
    plot(cos(th), sin(th), 'k:'); % unit circle

    plot(real(p_ol),  imag(p_ol),  'o', 'LineWidth', 1.5);
    plot(real(p_cl),  imag(p_cl),  'x', 'LineWidth', 1.8);
    plot(real(p_obs), imag(p_obs), '^', 'LineWidth', 1.8);

    if ~isempty(z_plant)
        plot(real(z_plant), imag(z_plant), 's', 'LineWidth', 1.5);
        legend('unit circle','poles plant (A)','poles CL (A-BK)','poles obs','zeros plant','Location','bestoutside');
    else
        legend('unit circle','poles plant (A)','poles CL (A-BK)','poles obs','Location','bestoutside');
    end

    xlim([-1.2 1.2]); ylim([-1.2 1.2]);
end

function [Nx,Nu,Nbar] = refi(phi,gam,Hr,K)
    I=eye(size(phi));
    [m,n]=size(Hr);
    np=inv([phi-I gam;Hr zeros(m)])*([zeros(n,m);eye(m)]);
    Nx=np(1:n,:);
    Nu=np(n+1:n+m,:);
    Nbar=Nu+K*Nx;
end

function [P, K, e_cl, info] = dlqr_iter_nolic(A,B,Q,R)
    maxit = 5000;
    tol   = 1e-10;

    P = Q;
    err = Inf;

    for it = 1:maxit
        G = R + B'*P*B;
        Ktmp = G \ (B'*P*A);
        Pn = A'*P*A - A'*P*B*Ktmp + Q;

        err = norm(Pn - P, 'fro');
        P = Pn;

        if err < tol
            break;
        end
    end

    K = (R + B'*P*B) \ (B'*P*A);
    e_cl = eig(A - B*K);

    info.iters = it;
    info.err   = err;
    info.converged = (err < tol);

    if ~info.converged
        warning('Riccati iterativa NO convergió (err=%.3e).', err);
    end
end
