%% =========================
%  LQG DISCRETO (dlqr + dlqe)
%  + Integrador (estado v)
%  + SOLO observador ACTUAL (corrige con y(k+1))
%  + Ruido de PROCESO y MEDICIÓN usando:
%      sigma_w = S.q_global
%      sigma_v = S.sigmaR
%  (Planta discreta desde .mat, orden 3)
%% =========================
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

Ts   = 1/100;                 % 100 Hz
sysD = c2d(plantaC, Ts, 'zoh');
[A,B,C,D] = ssdata(ss(sysD));

n = size(A,1);
if n ~= 3
    error('Esta plantilla asume planta de orden 3. n=%d', n);
end
if size(C,1) ~= 1
    C = C(1,:);
end

fprintf('Ts=%.9f | n=%d\n', Ts, n);

%% =========================
% 2) DISEÑO LQR AUMENTADO (integrador) — dlqr
%% =========================
m = 1;                      % integrador escalar
Ahat = [A B; zeros(m,n+m)];
Bhat = [zeros(n,m); eye(m)];
Chat = [C zeros(1,m)];

% --- interpretación física: "step ~20" y "u no pase ~300" ---
y_step = 20;
u_max  = 300;

% knobs
wy = 20;        % subí => seguís más (más agresivo)
wu = 50;       % subí => penalizás más u (más tímido)
wv = 0.00000000000001;         % subí => penalizás integrador

% OJO: acá tu “escala física” es discutible. Esto es heurístico.
qy = wy*(1/(y_step));
ru = wu*(1/(u_max));
qv = wv*(1/(y_step));

Qa = blkdiag(qy*(C'*C) + 1e-8*eye(n), qv);
R  = ru;

[Khat,Pa,ecl] = dlqr(Ahat, Bhat, Qa, R);

Aux  = [A-eye(size(A))  B;
        C*A             C*B];

K2K1 = (Khat + [zeros(1,n) eye(m)]) / Aux;

Kx = K2K1(1,1:n);
Ki = K2K1(1,n+1);

fprintf('\nKx = '); disp(Kx);
fprintf('Ki = '); disp(Ki);
fprintf('poles(Ahat-Bhat*Khat) = '); disp(ecl.');

%% =========================
% 3) DISEÑO KALMAN (dlqe) — LQE discreto
%% =========================
% x(k+1)=A x + B u + G w
% y(k)=C x + v
G = eye(n);

S = load("RQ_estimados.mat");
sigma_w = S.q_global;  % proceso
sigma_v = S.sigmaR;    % medición

Qn = (sigma_w^2)*eye(n);   % cov(w) para dlqe
Rn = (sigma_v^2);          % cov(v)

[Lk,pobs,eobs] = dlqe(A, G, C, Qn, Rn);

fprintf('\nLk (Kalman) = '); disp(Lk);
fprintf('poles(A-LCA) = '); disp(eobs.');

%% =========================
% 4) SIMULACIÓN (SOLO ACTUAL) con ruido de proceso+medición
%% =========================
N  = 1200;
t  = 0:Ts:(N-1)*Ts;

% referencia
r = ones(1,N)*25;
r(1:30) = 0;

umin = -u_max; umax = u_max;

% ---- estados reales ----
X  = zeros(n,N);

% ---- estimados ----
Xh = zeros(n,N);

% integrador
V  = zeros(1,N);

% control
U  = zeros(1,N);

% outputs medidos/verdaderos
Y_true = zeros(1,N);
Y_meas = zeros(1,N);

% ruido (gaussiano blanco)
% w_k ~ N(0, Qn), v_k ~ N(0, Rn)
for k = 1:N-1
    rk = r(k);

    % medición actual (y(k))
    y_true_k = C*X(:,k);
    v_k      = sigma_v*randn;           % SISO
    y_meas_k = y_true_k + v_k;

    Y_true(k) = y_true_k;
    Y_meas(k) = y_meas_k;

    % integrador (usa medición para integral “realista”)
    V(k+1) = V(k) + (rk - y_meas_k);

    % ley de control (con xhat)
    u_unsat = -Kx*Xh(:,k) + Ki*V(k+1);
    u_k     = min(max(u_unsat, umin), umax);
    U(k)    = u_k;

    % ===== planta con ruido de proceso =====
    w_k = sigma_w*randn(n,1);           % como G=I => w entra directo
    X(:,k+1) = A*X(:,k) + B*u_k + w_k;

    % medición siguiente y(k+1) (para “ACTUAL”)
    y_true_next = C*X(:,k+1);
    v_next      = sigma_v*randn;
    y_meas_next = y_true_next + v_next;

    % ===== observador ACTUAL =====
    % primero predict
    z = A*Xh(:,k) + B*u_k;
    % luego corrige con y(k+1)
    Xh(:,k+1) = z + Lk*(y_meas_next - C*z);
end

% último sample para plots
Y_true(N) = C*X(:,N);
Y_meas(N) = Y_true(N) + sigma_v*randn;
U(N)      = U(N-1);

%% =========================
% 5) PLOTS
%% =========================
figure('Name','y(t) — LQG + integrador (solo actual) + ruido');
plot(t, Y_true, 'LineWidth',1.2); hold on;
plot(t, Y_meas, '.', 'LineWidth',1.0);
plot(t, r, 'k--','LineWidth',1.1);
grid on; grid minor;
xlabel('t [s]'); ylabel('y');
legend('y true','y meas','r','Location','best');

figure('Name','u(t) — saturado');
plot(t, U, 'LineWidth',1.4);
grid on; grid minor;
xlabel('t [s]'); ylabel('u');
ylim([umin umax]);

figure('Name','v(t) — integrador');
plot(t, V, 'LineWidth',1.4);
grid on; grid minor;
xlabel('t [s]'); ylabel('v');

figure('Name','Z-plane — planta, aumentado, obs');
subplot(1,3,1); zgrid; hold on; grid on; box on;
p_planta = eig(A);
plot(real(p_planta), imag(p_planta), 'ko','MarkerSize',8,'LineWidth',1.4);
title('Planta (A)');

subplot(1,3,2); zgrid; hold on; grid on; box on;
p_aug    = eig(Ahat);
p_aug_cl = eig(Ahat - Bhat*Khat);
plot(real(p_aug),    imag(p_aug),    'm^','MarkerSize',8,'LineWidth',1.4);
plot(real(p_aug_cl), imag(p_aug_cl), 'c*','MarkerSize',10,'LineWidth',1.6);
title('Aumentado: Ahat y Ahat-BhatK');

subplot(1,3,3); zgrid; hold on; grid on; box on;
p_obs = eig(A - Lk*C);
plot(real(p_obs), imag(p_obs), 'rx','MarkerSize',9,'LineWidth',1.5);
title('Obs error (A-LC)');

sgtitle('LQG discreto + integrador (dlqr + dlqe) — SOLO ACTUAL + ruido');
