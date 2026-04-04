%% =========================
%  LQG SERVO “firmware friendly”
%  (kalman + lqi + lqgtrack)
%  + estimador CURRENT (default en discreto)
%  + ruido de proceso y medición (sigma_w, sigma_v)
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
if size(C,1) ~= 1
    C = C(1,:);
    D = D(1,:);
end
if size(B,2) ~= 1
    error('Este script asume SISO (1 entrada). size(B,2)=%d', size(B,2));
end

fprintf('Ts=%.9f | n=%d\n', Ts, n);

%% =========================
% 2) RUIDOS (sigma -> covarianzas)
%% =========================
S = load("RQ_tuning_fixedR.mat");

Qn = S.Q_best;   % cov(w) : n×n
Rn = S.R;          % cov(v) : 1×1
sigma_v = S.sigma_v;
sigma_w = S.best_q;
Nn = zeros(n,1);           % cov(wv') asumimos 0

%% =========================
% 3) KALMAN: sys debe incluir el canal de ruido w
%% =========================
G = eye(n);          % w entra a todos los estados
H = zeros(1,n);      % y NO depende directamente de w (H=0)

sysK = ss(A, [B G], C, [D H], Ts);   % inputs: [u ; w]

% Para tu MATLAB: TYPE va como ÚLTIMO argumento.
% 'current' es el default, pero lo dejamos explícito.
[kest, Lk, Pk, Mx, Z, My] = kalman(sysK, Qn, Rn, Nn, 'current');

fprintf('\nLk (kalman) = '); disp(Lk);
fprintf('poles(A-LC) = '); disp(eig(A - Lk*C).');

%% =========================
% 4) LQI: K = [Kx Ki] para u = -Kx*xhat - Ki*xi
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

fprintf('\nKx (lqi) = '); disp(Kx);
fprintf('Ki (lqi) = '); disp(Ki);


%% =========================
% 6) SIMULACIÓN (tu estilo) con ruido de proceso+medición
%% =========================
N  = 1200;
t  = 0:Ts:(N-1)*Ts;

r = ones(1,N)*25;
r(1:30) = 0;

umin = -u_max; umax = u_max;

X  = zeros(n,N);   % estados reales
Xh = zeros(n,N);   % estados estimados (los que usás en tu control)
V  = zeros(1,N);   % integrador
U  = zeros(1,N);   % control

Y_true = zeros(1,N);
Y_meas = zeros(1,N);

for kidx = 1:N-1
    rk = r(kidx);

    % medición y[k]
    y_true_k = C*X(:,kidx);
    v_k      = sigma_v*randn;
    y_meas_k = y_true_k + v_k;

    Y_true(kidx) = y_true_k;
    Y_meas(kidx) = y_meas_k;

    % integrador xi[k+1] = xi[k] + (r-y)
    V(kidx+1) = V(kidx) + (rk - y_meas_k);

    % control: u = -Kx*xhat - Ki*xi   (OJO SIGNO: es MENOS Ki, no +Ki)
    u_unsat = -Kx*Xh(:,kidx) + Ki*V(kidx+1);
    u_k     = min(max(u_unsat, umin), umax);
    U(kidx) = u_k;

    % planta con ruido de proceso: x[k+1] = A x + B u + w
    w_k = sigma_w*randn(n,1);
    X(:,kidx+1) = A*X(:,kidx) + B*u_k + w_k;

    % ===== estimador (usa Lk). En kalman discreto:
    % xhat[k+1|k] = A xhat[k|k-1] + B u + L*(y[k] - C xhat[k|k-1] - D u)
    %
    % Tu Xh lo estás usando como xhat[k|k-1] (predicho). Entonces:
    Xh(:,kidx+1) = A*Xh(:,kidx) + B*u_k + Lk*(y_meas_k - (C*Xh(:,kidx) + D*u_k));
end

% último sample para plots
Y_true(N) = C*X(:,N);
Y_meas(N) = Y_true(N) + sigma_v*randn;
U(N)      = U(N-1);

%% =========================
% 7) PLOTS
%% =========================
figure('Name','y(t) — LQG + integrador + ruido');
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

figure('Name','xi(t) — integrador');
plot(t, V, 'LineWidth',1.4);
grid on; grid minor;
xlabel('t [s]'); ylabel('\xi');


Aaug = [A,B;...
        Kx-Kx*A-Ki*C*A,1-Kx*B-Ki*C*B];
Baug = [0;0;0;Ki];
Caug    = [C,0];
D = 0;
sysDaug = ss(Aaug,Baug,Caug,D,Ts);

figure;pzmap(sysDaug);grid on;zgrid;hold on;pzmap()
