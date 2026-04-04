%% ============================================================
% Tuning de Q = q*I para Kalman (dlqe) con R FIJO (sigma_v ~ 2..3 cm)
% usando datos reales (dato1,dato2) desde iddata.
%
% - Planta: BJ continuo en planta (1).mat  -> G = B/F
% - Datos : datos.mat con dato1,dato2 (iddata) con y en cm, u en du_us
% - Ts_target: 0.01 s (decimación exacta si Ts original es divisor)
%
% Objetivo: var(eta) ~ 1 donde eta = nu / sqrt(S), S = C P C' + R
% (criterio consistente para elegir q cuando el modelo no es perfecto)
%% ============================================================
close all; clear; clc

%% ===== rutas =====
mat_planta = 'planta (1).mat';
mat_datos  = 'D:\GitHub\auto\TPF-HelicopteroVertical\Matlab\PruebasEmpiricas\datos.mat';

Ts_target = 0.01;                 % 100 Hz

%% ============================================================
% 1) CARGAR PLANTA Y DISCRETIZAR
%% ============================================================
S = load(mat_planta);

if isfield(S,'plantaC')
    plantaC = S.plantaC;
elseif isfield(S,'sysC')
    plantaC = S.sysC;
else
    error('No encuentro "plantaC" ni "sysC" dentro de %s', mat_planta);
end

% Planta determinista desde BJ: G = B/F
G  = tf(plantaC.B, plantaC.F);
Gd = c2d(ss(G), Ts_target, 'zoh');
[A,B,C,D] = ssdata(Gd);

n = size(A,1);
if size(B,2) ~= 1
    error('SISO requerido. size(B,2)=%d', size(B,2));
end
if size(C,1) ~= 1
    C = C(1,:);
    D = D(1,:);
end
if isempty(D), D = 0; end
D = double(D);

fprintf('Planta discretizada: n=%d, Ts_target=%.6g\n', n, Ts_target);

%% ============================================================
% 2) CARGAR DATOS (iddata) Y ARMAR u,y (concatenados)
%% ============================================================
DD = load(mat_datos);
use_names = {'dato1','dato2'};

u_all = [];
y_all = [];

for i=1:numel(use_names)
    nm = use_names{i};
    if ~isfield(DD,nm)
        error('No existe %s en %s', nm, mat_datos);
    end

    zi = DD.(nm);
    if ~isa(zi,'iddata')
        error('%s no es iddata.', nm);
    end

    Ts_i = zi.Ts;
    if isempty(Ts_i) || Ts_i <= 0
        error('%s: iddata sin Ts válido.', nm);
    end

    % señales crudas
    yraw = zi.OutputData(:);
    uraw = zi.InputData(:);

    % limpiar NaN/Inf (sin isfinite por compat)
    m = ~isnan(yraw) & ~isinf(yraw) & ~isnan(uraw) & ~isinf(uraw);
    yraw = yraw(m);
    uraw = uraw(m);

    % decimación exacta a Ts_target
    if abs(Ts_i - Ts_target) > 1e-12
        ratio = Ts_target / Ts_i;
        if abs(ratio - round(ratio)) > 1e-12
            error('%s: Ts_target/Ts_i no entero. Ts_i=%.17g Ts_target=%.17g', nm, Ts_i, Ts_target);
        end
        M = round(ratio);
        fprintf('%s: decimación M=%d (Ts %.17g -> %.17g)\n', nm, M, Ts_i, Ts_target);
        yraw = yraw(1:M:end);
        uraw = uraw(1:M:end);
    else
        fprintf('%s: Ts ya coincide (%.17g)\n', nm, Ts_i);
    end

    % (opcional) detrend: sacá media para no pelear con offsets
    yraw = yraw - mean(yraw);
    uraw = uraw - mean(uraw);

    fprintf('%s: N=%d\n', nm, numel(yraw));

    % concatenar
    y_all = [y_all; yraw(:)]; %#ok<AGROW>
    u_all = [u_all; uraw(:)]; %#ok<AGROW>
end

y = y_all;
u = u_all;

fprintf('TOTAL: N=%d muestras\n', numel(y));

%% ============================================================
% 3) FIJAR R (ruido de medición) y TUNEAR q en Q=q*I
%% ============================================================
sigma_v = 2.043227851;     % cm  (poné 2..3 según tu medición)
R = sigma_v^2;

q_grid = logspace(-12, 2, 120);

best_q = NaN; best_err = Inf;
stats_q = zeros(numel(q_grid),1);
stats_var_eta = zeros(numel(q_grid),1);
stats_var_nu  = zeros(numel(q_grid),1);

for i=1:numel(q_grid)
    q = q_grid(i);
    Q = q*eye(n);

    % Kalman (w entra a estados)
    Gk = eye(n);
    [L,P,~] = dlqe(A,Gk,C,Q,R);

    % Sinnov estacionario
    Sinnov = C*P*C' + R;
    if Sinnov < 1e-12, Sinnov = 1e-12; end

    xhat = zeros(n,1);
    nu   = zeros(numel(y),1);

    for k=1:numel(y)
        ypred = C*xhat + D*u(k);
        nu(k) = y(k) - ypred;

        % delayed estimator estilo firmware
        xhat = xhat + L*nu(k);
        xhat = A*xhat + B*u(k);
    end

    eta = nu / sqrt(Sinnov);

    var_eta = var(eta,1);
    var_nu  = var(nu,1);

    stats_q(i) = q;
    stats_var_eta(i) = var_eta;
    stats_var_nu(i)  = var_nu;

    err = abs(log(var_eta));   % objetivo var_eta ~ 1
    if err < best_err
        best_err = err;
        best_q = q;
        best_L = L;
        best_P = P;
        best_S = Sinnov;
    end
end

fprintf('\n=== Resultado tuning Q con R fijo ===\n');
fprintf('sigma_v = %.3f cm => R=%.3f cm^2\n', sigma_v, R);
fprintf('q_best  = %.3e\n', best_q);
fprintf('Sinnov  = %.3f  => sigma_pred_innov = %.3f cm\n', best_S, sqrt(best_S));
fprintf('L_best  =\n'); disp(best_L);

%% ============================================================
% 4) PLOTS
%% ============================================================
figure('Name','Tuning Q: var(eta)'); 
semilogx(stats_q, stats_var_eta,'LineWidth',1.2); grid on; grid minor;
xlabel('q'); ylabel('var(\eta)'); title('Objetivo: var(\eta) \approx 1');

figure('Name','Tuning Q: sigma(nu)'); 
semilogx(stats_q, sqrt(stats_var_nu),'LineWidth',1.2); grid on; grid minor;
xlabel('q'); ylabel('sigma(\nu) [cm]'); title('Innovación (predicción) vs q');

%% ============================================================
% 5) Guardar
%% ============================================================
Q_best = best_q*eye(n);
L_best = best_L;
save('RQ_tuning_fixedR.mat','sigma_v','R','best_q','Q_best','L_best','best_P','best_S','A','B','C','D','Ts_target');
disp('Guardado: RQ_tuning_fixedR.mat');
