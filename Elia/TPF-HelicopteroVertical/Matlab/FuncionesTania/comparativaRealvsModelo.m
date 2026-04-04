%% ============================================================
%  comparativaRealvsModelo_discreto_Ts.m
%
%  - Carga plantaC de 'planta (1).mat'
%  - Carga dataset .mat con n,y,u (Nx1)
%  - Selección de rango con 2 clicks (sobre y vs n)
%  - Remueve offset (por defecto: mean(y) del segmento)
%  - Discretiza: sysD = c2d(plantaC, Ts, 'zoh')
%  - Simula SOLO en Ts (sin inventar entre-muestras)
%  - Error y métricas solo en muestras
%  - Plots con stairs (nada de stem)
%% ============================================================
clear; close all; clc

%% =========================
% PARÁMETROS (EDITÁ ACÁ)
%% =========================
PLANTA_MAT_PATH  = 'planta (1).mat';
DATA_MAT_PATH    = 'D:\GitHub\auto\TPF-HelicopteroVertical\Matlab\Compensadores\BODE_practico.mat';

FsHz             = 1000;   % <-- tu frecuencia de muestreo
C2D_METHOD       = 'zoh';  % fijo zoh como pediste (podés cambiar si querés)

REMOVE_MEAN_Y    = true;   % resta mean(yD_seg)
REMOVE_MEAN_U    = false;  % opcional: resta mean(uD_seg)

VAR_N = "n";
VAR_Y = "y";
VAR_U = "u";

%% =========================
% 1) CARGA PLANTA CONTINUA
%% =========================
S = load(PLANTA_MAT_PATH);
if isfield(S,'plantaC')
    plantaC = S.plantaC;
elseif isfield(S,'sysC')
    plantaC = S.sysC;
else
    error('No encuentro "plantaC" ni "sysC" dentro de %s', PLANTA_MAT_PATH);
end

%% =========================
% 2) CARGA DATASET
%% =========================
D = load(DATA_MAT_PATH);
if ~isfield(D, VAR_N) || ~isfield(D, VAR_Y) || ~isfield(D, VAR_U)
    error('En %s deben existir las variables %s, %s, %s', DATA_MAT_PATH, VAR_N, VAR_Y, VAR_U);
end

n      = D.(VAR_N); n = n(:);
y_full = D.(VAR_Y); y_full = y_full(:);
u_full = D.(VAR_U); u_full = u_full(:);

Nfull = numel(n);
if numel(y_full) ~= Nfull || numel(u_full) ~= Nfull
    error('n, y, u deben tener el mismo largo (Nx1).');
end
if any(~isfinite(n)) || any(~isfinite(y_full)) || any(~isfinite(u_full))
    error('Hay NaN/Inf en n, y o u.');
end
if any(diff(n) ~= 1)
    warning('n no parece ser consecutivo con paso 1. Igual sigo.');
end

Ts = 1/FsHz;

%% =========================
% 3) FIGURA PREVIA vs n + clicks
%% =========================
figure('Name','Elegir rango (2 clicks en y)', 'Color','w');

ax1 = subplot(2,1,1);
stairs(n, u_full, 'LineWidth', 1.2);
grid on; xlabel('n'); ylabel('u'); title('u completo (vs n)');

ax2 = subplot(2,1,2);
stairs(n, y_full, 'LineWidth', 1.2);
grid on; xlabel('n'); ylabel('y'); title('y completo (vs n)  -> 2 clicks');

fprintf('\n2 clicks sobre el subplot de y (inicio y fin)\n');
axes(ax2); %#ok<LAXES>
[xc,~] = ginput(2);

n_ini = round(min(xc));
n_fin = round(max(xc));

n_ini = max(n_ini, n(1));
n_fin = min(n_fin, n(end));

fprintf('Rango elegido: n_ini=%d, n_fin=%d (%.3f s)\n', n_ini, n_fin, (n_fin-n_ini)*Ts);

axes(ax1); hold on; xline(n_ini,'--'); xline(n_fin,'--'); hold off;
axes(ax2); hold on; xline(n_ini,'--'); xline(n_fin,'--'); hold off;

if n_fin <= n_ini
    error('Rango inválido (n_fin <= n_ini).');
end

%% =========================
% 4) RECORTE + remover mean
%% =========================
idx  = (n >= n_ini) & (n <= n_fin);
nseg = n(idx);
yD   = y_full(idx);
uD   = u_full(idx);

N  = numel(nseg);
tD = (nseg - nseg(1)) * Ts;   % 0..(N-1)Ts (paso Ts)

y_mean = mean(yD);
u_mean = mean(uD);

if REMOVE_MEAN_Y, yD = yD - y_mean; end
if REMOVE_MEAN_U, uD = uD - u_mean; end

%% =========================
% 5) DISCRETIZAR A Ts (ZOH) Y SIMULAR EN Ts
%% =========================
sysD = c2d(plantaC, Ts, C2D_METHOD);

% Simulación discreta coherente: tD tiene paso Ts y sysD tiene Ts
ySim = lsim(sysD, uD, tD);
ySim = ySim(:);

% Alinear DC del sim con yD centrado (si removiste mean)
if REMOVE_MEAN_Y
    ySim = ySim - mean(ySim);
end

%% =========================
% 6) ERROR Y MÉTRICAS (todas en muestras)
%% =========================
e = yD - ySim;

MAE  = mean(abs(e));
MSE  = mean(e.^2);
RMSE = sqrt(MSE);

den = abs(yD); den(den < 1e-12) = NaN;
MAPE = mean(abs(e)./den, 'omitnan') * 100;

fprintf('\n================= MÉTRICAS (muestras) =================\n');
fprintf('Fs=%.3f Hz | Ts=%.6g s | c2d=%s\n', FsHz, Ts, C2D_METHOD);
fprintf('REMOVE_MEAN_Y=%d (y_mean=%.6g) | REMOVE_MEAN_U=%d (u_mean=%.6g)\n', ...
    REMOVE_MEAN_Y, y_mean, REMOVE_MEAN_U, u_mean);
fprintf('MAE  = %.6g\n', MAE);
fprintf('MSE  = %.6g\n', MSE);
fprintf('RMSE = %.6g\n', RMSE);
fprintf('MAPE = %.6g %%\n', MAPE);
fprintf('=======================================================\n\n');

%% =========================
% 7) FIGURA FINAL (3 subplots) - todo stairs
%% =========================
figure('Name','Comparación discreta (Ts)','Color','w');

subplot(3,1,1);
stairs(tD, uD, 'LineWidth', 1.3);
grid on; xlabel('t [s]'); ylabel('u');
title('Entrada u (digital, Ts)');

subplot(3,1,2);
stairs(tD, ySim, 'LineWidth', 1.2); hold on;
stairs(tD, yD,   'LineWidth', 1.3);
grid on; xlabel('t [s]'); ylabel('y');
title('Salida: y_{modelo}(Ts) vs y_{real}(Ts)');
legend('y_{modelo}','y_{real}','Location','best');

subplot(3,1,3);
stairs(tD, e, 'LineWidth', 1.3);
grid on; xlabel('t [s]'); ylabel('e = y_{real}-y_{mod}');
title(sprintf('Error | MAE=%.3g  RMSE=%.3g  MAPE=%.3g%%', MAE, RMSE, MAPE));

%% =========================
% 8) EXPORTAR results
%% =========================
results = struct();
results.PLANTA_MAT_PATH = PLANTA_MAT_PATH;
results.DATA_MAT_PATH   = DATA_MAT_PATH;

results.n_ini = n_ini;
results.n_fin = n_fin;

results.FsHz  = FsHz;
results.Ts    = Ts;

results.c2d_method = C2D_METHOD;

results.remove_mean_y = REMOVE_MEAN_Y;
results.remove_mean_u = REMOVE_MEAN_U;
results.y_mean = y_mean;
results.u_mean = u_mean;

results.sysD = sysD;

results.tD   = tD;
results.uD   = uD;
results.yD   = yD;
results.ySim = ySim;

results.e    = e;
results.MAE  = MAE;
results.MSE  = MSE;
results.RMSE = RMSE;
results.MAPE = MAPE;

assignin('base','results_compare',results);
disp('Listo: dejé todo también en "results_compare".');
