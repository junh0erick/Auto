clc; close all; clear;

%% =========================
% Parámetros
%% =========================
titulo = 'SS predictor (u e y vs n)';
DATA_MAT_PATH = ['SS_actual_practico.mat'];

%% =========================
% Carga
%% =========================
S = load(DATA_MAT_PATH);

% Chequeos mínimos + fallback por si n no existe
if isfield(S,'n')
    n = S.n(:);
else
    warning('No existe S.n. Genero n = (1:N).');
    n = (1:numel(S.y)).';
end

if ~isfield(S,'y') || ~isfield(S,'u')
    error('El .mat debe contener variables "y" y "u".');
end

y = S.y(:);
u = S.u(:);

N = min([numel(n), numel(y), numel(u)]);
n = n(1:N); y = y(1:N); u = u(1:N);

%% =========================
% Figura linda
%% =========================
figure('Color','w','Name','u e y vs n');

tiledlayout(2,1, 'Padding','compact', 'TileSpacing','compact');

% --- y vs n ---
nexttile;
stairs(n, y, 'LineWidth', 1.4);
grid on; grid minor;
xlabel('n [muestra]');
ylabel('y');
title('Salida y vs n');

% --- u vs n ---
nexttile;
stairs(n, u, 'LineWidth', 1.4);
grid on; grid minor;
xlabel('n [muestra]');
ylabel('u');
title('Esfuerzo u vs n');

% Título general
sgtitle(titulo, 'FontWeight','bold');
