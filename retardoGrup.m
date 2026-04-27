%% --- Cálculo de Retardo de Grupo de una TF Digital en Tiempo Real ---
clear; clc; close all;

%% 1. PARÁMETROS DEL SISTEMA
Ts = 1/400;  % Período de muestreo en segundos (5ms = 0.005s)
            % Cambiar según tu sistema:
            % Ts = 5e-3;   % 5 ms
            % Ts = 10e-3;  % 10 ms
            % Ts = 1e-3;   % 1 ms

%% 2. FUNCIÓN DE TRANSFERENCIA H(z) = B(z) / A(z)
% Coeficientes del numerador (ceros)
% Coeficientes del denominador (polos)

% Ejemplo 1: Tu controlador P discretizado
b1 = [1, -1];
a1 = [1, -0.455938127765996];

% Ejemplo 2: Derivada discreta (filtro diferenciador)
b2 = [1, -1];
a2 = [1, -0.2078795763507621];

%% 3. CONFIGURACIÓN DEL CÁLCULO
N = 1024;  % Número de puntos de frecuencia (potencia de 2)

%% 4. CALCULAR RETARDO DE GRUPO
[gd1, w] = grpdelay(b1, a1, N);
[gd2, ~] = grpdelay(b2, a2, N);

% Convertir frecuencia angular a Hz
f_Hz = w / (2*pi) * (1/Ts);  % Frecuencia en Hz

% Convertir retardo de grupo de [muestras] a [tiempo]
% gd está en muestras, multiplicar por Ts para obtener segundos
gd1_ms = gd1 * Ts * 1000;      % Retardo en milisegundos
gd1_us = gd1 * Ts * 1e6;       % Retardo en microsegundos

gd2_ms = gd2 * Ts * 1000;
gd2_us = gd2 * Ts * 1e6;

%% 5. DETERMINAR UNIDAD ÓPTIMA (ms o µs)
% Si el retardo máximo es < 10ms, usar microsegundos
max_retardo_ms = max([gd1_ms; gd2_ms]);

if max_retardo_ms < 10
    % Usar microsegundos
    gd1_plot = gd1_us;
    gd2_plot = gd2_us;
    unidad_tiempo = '\mus';
    label_tiempo = 'Retardo de Grupo (μs)';
else
    % Usar milisegundos
    gd1_plot = gd1_ms;
    gd2_plot = gd2_ms;
    unidad_tiempo = 'ms';
    label_tiempo = 'Retardo de Grupo (ms)';
end

%% 6. GRAFICAR RESULTADOS

% --- Gráfico 1: Retardo vs Frecuencia Normalizada ---
figure('Position', [100 100 1200 500]);

subplot(1,2,1);
plot(w/pi, gd1_plot, 'LineWidth', 2, 'Color', [0.85 0.33 0.10], 'DisplayName', 'H_1(z)');
hold on;
plot(w/pi, gd2_plot, 'LineWidth', 2, 'Color', [0 0.45 0.74], 'DisplayName', 'H_2(z)');
grid on;
title(sprintf('Retardo de Grupo (T_s = %.1f ms)', Ts*1000), 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia Normalizada (×π rad/muestra)', 'FontSize', 11);
ylabel(label_tiempo, 'FontSize', 11);
legend('Location', 'best');
xlim([0 1]);

% --- Gráfico 2: Retardo vs Frecuencia en Hz ---
subplot(1,2,2);
plot(f_Hz, gd1_plot, 'LineWidth', 2, 'Color', [0.85 0.33 0.10], 'DisplayName', 'H_1(z)');
hold on;
plot(f_Hz, gd2_plot, 'LineWidth', 2, 'Color', [0 0.45 0.74], 'DisplayName', 'H_2(z)');
grid on;
title(sprintf('Retardo de Grupo (T_s = %.1f ms)', Ts*1000), 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frecuencia (Hz)', 'FontSize', 11);
ylabel(label_tiempo, 'FontSize', 11);
legend('Location', 'best');
xlim([0 1/(2*Ts)]);  % Hasta frecuencia de Nyquist

%% 7. MOSTRAR INFORMACIÓN EN CONSOLA
fprintf('\n========================================\n');
fprintf('  ANÁLISIS DE RETARDO DE GRUPO\n');
fprintf('========================================\n');
fprintf('Período de muestreo (Ts): %.3f ms (%.0f Hz)\n', Ts*1000, 1/Ts);
fprintf('Frecuencia de Nyquist:    %.1f Hz\n\n', 1/(2*Ts));

fprintf('--- Función H1(z) ---\n');
fprintf('Numerador:   b = [');
fprintf('%.6f ', b1);
fprintf(']\n');
fprintf('Denominador: a = [');
fprintf('%.6f ', a1);
fprintf(']\n');
fprintf('Retardo promedio: %.2f %s (%.2f muestras)\n', mean(gd1_plot), unidad_tiempo, mean(gd1));
fprintf('Retardo máximo:   %.2f %s (%.2f muestras)\n', max(gd1_plot), unidad_tiempo, max(gd1));
fprintf('Retardo mínimo:   %.2f %s (%.2f muestras)\n\n', min(gd1_plot), unidad_tiempo, min(gd1));

fprintf('--- Función H2(z) ---\n');
fprintf('Numerador:   b = [');
fprintf('%.6f ', b2);
fprintf(']\n');
fprintf('Denominador: a = [');
fprintf('%.6f ', a2);
fprintf(']\n');
fprintf('Retardo promedio: %.2f %s (%.2f muestras)\n', mean(gd2_plot), unidad_tiempo, mean(gd2));
fprintf('Retardo máximo:   %.2f %s (%.2f muestras)\n', max(gd2_plot), unidad_tiempo, max(gd2));
fprintf('Retardo mínimo:   %.2f %s (%.2f muestras)\n', min(gd2_plot), unidad_tiempo, min(gd2));
fprintf('========================================\n\n');