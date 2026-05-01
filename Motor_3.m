%% ========================================
%  CÓDIGO PARA PROBAR PI PASO A PASO
%  Ajusta los parámetros y observa resultados
% ========================================

clear all; close all; clc;

%% PARÁMETROS DEL MOTOR DC
Ra = 4.04;
La = 0.00255;
Km = 0.1863;
Bm = 1.391e-3;
Jm = 8.315e-4;

numM = Km;
denM = [La*Jm, (Ra*Jm + La*Bm), (Ra*Bm + Km^2)];
G_m = tf(numM, denM);

%% DISCRETIZACIÓN
Fs = 400;           % Frecuencia de muestreo [Hz]
T = 1/Fs;           % Período: 0.0025 s = 2.5 ms

plantaMD = c2d(G_m, T, 'zoh');
[numMD, denMD] = tfdata(plantaMD, 'v');

b_planta = numMD / denMD(1);
a_planta = denMD / denMD(1);

fprintf('========================================\n');
fprintf('  SISTEMA DISCRETIZADO\n');
fprintf('========================================\n');
fprintf('Frecuencia de muestreo: %.0f Hz\n', Fs);
fprintf('Período de muestreo: %.4f s (%.2f ms)\n\n', T, T*1000);

%% ========================================
%  AJUSTE DE PARÁMETROS
% ========================================

fprintf('========================================\n');
fprintf('  AJUSTE DE PARÁMETROS\n');
fprintf('========================================\n\n');

% ┌─────────────────────────────────────────┐
% │ AJUSTA ESTOS VALORES Y VUELVE A EJECUTAR│
% └─────────────────────────────────────────┘

% BASADO EN TUS RESULTADOS:
% - Kp=5 da overshoot de 1.7% (estable)
% - Kp=10 da overshoot de 30% (límite)
% - Kp≥40 es INESTABLE

Kp = 1;         % <-- Valor conservador y estable
Ti = 0.8;       % <-- Empezar aquí


% ┌─────────────────────────────────────────┐
% │        FIN DE PARÁMETROS AJUSTABLES      │
% └─────────────────────────────────────────┘

fprintf('Parámetros de prueba:\n');
fprintf('  Kp = %.1f\n', Kp);
fprintf('  Ti = %.3f s\n\n', Ti);

% Crear controlador
C_s = tf(Kp * [Ti, 1], [Ti, 0]);
C_z = c2d(C_s, T, 'tustin');

% Lazo cerrado
cloop = feedback(C_z * plantaMD, 1);

% Análisis
polos = pole(cloop);
max_polo = max(abs(polos));

fprintf('Polos del lazo cerrado:\n');
for i = 1:length(polos)
    fprintf('  Polo %d: %.6f %+.6fi  (|z| = %.4f)\n', ...
            i, real(polos(i)), imag(polos(i)), abs(polos(i)));
end

if max_polo < 1
    fprintf('\n✅ Sistema ESTABLE\n');
else
    fprintf('\n❌ Sistema INESTABLE\n');
end

% Respuesta al escalón
[y, t] = step(cloop, 3);

pico = max(y);
overshoot = ((pico - 1) / 1) * 100;

idx_settled = find(abs(y - 1) <= 0.02, 1);
if ~isempty(idx_settled)
    t_settling = t(idx_settled);
else
    t_settling = NaN;
end

error_ss = abs(1 - mean(y(end-50:end)));

fprintf('\nMétricas:\n');
fprintf('  Overshoot: %.1f%%\n', overshoot);
fprintf('  Settling time: %.3f s\n', t_settling);
fprintf('  Error SS: %.6f\n', error_ss);

fprintf('\n========================================\n');
fprintf('EVALUACIÓN:\n');
fprintf('========================================\n');

score = 0;
if max_polo < 1
    fprintf('✅ Estable\n');
    score = score + 1;
else
    fprintf('❌ Inestable - REDUCIR Kp\n');
end

if overshoot < 30
    fprintf('✅ Overshoot aceptable (%.1f%%)\n', overshoot);
    score = score + 1;
else
    fprintf('⚠️  Overshoot alto (%.1f%%) - REDUCIR Kp o AUMENTAR Ti\n', overshoot);
end

if error_ss < 0.01
    fprintf('✅ Error estado estacionario prácticamente cero\n');
    score = score + 1;
else
    fprintf('⚠️  Error SS alto - REDUCIR Ti\n');
end

if t_settling < 2.0
    fprintf('✅ Respuesta rápida (Ts = %.2fs)\n', t_settling);
    score = score + 1;
else
    fprintf('⚠️  Respuesta lenta - AUMENTAR Kp o REDUCIR Ti\n');
end

fprintf('\nPuntuación: %d/4\n', score);
if score >= 3
    fprintf('🎯 ¡Buen ajuste!\n');
elseif score >= 2
    fprintf('⚠️  Mejorable, ajusta más\n');
else
    fprintf('❌ Necesita ajustes importantes\n');
end

fprintf('========================================\n\n');

%% ========================================
%  GRÁFICOS
% ========================================

figure('Position', [100 100 1400 800]);

% Gráfico 1: Respuesta al escalón
subplot(2,3,1);
step(cloop, 'b', 3);
grid on;
title('Respuesta al Escalón', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Amplitud');

% Gráfico 2: Mapa polo-cero
subplot(2,3,2);
pzmap(cloop, 'b');
zgrid;
title('Mapa Polo-Cero', 'FontSize', 11, 'FontWeight', 'bold');

% Gráfico 3: Bode
subplot(2,3,3);
bode(C_z * plantaMD, 'b');
grid on;
title('Diagrama de Bode', 'FontSize', 11, 'FontWeight', 'bold');

% Gráfico 4: Root Locus
subplot(2,3,4);
rlocus(C_z * plantaMD);
zgrid;
title(sprintf('Root Locus (Kp=%.0f, Ti=%.2f)', Kp, Ti), 'FontSize', 11, 'FontWeight', 'bold');

% Gráfico 5: Polos
subplot(2,3,5);
plot(real(polos), imag(polos), 'bx', 'MarkerSize', 12, 'LineWidth', 2); hold on;
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'k--', 'LineWidth', 1);
grid on;
axis equal;
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);
xlabel('Real');
ylabel('Imaginario');
title('Polos del Sistema', 'FontSize', 11, 'FontWeight', 'bold');

% Gráfico 6: Métricas
subplot(2,3,6);
bar([overshoot; error_ss*100; t_settling*100]);
grid on;
title('Métricas de Desempeño', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'XTickLabel', {'Overshoot [%]', 'Error SS [%]', 'Ts [×10ms]'});
ylabel('Valor');

%% ========================================
%  SECCIÓN 4: SIMULACIÓN COMPLETA
% ========================================

fprintf('========================================\n');
fprintf('  SIMULACIÓN TEMPORAL\n');
fprintf('========================================\n\n');

% Extraer coeficientes discretos
[numCD, denCD] = tfdata(C_z, 'v');
b_ctrl = numCD / denCD(1);
a_ctrl = denCD / denCD(1);

d0 = b_ctrl(1);
d1 = b_ctrl(2);
c1 = a_ctrl(2);

fprintf('Coeficientes del controlador discreto:\n');
fprintf('  d0 = %.10f\n', d0);
fprintf('  d1 = %.10f\n', d1);
fprintf('  c1 = %.10f\n\n', c1);

fprintf('Ecuación en diferencias:\n');
fprintf('  u[k] = %.4f*u[k-1] + %.4f*e[k] + %.4f*e[k-1]\n\n', -c1, d0, d1);

fprintf('Código C para PSoC:\n');
fprintf('--------------------\n');
fprintf('static const float d0 = %.10ff;\n', d0);
fprintf('static const float d1 = %.10ff;\n', d1);
fprintf('static const float c1 = %.10ff;\n\n', c1);
fprintf('float u_k = -c1*u_k1 + d0*e_k + d1*e_k1;\n');
fprintf('--------------------\n\n');

% Simulación
t_sim = 3.0;
td = 0:T:t_sim;
n = length(td);

ud = zeros(1, n);
yd = zeros(1, n);
ed = zeros(1, n);
ref_d = zeros(1, n);

limite = 1264;

% Estados
e_k1 = 0;
u_k1 = 0;
y_k1 = 0;
y_k2 = 0;
u_planta_k1 = 0;
u_planta_k2 = 0;

% Referencia
for k = 1:n
    if td(k) < 0.5
        ref_d(k) = 0;
    elseif td(k) < 2.0
        ref_d(k) = 10.0;
    else
        ref_d(k) = 5.0;
    end
end

% Loop de control
for k = 2:n
    ruido = (rand() - 0.5) * 0.05;
    y_medido = yd(k-1) + ruido;
    
    ed(k) = ref_d(k) - y_medido;
    
    ud_calc = -c1 * u_k1 + d0 * ed(k) + d1 * e_k1;
    
    if ud_calc > limite
        ud(k) = limite;
    elseif ud_calc < -limite
        ud(k) = -limite;
    else
        ud(k) = ud_calc;
    end
    
    yd(k) = b_planta(2)*u_planta_k1 + b_planta(3)*u_planta_k2 ...
          - a_planta(2)*y_k1 - a_planta(3)*y_k2;
    
    if abs(td(k) - 1.5) < 0.01
        yd(k) = yd(k) - 2.0;
    end
    
    e_k1 = ed(k);
    u_k1 = ud(k);
    u_planta_k2 = u_planta_k1;
    u_planta_k1 = ud(k);
    y_k2 = y_k1;
    y_k1 = yd(k);
end

% Gráficos de simulación
figure('Position', [100 100 1400 600]);

subplot(2,1,1);
plot(td, ref_d, 'r--', 'LineWidth', 2); hold on;
plot(td, yd, 'b', 'LineWidth', 1.5);
grid on;
title(sprintf('Simulación Temporal (Kp=%.1f, Ti=%.3f)', Kp, Ti), 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Velocidad [rad/s]');
legend('Referencia', 'Velocidad Real', 'Location', 'best');

subplot(2,1,2);
stairs(td, ud, 'g', 'LineWidth', 1.5); hold on;
yline(limite, 'r--', 'LineWidth', 1, 'DisplayName', 'Límite superior');
yline(-limite, 'r--', 'LineWidth', 1, 'DisplayName', 'Límite inferior');
grid on;
title('Señal de Control (PWM)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('u[k]');
legend('PWM', 'Location', 'best');

fprintf('========================================\n');
fprintf('  RESUMEN FINAL\n');
fprintf('========================================\n');
fprintf('Parámetros ajustados:\n');
fprintf('  Kp = %.1f\n', Kp);
fprintf('  Ti = %.3f s\n\n', Ti);

fprintf('Error estado estacionario: %.6f rad/s\n', abs(ref_d(end) - mean(yd(end-50:end))));
fprintf('========================================\n\n');

fprintf('💡 TIP: Ajusta Kp y Ti en la SECCIÓN 2 y vuelve a ejecutar\n\n');