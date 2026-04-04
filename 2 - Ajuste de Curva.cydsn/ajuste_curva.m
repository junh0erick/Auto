%% Ajuste de Curva — Motor DC
%  Datos: PWM | VOLTAJE | RPM
%  Ajustes polinomiales orden 1 a 5 en un mismo grafico

clc; clear; close all;

%% 1. Leer datos del Excel
datos = readtable('BarridoMotor.xlsx');

pwm     = datos.PWM(:);
voltaje = datos.VOLTAJE(:);
rpm     = datos.RPM(:);

ordenes = 1:5;
colores = {'r', 'g', 'b', 'm', 'c'};

%% 2. PWM → RPM
figure(1); hold on; grid on;
plot(pwm, rpm, 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Datos');
pwm_vec = linspace(min(pwm), max(pwm), 500)';

fprintf('=== Ajuste PWM → RPM ===\n');
for k = ordenes
    tipo = sprintf('poly%d', k);
    [f, gof] = fit(pwm, rpm, tipo);
    plot(pwm_vec, feval(f, pwm_vec), colores{k}, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Orden %d  R²=%.4f', k, gof.rsquare));
    fprintf('  Orden %d: R² = %.6f\n', k, gof.rsquare);
end
xlabel('PWM (cuentas)'); ylabel('RPM');
title('Ajuste PWM \rightarrow RPM — Ordenes 1 a 5');
legend('Location', 'northwest'); hold off;

%% 3. PWM → Voltaje
figure(2); hold on; grid on;
plot(pwm, voltaje, 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Datos');
volt_vec = linspace(min(pwm), max(pwm), 500)';

fprintf('\n=== Ajuste PWM → Voltaje ===\n');
for k = ordenes
    tipo = sprintf('poly%d', k);
    [f, gof] = fit(pwm, voltaje, tipo);
    plot(volt_vec, feval(f, volt_vec), colores{k}, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Orden %d  R²=%.4f', k, gof.rsquare));
    fprintf('  Orden %d: R² = %.6f\n', k, gof.rsquare);
end
xlabel('PWM (cuentas)'); ylabel('Voltaje (V)');
title('Ajuste PWM \rightarrow Voltaje — Ordenes 1 a 5');
legend('Location', 'northwest'); hold off;

%% 4. Voltaje → RPM
figure(3); hold on; grid on;
plot(voltaje, rpm, 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Datos');
vr_vec = linspace(min(voltaje), max(voltaje), 500)';

fprintf('\n=== Ajuste Voltaje → RPM ===\n');
for k = ordenes
    tipo = sprintf('poly%d', k);
    [f, gof] = fit(voltaje, rpm, tipo);
    plot(vr_vec, feval(f, vr_vec), colores{k}, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Orden %d  R²=%.4f', k, gof.rsquare));
    fprintf('  Orden %d: R² = %.6f\n', k, gof.rsquare);
end
xlabel('Voltaje (V)'); ylabel('RPM');
title('Ajuste Voltaje \rightarrow RPM — Ordenes 1 a 5');
legend('Location', 'northwest'); hold off;
