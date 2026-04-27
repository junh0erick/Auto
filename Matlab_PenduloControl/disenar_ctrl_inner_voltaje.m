% disenar_ctrl_inner_voltaje.m
% Diseño de controlador PI – inner loop (velocidad angular → Voltios)
%
% Planta en dominio Voltios: G(s) = K_m / (tau_m·s + 1)  [rad/s / V]
% Controlador PI con cancelación polo-cero: C(s) = Kp·(Ti·s+1)/(Ti·s)
% Salida del controlador en [V] — PSoC aplica PWM_Desde_Voltaje(u) al motor
% Referencia en [rad/s], realimentación del encoder en [rad/s]
%
% Uso en pendulo_gui2:
%   1. Ejecutar este script → C_z queda en workspace
%   2. Configurar Ctrl → Planta 1 → Modo: TF → campo TF: C_z
%   3. ☑ "Salida en [V] (ref rad/s)"   (NO marcar "Ref en Voltios")
%   4. u_min = -12  /  u_max = 12  [V]
%   5. Referencia en [rad/s]

clear; clc; close all;

% ── Ajustar aquí ─────────────────────────────────────────────────────────
wc_target = 60;      % [rad/s]  frecuencia de cruce deseada (probar 10–60)
Ts        = 5e-3;    % [s]      periodo de muestreo del PSoC
% ─────────────────────────────────────────────────────────────────────────

%% ── Parámetros del motor (dominio físico – Motor_3) ─────────────────────
K_m   = 4.61969587;   % [rad/s / V]  ganancia estática del motor
tau_m = 0.08329984;   % [s]          constante de tiempo mecánica

% Si ya tiene Motor_4.m regenerado (fine-tuned), descomante:
% run('Motor_4.m');  K_m = K_1 * 1264/12;  tau_m = tau_1;

%% ── Planta continua ──────────────────────────────────────────────────────
G_volt = tf(K_m, [tau_m 1]);

fprintf('Planta inner (Voltios):\n');
fprintf('  G(s) = %.5f / (%.5f·s + 1)   [rad/s / V]\n', K_m, tau_m);
fprintf('  Polo mecánico: wn = %.2f rad/s  (%.2f Hz)\n\n', ...
        1/tau_m, 1/(2*pi*tau_m));

%% ── Diseño PI: cancelación polo-cero ────────────────────────────────────
% C(s) = Kp · (Ti·s + 1) / (Ti·s)   con Ti = tau_m
% Lazo abierto tras cancelación: L(s) = Kp·K_m / (tau_m·s)  (integrador puro)
% Cruce de ganancia: |L(j·wc)| = 1  →  Kp = wc·tau_m / K_m

Ti = tau_m;
Kp = wc_target * Ti / K_m;
Ki = Kp / Ti;

fprintf('── Controlador PI continuo ──────────────────────────────\n');
fprintf('  Ti  = tau_m  = %.6f s\n',    Ti);
fprintf('  Kp  = %.6f  [V·s/rad]\n',    Kp);
fprintf('  Ki  = Kp/Ti = %.6f  [V/rad]\n', Ki);
fprintf('  wc_target   = %.1f rad/s  (fc = %.2f Hz)\n\n', ...
        wc_target, wc_target/(2*pi));

C_cont = tf(Kp * [Ti 1], [Ti 0]);

%% ── Discretización Tustin ────────────────────────────────────────────────
C_z = c2d(C_cont, Ts, 'tustin');
[num_z, den_z] = tfdata(C_z, 'v');

fprintf('── Controlador PI discreto (Tustin, Ts=%.0f ms) ─────────\n', Ts*1e3);
fprintf('  b = ['); fprintf(' %+.8f', num_z); fprintf(' ]\n');
fprintf('  a = ['); fprintf(' %+.8f', den_z); fprintf(' ]\n\n');

%% ── Márgenes de estabilidad ──────────────────────────────────────────────
L_disc = C_z * c2d(G_volt, Ts, 'zoh');
[Gm, Pm, Wgm, Wpm] = margin(L_disc);

fprintf('── Márgenes (lazo discreto) ─────────────────────────────\n');
if isinf(Gm)
    fprintf('  Margen de ganancia: ∞ dB  (integrador – ningún cruce de -180°)\n');
else
    fprintf('  Margen de ganancia: %.1f dB @ %.1f rad/s\n', 20*log10(Gm), Wgm);
end
fprintf('  Margen de fase:     %.1f °  @ %.1f rad/s\n\n', Pm, Wpm);

%% ── Figura 1 – Bode del lazo abierto ─────────────────────────────────────
figure('Name','Lazo abierto – Bode','NumberTitle','off');
bodeopt = bodeoptions;
bodeopt.FreqUnits = 'rad/s';
bode(C_cont * G_volt, bodeopt); grid on; hold on;
bode(L_disc, bodeopt);
legend('L(s) continuo','L(z) discreto','Location','southwest');
title(sprintf('Bode lazo abierto — wc = %.0f rad/s, Ts = %.0f ms', ...
      wc_target, Ts*1e3));

%% ── Figura 2 – Respuesta al escalón en lazo cerrado ──────────────────────
CL_cont = feedback(C_cont * G_volt,  1);
CL_disc = feedback(L_disc, 1);

figure('Name','Escalón lazo cerrado','NumberTitle','off');
subplot(2,1,1);
[y_c, t_c] = step(CL_cont, 0.5);
plot(t_c*1e3, y_c, 'b', 'LineWidth', 1.5); grid on;
xlabel('t [ms]'); ylabel('\omega / \omega_{ref}');
title('Continuo');
yline(1,'--r','Referencia');

subplot(2,1,2);
[y_d, t_d] = step(CL_disc, 0.5);
plot(t_d*1e3, y_d, 'b.-', 'LineWidth', 1.2); grid on;
xlabel('t [ms]'); ylabel('\omega / \omega_{ref}');
title(sprintf('Discreto (Ts=%.0f ms)',Ts*1e3));
yline(1,'--r','Referencia');
sgtitle(sprintf('Lazo cerrado — wc=%.0f rad/s  /  Kp=%.4f  Ki=%.4f', ...
        wc_target, Kp, Ki));

%% ── Figura 3 – Sensibilidad + saturación ────────────────────────────────
S_disc  = feedback(1, L_disc);   % sensibilidad
T_disc  = feedback(L_disc, 1);   % complementaria
CS_disc = C_z * S_disc;          % esfuerzo de control

figure('Name','Sensibilidad','NumberTitle','off');
subplot(2,1,1);
bodemag(S_disc, T_disc); grid on;
legend('S(z)','T(z)','Location','southeast');
title('Sensibilidad y sensibilidad complementaria');

subplot(2,1,2);
bodemag(CS_disc); grid on;
yline(20*log10(12), '--r', '12 V');   % límite de saturación a 12 V
title('Esfuerzo de control CS(z)  [dB re 1 V/rad·s^{-1}]');
ylabel('Magnitud [dB]'); xlabel('Frecuencia [rad/s]');

%% ── Exportar al workspace ────────────────────────────────────────────────
assignin('base', 'C_z',     C_z);
assignin('base', 'C_cont',  C_cont);
assignin('base', 'G_volt',  G_volt);
assignin('base', 'CL_disc', CL_disc);
assignin('base', 'Kp_inner', Kp);
assignin('base', 'Ki_inner', Ki);

fprintf('Variables en workspace: C_z, C_cont, G_volt, CL_disc, Kp_inner, Ki_inner\n\n');

%% ── Instrucciones GUI ────────────────────────────────────────────────────
fprintf('══════════════════════════════════════════════════════════\n');
fprintf('PASOS – pendulo_gui2  (modo "Salida en Voltios")\n');
fprintf('  1. Configurar Ctrl  →  Planta 1 (inner)\n');
fprintf('  2. Modo: TF\n');
fprintf('  3. Campo TF: escribir   C_z\n');
fprintf('  4. Marcar ☑ "Salida en [V] (ref rad/s)"\n');
fprintf('        (NO marcar "Ref en Voltios")\n');
fprintf('  5. u_min = -12  /  u_max = 12   [V]\n');
fprintf('  6. Referencia en [rad/s]  (ej. 50 rad/s ≈ 478 RPM)\n');
fprintf('  7. Aplicar → Iniciar\n');
fprintf('──────────────────────────────────────────────────────────\n');
fprintf('Para ajustar ancho de banda: cambie wc_target y reeje.\n');
fprintf('  wc =  10 rad/s → más lento, más robusto\n');
fprintf('  wc =  30 rad/s → compromiso habitual\n');
fprintf('  wc =  60 rad/s → rápido, Ts comienza a limitar\n');
fprintf('══════════════════════════════════════════════════════════\n');
