% Motor_4.m — Modelo fine-tuned contra datos reales
% Generado automáticamente por motor_finetune.m
% Fuente de datos: D:\PSoc\Auto\TPF\TPF_PenduloInvertido\Matlab_PenduloControl\pruebaStepsRapidos_filtrado.mat
% Fecha: 2026-04-13 19:39:52
% -------------------------------------------------------------------------
% NOTA SOBRE UNIDADES:
%   La entrada del modelo identificado es u1 [PWM], igual que el PSoC.
%   La salida es ω [rad/s].
%   El parámetro K absorbe la conversión PWM→Voltaje.
% -------------------------------------------------------------------------

clear all
close all

% ===========================================================================
% MODELO DE 1er ORDEN  G(s) = K / (tau·s + 1)
% ===========================================================================
K_1   = 0.04859924;   % [rad/s / PWM]
tau_1 = 0.28084528;   % [s]  (280.85 ms)

G1 = tf(K_1, [tau_1, 1]);

% ===========================================================================
% MODELO DE 2do ORDEN  G(s) = b0 / (a2·s² + a1·s + a0)
% ===========================================================================
b0_2 = 83241.62893075;
a2_2 = 1.00000000;
a1_2 = 480948.87280775;
a0_2 = 1712828.81438917;

wn_2   = sqrt(a0_2/a2_2);          % 1308.7509 rad/s
zeta_2 = a1_2/(2*sqrt(a2_2*a0_2)); % 183.7435

G2 = tf(b0_2, [a2_2, a1_2, a0_2]);

% ===========================================================================
% PARÁMETROS ORIGINALES (Motor_3) — referencia
% ===========================================================================
Ra = 4.04000;  La = 0.002550;  Km = 0.18630;
Bm = 1.3910e-03;  Jm = 8.3150e-04;
K_m3   = 4.61969587;  tau_m3 = 0.08329984;

% ===========================================================================
% DISCRETIZACIÓN (ZOH)
% ===========================================================================
T = 0.005000;  % Ts de los datos de identificación

G1_d = c2d(G1, T, 'zoh');
G2_d = c2d(G2, T, 'zoh');

[num1d, den1d] = tfdata(G1_d, 'v');
[num2d, den2d] = tfdata(G2_d, 'v');

% Coeficientes para ecuación en diferencias:
b1_1 = num1d(2);  a1_1 = den1d(2);  % 1er orden
b1_2 = num2d(2);  b2_2 = num2d(3);  % 2do orden
a1_2d = den2d(2); a2_2d = den2d(3);

% ===========================================================================
% CONTROLADOR PI (mismo que Motor_3 — rediseñar si es necesario)
% ===========================================================================
Kp = 0.5748;
Ti = 0.0695;
C_s = tf(Kp * [Ti 1], [Ti 0]);
C_z1 = c2d(C_s, T, 'tustin');

cloop1 = feedback(C_z1 * G1_d, 1);
cloop2 = feedback(C_z1 * G2_d, 1);

% ===========================================================================
% GRÁFICAS
% ===========================================================================
figure;
step(cloop1, cloop2);
legend('Lazo cerrado 1er orden','Lazo cerrado 2do orden');
title('Respuesta al escalón — Motor\_4 fine-tuned');
grid on;

figure;
bode(G1, G2);
legend('G1 (1er orden)','G2 (2do orden)');
title('Bode planta — Motor\_4 fine-tuned');
grid on;

rlocus(C_z1 * G2_d);
zgrid;
title('Lugar de raíces — Motor\_4 (2do orden)');
