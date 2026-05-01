%% ========================================
%  SIMULACIÓN CON FRECUENCIA ÚNICA: 400 Hz
%  Lazo interno (motor): 400 Hz
%  Lazo externo (péndulo): 400 Hz
% ========================================

clear all; close all; clc;

%% ========================================
%  PARÁMETROS DE MUESTREO
% ========================================

fprintf('========================================\n');
fprintf('  CONFIGURACIÓN: FRECUENCIA ÚNICA\n');
fprintf('========================================\n\n');

Fs = 400;         % Frecuencia única [Hz]
Ts = 1/Fs;        % Período único [s]

fprintf('Ambos lazos a la misma frecuencia:\n');
fprintf('  Fs = %.0f Hz  (Ts = %.4f s = %.2f ms)\n\n', Fs, Ts, Ts*1000);

fprintf('Ventajas:\n');
fprintf('  ✅ Código más simple (un solo timer)\n');
fprintf('  ✅ Péndulo se actualiza 2× más frecuente\n');
fprintf('  ✅ Detección de caída más rápida\n');
fprintf('  ✅ Sincronización perfecta\n\n');

%% ========================================
%  LAZO INTERNO: MODELO DEL MOTOR
% ========================================

fprintf('========================================\n');
fprintf('  LAZO INTERNO: MOTOR DC\n');
fprintf('========================================\n\n');

% Parámetros del motor
Ra = 4.04;
La = 0.00255;
Km = 0.1863;
Bm = 1.391e-3;
Jm = 8.315e-4;

% Modelo continuo del motor
numM = Km;
denM = [La*Jm, (Ra*Jm + La*Bm), (Ra*Bm + Km^2)];
G_motor_s = tf(numM, denM);

% Discretizar a 400 Hz
G_motor_z = c2d(G_motor_s, Ts, 'zoh');
[numM_z, denM_z] = tfdata(G_motor_z, 'v');

% Normalizar
b_motor = numM_z / denM_z(1);
a_motor = denM_z / denM_z(1);

fprintf('Planta del motor discretizada (400 Hz):\n');
fprintf('  b0 = %.10f\n', b_motor(1));
fprintf('  b1 = %.10f\n', b_motor(2));
fprintf('  b2 = %.10f\n', b_motor(3));
fprintf('  a1 = %.10f\n', a_motor(2));
fprintf('  a2 = %.10f\n\n', a_motor(3));

%% ========================================
%  LAZO INTERNO: CONTROLADOR PI DEL MOTOR
% ========================================

% Parámetros del PI
Kp_motor = 5;
Ti_motor = 0.1;

% PI continuo
C_motor_s = tf(Kp_motor * [Ti_motor, 1], [Ti_motor, 0]);

% Discretizar a 400 Hz
C_motor_z = c2d(C_motor_s, Ts, 'tustin');
[numC_motor, denC_motor] = tfdata(C_motor_z, 'v');

% Normalizar
b_ctrl_motor = numC_motor / denC_motor(1);
a_ctrl_motor = denC_motor / denC_motor(1);

d0_motor = b_ctrl_motor(1);
d1_motor = b_ctrl_motor(2);
c1_motor = a_ctrl_motor(2);

fprintf('Controlador PI del motor:\n');
fprintf('  Kp = %.1f\n', Kp_motor);
fprintf('  Ti = %.3f s\n\n', Ti_motor);

fprintf('Coeficientes discretos (400 Hz):\n');
fprintf('  d0 = %.10f\n', d0_motor);
fprintf('  d1 = %.10f\n', d1_motor);
fprintf('  c1 = %.10f\n\n', c1_motor);

%% ========================================
%  LAZO EXTERNO: MODELO DEL PÉNDULO
% ========================================

fprintf('========================================\n');
fprintf('  LAZO EXTERNO: PÉNDULO INVERTIDO\n');
fprintf('========================================\n\n');

% Parámetros físicos del péndulo
M = 0.288;   % Masa del brazo [kg]
m = 0.0935;  % Masa del péndulo [kg]
l = 0.175;   % Longitud al centro de masa [m]
g = 9.8;     % Gravedad [m/s²]

fprintf('Parámetros físicos:\n');
fprintf('  M = %.3f kg\n', M);
fprintf('  m = %.3f kg\n', m);
fprintf('  l = %.3f m\n', l);
fprintf('  g = %.1f m/s²\n\n', g);

% Planta continua del péndulo
num_pend = -1/(M*l);
den_pend = [1, 0, -((M+m)*g/(M*l))];
G_pendulo_s = tf(num_pend, den_pend);

% Discretizar a 400 Hz (AHORA A LA MISMA FRECUENCIA)
G_pendulo_z = c2d(G_pendulo_s, Ts, 'zoh');
[numP_z, denP_z] = tfdata(G_pendulo_z, 'v');

% Normalizar
b_pendulo = numP_z / denP_z(1);
a_pendulo = denP_z / denP_z(1);

fprintf('Planta del péndulo discretizada (400 Hz):\n');
fprintf('  b0 = %.10f\n', b_pendulo(1));
fprintf('  b1 = %.10f\n', b_pendulo(2));
fprintf('  b2 = %.10f\n', b_pendulo(3));
fprintf('  a1 = %.10f\n', a_pendulo(2));
fprintf('  a2 = %.10f\n\n', a_pendulo(3));

%% ========================================
%  LAZO EXTERNO: CONTROLADOR PID DEL PÉNDULO
% ========================================

% Parámetros del PID
Kp_pend = -35;
Ti_pend = 0.0955;
Td_pend = 0.065;
N_pend = 10;

fprintf('Controlador PID del péndulo:\n');
fprintf('  Kp = %.1f\n', Kp_pend);
fprintf('  Ti = %.4f s\n', Ti_pend);
fprintf('  Td = %.4f s\n', Td_pend);
fprintf('  N = %d\n\n', N_pend);

% PID continuo
numC_pend = Kp_pend * [(Ti_pend*Td_pend*(N_pend+1)/N_pend), (Ti_pend + Td_pend/N_pend), 1];
denC_pend = [Td_pend*Ti_pend/N_pend, Ti_pend, 0];
C_pendulo_s = tf(numC_pend, denC_pend);

% Discretizar a 400 Hz (AHORA A LA MISMA FRECUENCIA)
C_pendulo_z = c2d(C_pendulo_s, Ts, 'tustin');
[numC_pend_z, denC_pend_z] = tfdata(C_pendulo_z, 'v');

% Normalizar
b_ctrl_pend = numC_pend_z / denC_pend_z(1);
a_ctrl_pend = denC_pend_z / denC_pend_z(1);

d0_pend = b_ctrl_pend(1);
d1_pend = b_ctrl_pend(2);
d2_pend = b_ctrl_pend(3);
c1_pend = a_ctrl_pend(2);
c2_pend = a_ctrl_pend(3);

fprintf('Coeficientes discretos (400 Hz):\n');
fprintf('  d0 = %.10f\n', d0_pend);
fprintf('  d1 = %.10f\n', d1_pend);
fprintf('  d2 = %.10f\n', d2_pend);
fprintf('  c1 = %.10f\n', c1_pend);
fprintf('  c2 = %.10f\n\n', c2_pend);

%% ========================================
%  CÓDIGO C PARA PSoC
% ========================================

fprintf('========================================\n');
fprintf('  CÓDIGO C PARA PSoC (FRECUENCIA ÚNICA)\n');
fprintf('========================================\n\n');

fprintf('/* ================================================\n');
fprintf(' * CONTROL EN CASCADA - FRECUENCIA ÚNICA\n');
fprintf(' * Ambos lazos: 400 Hz (cada 2.5 ms)\n');
fprintf(' * Timer único: isr_1 ejecuta ambos controles\n');
fprintf(' * ================================================ */\n\n');

fprintf('/* --- LAZO INTERNO (MOTOR) --- */\n');
fprintf('static const float d0_motor = %.10ff;\n', d0_motor);
fprintf('static const float d1_motor = %.10ff;\n', d1_motor);
fprintf('static const float c1_motor = %.10ff;\n\n', c1_motor);

fprintf('static float e_motor[2] = {0.0f, 0.0f};  // e[k], e[k-1]\n');
fprintf('static float u_motor[2] = {0.0f, 0.0f};  // u[k], u[k-1]\n\n');

fprintf('/* --- LAZO EXTERNO (PÉNDULO) --- */\n');
fprintf('static const float d0_pend = %.10ff;\n', d0_pend);
fprintf('static const float d1_pend = %.10ff;\n', d1_pend);
fprintf('static const float d2_pend = %.10ff;\n', d2_pend);
fprintf('static const float c1_pend = %.10ff;\n', c1_pend);
fprintf('static const float c2_pend = %.10ff;\n\n', c2_pend);

fprintf('static float e_pend[3] = {0.0f, 0.0f, 0.0f};  // e[k], e[k-1], e[k-2]\n');
fprintf('static float vel_ref[3] = {0.0f, 0.0f, 0.0f};  // vel_ref[k], [k-1], [k-2]\n\n');

fprintf('/* ISR - Timer a 400 Hz (2.5 ms) */\n');
fprintf('CY_ISR(isr_control_cascade)\n');
fprintf('{\n');
fprintf('    /* 1. Leer sensores */\n');
fprintf('    int32_t ticks_motor = QuadDec_1_GetCounter();\n');
fprintf('    int32_t ticks_pendulo = QuadDec_2_GetCounter();\n');
fprintf('    \n');
fprintf('    float vel_motor = calcular_velocidad(ticks_motor);  // Tu función\n');
fprintf('    float angulo_pend = calcular_angulo(ticks_pendulo); // Tu función\n');
fprintf('    \n');
fprintf('    /* 2. Control del péndulo (lazo externo) */\n');
fprintf('    float angulo_ref = 3.14159265f;  // π rad\n');
fprintf('    e_pend[0] = angulo_ref - angulo_pend;\n');
fprintf('    \n');
fprintf('    float vel_ref_calc = d0_pend * e_pend[0] + d1_pend * e_pend[1] + d2_pend * e_pend[2]\n');
fprintf('                       - c1_pend * vel_ref[1] - c2_pend * vel_ref[2];\n');
fprintf('    \n');
fprintf('    // Saturar velocidad\n');
fprintf('    if (vel_ref_calc > 20.0f) vel_ref_calc = 20.0f;\n');
fprintf('    if (vel_ref_calc < -20.0f) vel_ref_calc = -20.0f;\n');
fprintf('    \n');
fprintf('    vel_ref[0] = vel_ref_calc;\n');
fprintf('    \n');
fprintf('    /* 3. Control del motor (lazo interno) */\n');
fprintf('    e_motor[0] = vel_ref_calc - vel_motor;\n');
fprintf('    \n');
fprintf('    float pwm = -c1_motor * u_motor[1] + d0_motor * e_motor[0] + d1_motor * e_motor[1];\n');
fprintf('    \n');
fprintf('    // Saturar PWM\n');
fprintf('    if (pwm > 1264.0f) pwm = 1264.0f;\n');
fprintf('    if (pwm < -1264.0f) pwm = -1264.0f;\n');
fprintf('    \n');
fprintf('    u_motor[0] = pwm;\n');
fprintf('    \n');
fprintf('    /* 4. Aplicar PWM */\n');
fprintf('    Aplicar_PWM((int16_t)pwm);  // Tu función\n');
fprintf('    \n');
fprintf('    /* 5. Actualizar estados (shift) */\n');
fprintf('    e_pend[2] = e_pend[1];\n');
fprintf('    e_pend[1] = e_pend[0];\n');
fprintf('    vel_ref[2] = vel_ref[1];\n');
fprintf('    vel_ref[1] = vel_ref[0];\n');
fprintf('    \n');
fprintf('    e_motor[1] = e_motor[0];\n');
fprintf('    u_motor[1] = u_motor[0];\n');
fprintf('}\n\n');

fprintf('========================================\n\n');

%% ========================================
%  SIMULACIÓN
% ========================================

fprintf('========================================\n');
fprintf('  SIMULACIÓN A 400 Hz\n');
fprintf('========================================\n\n');

% Tiempo de simulación
t_sim = 3.0;
t = 0:Ts:t_sim;
n = length(t);

% Variables
vel_motor = zeros(1, n);
vel_ref_motor = zeros(1, n);
pwm = zeros(1, n);
e_motor_vec = zeros(1, n);

angulo_pendulo = zeros(1, n);
angulo_ref = 0.09;  % rad
e_pend_vec = zeros(1, n);

% Estados
e_m = [0, 0];
u_m = [0, 0];
y_m = [0, 0];
u_m_plant = [0, 0];

e_p = [0, 0, 0];
vel_ref_p = [0, 0, 0];
y_p = [0, 0];
u_p_plant = [0, 0];

% Límites
PWM_MAX = 1264;
VEL_MAX = 20.0;

fprintf('Simulando %.1f segundos a %.0f Hz...\n\n', t_sim, Fs);

for k = 3:n
    % ========================================
    % LAZO EXTERNO (PÉNDULO) - cada ciclo
    % ========================================
    
    theta_actual = angulo_pendulo(k-1);
    e_p(1) = angulo_ref - theta_actual;
    e_pend_vec(k) = e_p(1);
    
    % PID del péndulo
    vel_ref_calc = d0_pend * e_p(1) + d1_pend * e_p(2) + d2_pend * e_p(3) ...
                 - c1_pend * vel_ref_p(2) - c2_pend * vel_ref_p(3);
    
    % Saturación
    if vel_ref_calc > VEL_MAX
        vel_ref_calc = VEL_MAX;
    elseif vel_ref_calc < -VEL_MAX
        vel_ref_calc = -VEL_MAX;
    end
    
    vel_ref_p(1) = vel_ref_calc;
    vel_ref_motor(k) = vel_ref_calc;
    
    % ========================================
    % LAZO INTERNO (MOTOR) - cada ciclo
    % ========================================
    
    vel_actual = vel_motor(k-1);
    e_m(1) = vel_ref_motor(k) - vel_actual;
    e_motor_vec(k) = e_m(1);
    
    % PI del motor
    pwm_calc = -c1_motor * u_m(2) + d0_motor * e_m(1) + d1_motor * e_m(2);
    
    % Saturación
    if pwm_calc > PWM_MAX
        pwm_calc = PWM_MAX;
    elseif pwm_calc < -PWM_MAX
        pwm_calc = -PWM_MAX;
    end
    
    pwm(k) = pwm_calc;
    u_m(1) = pwm_calc;
    
    % Planta del motor
    vel_motor(k) = b_motor(2) * u_m_plant(1) + b_motor(3) * u_m_plant(2) ...
                 - a_motor(2) * y_m(1) - a_motor(3) * y_m(2);
    
    % ========================================
    % PLANTA DEL PÉNDULO - cada ciclo
    % ========================================
    
    angulo_pendulo(k) = b_pendulo(2) * u_p_plant(1) + b_pendulo(3) * u_p_plant(2) ...
                      - a_pendulo(2) * y_p(1) - a_pendulo(3) * y_p(2);
    
    % Perturbación
    if abs(t(k) - 1.5) < 0.001
        angulo_pendulo(k) = angulo_pendulo(k) + 0.05;
    end
    
    % Actualizar estados
    e_p(3) = e_p(2);
    e_p(2) = e_p(1);
    vel_ref_p(3) = vel_ref_p(2);
    vel_ref_p(2) = vel_ref_p(1);
    
    e_m(2) = e_m(1);
    u_m(2) = u_m(1);
    u_m_plant(2) = u_m_plant(1);
    u_m_plant(1) = pwm(k);
    y_m(2) = y_m(1);
    y_m(1) = vel_motor(k);
    
    u_p_plant(2) = u_p_plant(1);
    u_p_plant(1) = vel_motor(k);
    y_p(2) = y_p(1);
    y_p(1) = angulo_pendulo(k);
end

% Métricas
error_ss_pend = abs(angulo_ref - mean(angulo_pendulo(end-100:end)));
error_ss_motor = abs(mean(vel_ref_motor(end-100:end)) - mean(vel_motor(end-100:end)));

fprintf('Métricas de desempeño:\n');
fprintf('  Error SS péndulo: %.6f rad (%.3f°)\n', error_ss_pend, error_ss_pend*180/pi);
fprintf('  Error SS motor:   %.6f rad/s\n', error_ss_motor);
fprintf('========================================\n\n');

%% GRÁFICOS
figure('Position', [100 100 1400 900]);

subplot(3,2,1);
plot(t, angulo_pendulo*180/pi, 'b', 'LineWidth', 2); hold on;
yline(angulo_ref*180/pi, 'r--', 'LineWidth', 1.5);
grid on;
title('Ángulo del Péndulo (400 Hz)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Ángulo [°]');

subplot(3,2,2);
plot(t, e_pend_vec*180/pi, 'r', 'LineWidth', 1.5);
grid on;
title('Error del Péndulo', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Error [°]');

subplot(3,2,3);
plot(t, vel_motor, 'b', 'LineWidth', 1.5); hold on;
plot(t, vel_ref_motor, 'r--', 'LineWidth', 1.5);
grid on;
title('Velocidad del Motor (400 Hz)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Velocidad [rad/s]');
legend('Real', 'Referencia', 'Location', 'best');

subplot(3,2,4);
plot(t, e_motor_vec, 'r', 'LineWidth', 1.5);
grid on;
title('Error del Motor', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Error [rad/s]');

subplot(3,2,5);
stairs(t, pwm, 'g', 'LineWidth', 1.5); hold on;
yline(PWM_MAX, 'r--', 'LineWidth', 1);
yline(-PWM_MAX, 'r--', 'LineWidth', 1);
grid on;
title('Señal PWM', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('PWM');

subplot(3,2,6);
stairs(t, vel_ref_motor, 'Color', [0.8 0.4 0], 'LineWidth', 1.5); hold on;
yline(VEL_MAX, 'r--', 'LineWidth', 1);
yline(-VEL_MAX, 'r--', 'LineWidth', 1);
grid on;
title('Referencia de Velocidad', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Tiempo [s]');
ylabel('Vel. Ref [rad/s]');

fprintf('✅ Simulación completada.\n\n');