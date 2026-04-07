clear all
close all


% Parametros del motor dc
Ra = 4.04;      % Ohms
La = 0.00255;   % H
Km = 0.1735;    % N.m/A
Tm = 0.0281;    % N.m
Bm = 1.189e-3;  % N.m.s/rad
Jm = 7.20e-4;   % Kg.m^2

K = Km / ((Km)^2 + Ra*Bm);
tau = (Ra*Jm) / ((Km)^2 + Ra*Bm);

% Modelo simplificado estandar
numM = K;
denM = [tau 1];
G_m = tf(numM,denM);


% % Modelo completo de 2do orden
% numM2 = Km;
% denM2 = [La*Jm (Ra*Jm + La*Bm) (Ra*Bm + Km^2)];
% G_m2 = tf(numM2,denM2);

%step(G_m)
%grid on;


% Controlador PID Astrom. Para este caso solo PI
Kpi = 2.4972; Tii = 0.05542; Tdi = 0; Ni = 1;
numCM = Kpi*[Tii*Tdi, (Tii + Tdi/Ni), 1];
denCM = [Tii*Tdi/Ni, Tii, 0];
pi_motor = tf(numCM, denCM);

%rlocus(pi_motor*G_m)

bloque_I = feedback(pi_motor*G_m,1);
step(bloque_I)
grid on;


% --- PARAMETROS DE DISCRETIZACION ---
T = 1/1000;

% 1. Discretizacion de la Planta del Motor (ZOH)
plantaMD = c2d(G_m,T,'zoh');
[numMD, denMD] = tfdata(plantaMD,'v');
% Coeficientes de la planta (Primer orden: y[k] = b0*u[k-1] - a1*y[k-1])
% Nota: En primer orden, numD suele ser [0 b0] y denD [1 a1]
b1_p = numMD(2); 
a1_p = denMD(2);

% 2. Discretizacion del Controlador PI (Tustin)
C_piD = c2d(pi_motor,T,'tustin');
[numCD, denCD] = tfdata(C_piD,'v');
% Coeficientes del controlador PI digital
% u[k] = -c1*u[k-1] + d0*e[k] + d1*e[k-1]
d0 = numCD(1);
d1 = numCD(2);
c1 = denCD(2);


% --- SIMULACION DISCRETA ---
td = 0:T:2.0; % 2 segundos de simulacion
n = length(td);
ud = zeros(1, n); % Voltaje / PWM
yd = zeros(1, n); % Velocidad angular (rad/s)
ed = zeros(1, n); % Error de velocidad
limite = 1264; % PWM
ref = 20.0; % Referencia de velocidad (escalon unitario)

% --- BUCLE DE CONTROL ---
for k = 2:n
    % 1. Error de velocidad
    % Agregamos un ruido de lectura de encoder
    ruido = (rand() - 0.5) * 0.01; 
    ed(k) = ref - (yd(k-1) + ruido);
    
    % 2. Ecuacion en diferencias del controlador PI
    % Basado en la discretizacion Tustin
    ud_calc = -c1*ud(k-1) + d0*ed(k) + d1*ed(k-1);
    
    % --- SATURACION (Limite del Puente H / PWM) ---
    % Suponiendo que tu PWM llega hasta 1000 (o 12V representados en escala) 
    if ud_calc > limite
       ud(k) = limite;
    elseif ud_calc < -limite
       ud(k) = -limite;
    else
       ud(k) = ud_calc;
    end
    
    % 3. Evolucion de la planta (Fisica del motor)
    % Ecuacion: yd[k] = b1_p * ud[k-1] - a1_p * yd[k-1]
    yd(k) = b1_p * ud(k-1) - a1_p * yd(k-1);
    
    % --- PERTURBACION (Friccion extra o golpe) ---
    if abs(td(k) - 1.0) < 0.005
        yd(k) = yd(k) - 0.2; % Frenamos el motor un instante
    end
end

% --- GRAFICOS ---
figure(4)
subplot(2,1,1)
plot(td, yd, 'b', 'LineWidth', 2); hold on;
line([td(1) td(end)], [ref ref], 'Color', 'r', 'LineStyle', '--');
title('Velocidad Angular del Brazo (Lazo Interno Discreto)');
ylabel('Velocidad [rad/s]'); grid on;

subplot(2,1,2)
stairs(td, ud, 'g', 'LineWidth', 1.5);
title('Esfuerzo de Control (PWM enviado al Motor)');
ylabel('u[k]'); xlabel('Tiempo [s]'); grid on;

