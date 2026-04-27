clear all
close all


% Parametros del Pendulo Invertido
M = 0.288;  % Kg
m = 0.0935; % Kg
l = 0.175;  % m
g = 9.8;    % m/s^2


% Planta
num = -1/(M*l);
den = [1 0 -((M+m)*g/(M*l))];
planta = tf(num,den);

% step(planta)
% grid on;


% Tiempo de muestreo T
T = 0.005;  % s

plantaD = c2d(planta,T,'zoh');
[numD, denD] = tfdata(plantaD,'v');
% Coeficientes de la planta discretizada
b0 = numD(1);
b1 = numD(2);
b2 = numD(3);
a0 = denD(1);
a1 = denD(2);
a2 = denD(3);

% rlocus(plantaD)
% axis([-2 2 -1.2 1.2]);
% zgrid

Kp = -35; Ti = 0.0955; Td = 0.065; N = 10;
% C(s) = Kp * (1 + 1/(Ti*s) + (Td*s)/(1 + (Td/N)*s))
numC = Kp*[(Ti*Td*(N+1)/N), (Ti + Td/N), 1];
denC = [Ti*Td/N, Ti, 0];
pid_planta = tf(numC, denC);
C_z_pend = c2d(pid_planta,T,'tustin');
[numCD, denCD] = tfdata(C_z_pend, 'v');
d0 = numCD(1);
d1 = numCD(2);
d2 = numCD(3);
c0 = denCD(1);
c1 = denCD(2);
c2 = denCD(3);

cloop = feedback(C_z_pend*plantaD,1);

rlocus(C_z_pend*plantaD)
axis([-2 2 -1.2 1.2]);
zgrid

figure;
step(cloop)

% integrador = tf([1 0],[1 -1],T);
% 
% rlocus(integrador*plantaD)
% axis([-2 2 -1.2 1.2]);
% zgrid
% 
% der = tf([1 -0.942],[1 -0.572],T);
% 
% ganancia = 0.59548;
% 
% rlocus(integrador*der*plantaMD)
% zgrid

% --- SIMULACION DISCRETA ---
td = 0:T:2.0; % 2 segundos de simulacion
n = length(td);
ud = zeros(1, n); % velocidad angular (motor) [rad/s]
yd = zeros(1, n); % posicion angular (pendulo) [rad]
ed = zeros(1, n); % Error de posicion
limite = 50; % velocidad del motor
ref = 0.09; % Referencia de posicion del pendulo


for k = 3:n
    % Error de posicion (en el lazo externo)
    ed(k) = ref - yd(k-1);
    
    % Ecuacion en diferencias del controlador
    % Genera la velocidad angular requerida para el motor (ud)
    ud_calc = -c1*ud(k-1) - c2*ud(k-2) + d0*ed(k) + d1*ed(k-1) + d2*ed(k-2);
    
    % --- SATURACION DE VELOCIDAD ---
    if ud_calc > limite
       ud(k) = limite;
    elseif ud_calc < -limite
       ud(k) = -limite;
    else
       ud(k) = ud_calc;
    end
    
    % Evolucion del Pendulo (planta fisica)
    yd(k) = b1*ud(k-1) + b2*ud(k-2) - a1*yd(k-1) - a2*yd(k-2);
end


% --- GRAFICOS DE LA SIMULACION ---
figure('Name', 'Simulación Lazo Externo: Péndulo Invertido');

% Subplot 1: Posición Angular (yd)
subplot(2,1,1)
plot(td, yd, 'b', 'LineWidth', 2); hold on;
line([td(1) td(end)], [ref ref], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
title(['Respuesta del Péndulo (Referencia = ', num2str(ref), ' rad)']);
ylabel('Ángulo [rad]'); 
grid on;
legend('Ángulo real (\theta)', 'Referencia (\theta_r)', 'Location', 'best');

% Subplot 2: Esfuerzo de Control (ud - Velocidad enviada al lazo interno)
subplot(2,1,2)
stairs(td, ud, 'Color', [0 0.5 0], 'LineWidth', 1.5);
title('Referencia de Velocidad para el Motor (\omega_r)');
ylabel('Velocidad [rad/s]'); 
xlabel('Tiempo [s]'); 
grid on;
legend('Acción de Control (u[k])', 'Location', 'best');


