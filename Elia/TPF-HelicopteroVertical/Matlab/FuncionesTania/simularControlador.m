close all
clc


Ts = 1/1000;
sysD = c2d(sysC,Ts,'zoh');
%% PID
Kp = 2.5;Ti = 5;Td = 0.1; N = 3;
numCA = Kp*[Ti*Td, (Ti + Td/N), 1]; % Ojo con la forma exacta de tu numCA
denCA = [Ti*Td/N, Ti, 0];
controlador_Ams = tf(numCA, denCA);
C = c2d(controlador_Ams,Ts,'tustin');

%% Lugar de raices

% C = tf(-0.017346322429932.*conv([1,-1.014028439216919],[1,-0.5]),conv([1,-0.952242280789835],[1,-0.989400000000000]),Ts);
%% Graficos
C
zpk(C)
figure;bode(C*sysD); grid on;grid minor;
figure;rlocus(C*sysD); zgrid;
figure;step(feedback(C*sysD,1));grid on; grid minor;
figure;step(feedback(C,sysD));grid on;grid minor;

L = minreal(C*sysD);          % lazo abierto
T = minreal(feedback(L,1));   % lazo cerrado (r->y)
format long;
poles_CL = pole(T)