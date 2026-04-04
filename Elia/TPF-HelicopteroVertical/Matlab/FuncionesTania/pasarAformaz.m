close all; clear; clc;

S = load("D:\auto\TPF-HelicopteroVertical\Matlab\planta (1).mat");
whos('-file',"D:\auto\TPF-HelicopteroVertical\Matlab\planta (1).mat")

% Elegí la planta correcta según lo que exista
if isfield(S,'plantaC')
    plantaC = S.plantaC;
elseif isfield(S,'sysC')
    plantaC = S.sysC;   % si tu planta se llama sysC
elseif isfield(S,'G')
    plantaC = S.G;
else
    error("No encontré plantaC/sysC/G dentro del .mat. Mirá la salida de whos -file.");
end


Ts = 1/2;
[num, den] = tfdata(c2d(plantaC,Ts,'zoh'),'v');

n = length(num)-1;   % grado numerador
m = length(den)-1;   % grado denominador
k = max(n,m);

num_z = [num zeros(1,k-n)];
den_z = [den zeros(1,k-m)];

Hz = tf(num_z, den_z, Ts, 'Variable','z');
zpk(Hz)


z = tf([1 0],1,Ts);
a0 = Hz.Numerator{1}(4-0);
a1 = Hz.Numerator{1}(4-1);
a2 = Hz.Numerator{1}(4-2);

p1 = zpk(Hz).P{1}(2);
p2 = zpk(Hz).P{1}(3);
K = 1/(a0+a1+a2);
b1 = (a0+a1)/(a0+a1+a2);
b0 = a0/(a0+a1+a2);


C1 = K*(z-p1)*(z-p2)/(z^2+b1*z+b0);%Hz^-1*(1/(z^3-1));

figure(1);step(feedback(Hz*C1,1));hold on;
[ud,n] = step(feedback(C1,Hz));
uc = repelem(ud, 40);
t = (0:Ts/40:(n(end))*Ts);
dt = Ts/40;
t  = (0:length(uc)-1)' * dt;     % mismo largo que uc
[yc,t] = lsim(plantaC, uc, t);
plot(t, yc);
figure(2);
stairs(n,ud);

C2 = Hz^-1*(1/(z^3-1));

figure(3);step(feedback(Hz*C2,1));hold on;
[ud,n] = step(feedback(C2,Hz));
uc = repelem(ud, 40);
t = (0:Ts/40:(n(end))*Ts);
dt = Ts/40;
t  = (0:length(uc)-1)' * dt;     % mismo largo que uc
[yc,t] = lsim(plantaC, uc, t);
plot(t, yc);
figure(4);
stairs(n,ud);

