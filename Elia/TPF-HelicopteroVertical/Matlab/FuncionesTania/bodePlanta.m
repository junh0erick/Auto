close all; clear; clc;

Ts = 1/1000;   % IMPORTANTE: poné el Ts real del diseño en CSD

A = [ 0.9972,      1,       0;
     -6.118e-06, 0.9972, -0.1187;
      0,          0,       1];

B = [0;
     0.01016;
     0.001206];

C = [-0.0001197, -0.01543, 0];

D = 0;

L = ss(A,B,C,D,Ts);   % LoopTransfer_C = L(z)

% (opcional) limpiar realizaciones numéricas
L = minreal(L, 1e-9);

% Lazo cerrado: T = L/(1+L)
T = feedback(L, 1);

% Bode unwrapped para que no te confunda el wrap de fase
opt = bodeoptions;
opt.PhaseWrapping = 'off';
opt.FreqUnits = 'rad/s';

w = logspace(0,4,2000);

figure; bodeplot(L, w, opt); grid on; grid minor;
title('Bode lazo abierto L(z) = LoopTransfer\_C');

figure; bodeplot(T, w, opt); grid on; grid minor;
title('Bode lazo cerrado T(z) = L/(1+L)');

% chequeos de sanidad
disp('Polos de L:');  disp(pole(L).');
disp('Polos de T:');  disp(pole(T).');
disp('¿T estable?');  disp(isstable(T));
