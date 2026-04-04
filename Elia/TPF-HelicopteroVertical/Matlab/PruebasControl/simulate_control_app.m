clear all
close all
clc
tau =1e6*2e-9;   % rad/s
s  = tf('s');
G  = 1/(tau*s + 1); % DC gain = 1, pole = -wp

Ts = 1/1e3;
Gd = c2d(G, Ts, 'zoh');

controlSystemDesigner(Gd)
