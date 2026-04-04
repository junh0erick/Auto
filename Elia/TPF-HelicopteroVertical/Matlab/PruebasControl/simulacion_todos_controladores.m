%% gen_sim_export_5ctrl_psoc.m
clear; clc; close all;

%% ===================== 0) POLOS (TOCÁ SOLO ESTO) =====================
p_ctrl = 0.60;   % polo deseado del lazo cerrado principal (REAL, |p|<1)
p_obs  = 0.30;   % polo del observador (más rápido => más chico)
p_int  = 0.50;   % polo extra para el aumentado con integrador (como tu Ogata)

%% ===================== 1) PLANTA (LA TUYA) =====================
tau = 1e6*2e-9;          % [s]
Ts  = 1/100e3;           % 100 kHz
s = tf('s');
G = 1/(tau*s + 1);

% Discretización ZOH
Gd = c2d(G, Ts, 'zoh'); %#ok<NASGU>

% Para coincidir con tu forma (B=1, C=1-A):
% si A = exp(-Ts/tau) => G(z)= (1-A)/(z-A)  (DC=1)
a = exp(-Ts/tau);
A1 = a;
B1 = 1;
C1 = 1 - a;
D1 = 0;

fprintf("a = exp(-Ts/tau) = %.9f\n", a);
fprintf("Planta 1-estado usada (forma PSoC): x+=A x + 1*u ; y=(1-A)x\n");
fprintf("A1=%.9f  B1=%.9f  C1=%.9f  D1=%.9f\n\n", A1,B1,C1,D1);

% Planta “rellena” a 2 estados (para tu firmware SS de 2 estados)
A = [A1 0; 0 0];
B = [B1; 0];
C = [C1 0];
D = D1;

%% ===================== 2) CONTROLADOR TF (el “óptimo” que pediste) =====================
% Deseo: T_des(z) = (1-p)/(z-p)  (ganancia DC=1, polo = p_ctrl)
z = tf('z', Ts);
Tdes = (1 - p_ctrl)/(z - p_ctrl);

% Planta discreta en z: Gz = (1-a)/(z-a)
Gz = (1 - a)/(z - a);

% C = Tdes / (Gz*(1-Tdes))
Cz = minreal( Tdes / (Gz * (1 - Tdes)) );

% Empaquetar Cz a DF2T (b0..b5 / a0..a5) en z^-1
COEFF_TF = pack_tf16_from_z(Cz);

%% ===================== 3) SS SIN integrador =====================
% K para colocar polo del estado: eig(A1 - B1*K) = p_ctrl
K = acker(A1, B1, p_ctrl);

% Nbar por dcgain (como pediste)
sys_cl = ss(A1 - B1*K, B1, C1, D1, Ts);  % r -> y con u = -Kx + Nbar r
gdc = dcgain(sys_cl);
Nbar = 1/gdc;  % esto es Kr en modo NO integrador

% Observadores
L_pred   = acker(A1', C1',      p_obs).';          % A - L C
L_actual = acker(A1', (C1*A1)', p_obs).';          % tu “actual” (equivale a usar C*A)

% Nuevo protocolo: c[13] (MATLAB c16(14)) = Kx
% En NO integrador: Kx = Kr = Nbar
COEFF_SS_PRED_NOI = pack_ss16_1state(A1,B1,C1,D1, L_pred,   K,    Nbar);
COEFF_SS_ACT_NOI  = pack_ss16_1state(A1,B1,C1,D1, L_actual, K,    Nbar);

%% ===================== 4) SS CON integrador (TU OGATA TAL CUAL) =====================
% OJO: tu firmware integra: vint += e*Ts
% Ogata (típico) asume v += e (sin Ts).
% Para equivalencia: Ki_psoc = K1/Ts
m = 1;
n = 1;
Ahat = [A1 B1; zeros(m,n+m)];
Bhat = [zeros(n,m); eye(m)];
Chat = [C1 zeros(1,m)]; %#ok<NASGU>

polos_i = [p_ctrl p_int];    % (n=1) => [p_ctrl, p_int]
Khat = acker(Ahat, Bhat, polos_i);

Aux  = [A1-1   B1;
        C1*A1  C1*B1];
K2K1 = (Khat + [zeros(1,n) eye(m)]) / Aux;

K2 = K2K1(1,1:n);          % realimentación sobre x
K1 = K2K1(1,n+1:end);      % sobre integrador (Ogata, v+=e)

Ki_psoc = K1 / Ts;         % ADAPTACIÓN A vint += e*Ts (tu firmware)

L_pred_i   = acker(A1', C1',      p_obs).';
L_actual_i = acker(A1', (C1*A1)', p_obs).';

% En integrador: Kx = Ki (y Kr efectivo será 0 en el firmware)
COEFF_SS_PRED_I = pack_ss16_1state(A1,B1,C1,D1, L_pred_i,   K2, Ki_psoc);
COEFF_SS_ACT_I  = pack_ss16_1state(A1,B1,C1,D1, L_actual_i, K2, Ki_psoc);

%% ===================== 5) SIMULACIÓN STEP (misma lógica que tu PSoC) =====================
Nsim   = 100;
k_step = 50;
r = zeros(1,Nsim);
r(k_step:end) = 1;

[t, y_tf, u_tf] = sim_TF_df2t(A1,B1,C1,D1, Ts, COEFF_TF, r);
[~, y_pn, u_pn] = sim_SS_mode(A1,B1,C1,D1, Ts, COEFF_SS_PRED_NOI, r, "PRED_NOI");
[~, y_an, u_an] = sim_SS_mode(A1,B1,C1,D1, Ts, COEFF_SS_ACT_NOI,  r, "ACT_NOI");
[~, y_pi, u_pi] = sim_SS_mode(A1,B1,C1,D1, Ts, COEFF_SS_PRED_I,   r, "PRED_I");
[~, y_ai, u_ai] = sim_SS_mode(A1,B1,C1,D1, Ts, COEFF_SS_ACT_I,    r, "ACT_I");

%% ===================== 6) PLOTS =====================
figure;
plot(t, r, 'k--', 'LineWidth', 1.2); hold on; grid on;
plot(t, y_tf, 'LineWidth', 1.2);
plot(t, y_pn, 'LineWidth', 1.2);
plot(t, y_an, 'LineWidth', 1.2);
plot(t, y_pi, 'LineWidth', 1.2);
plot(t, y_ai, 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('y');
title('Salida y(t) — step en r');
legend('r','TF optimo','SS pred no I','SS act no I','SS pred I','SS act I', 'Location','best');

figure;
stairs(t, u_tf, 'LineWidth', 1.2); hold on; grid on;
stairs(t, u_pn, 'LineWidth', 1.2);
stairs(t, u_an, 'LineWidth', 1.2);
stairs(t, u_pi, 'LineWidth', 1.2);
stairs(t, u_ai, 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('u');
title('Esfuerzo u(t)');
legend('TF optimo','SS pred no I','SS act no I','SS pred I','SS act I', 'Location','best');

%% ===================== 7) IMPRIMIR COEFICIENTES PARA PEGAR EN C =====================
print_c16("COEFF_TF",          COEFF_TF);
print_c16("COEFF_SS_PRED_NOI", COEFF_SS_PRED_NOI);
print_c16("COEFF_SS_ACT_NOI",  COEFF_SS_ACT_NOI);
print_c16("COEFF_SS_PRED_I",   COEFF_SS_PRED_I);
print_c16("COEFF_SS_ACT_I",    COEFF_SS_ACT_I);

%% ===================== FUNCIONES =====================

function c16 = pack_ss16_1state(A1,B1,C1,D1,L1,K_states,Kx)
    % Nuevo layout firmware (C index 0-based):
    % c[0..3]   A11 A12 A21 A22
    % c[4..5]   B1  B2
    % c[6..7]   C1  C2
    % c[8]      D
    % c[9..10]  L1  L2
    % c[11..12] K1  K2
    % c[13]     Kx (si modo *_I => Ki, si modo *_NOI => Kr)
    % c[14..15] reservados
    %
    % OJO: MATLAB 1-based => c[13] es c16(14).

    c16 = single(zeros(1,16));

    % A (2x2)
    c16(1)  = single(A1);   c16(2)  = 0;  c16(3)  = 0;  c16(4)  = 0;

    % B (2x1)
    c16(5)  = single(B1);   c16(6)  = 0;

    % C (1x2)
    c16(7)  = single(C1);   c16(8)  = 0;

    % D
    c16(9)  = single(D1);

    % L (2x1)
    c16(10) = single(L1);   c16(11) = 0;

    % K (1x2)
    c16(12) = single(K_states);  c16(13) = 0;

    % Kx (en C: c[13])
    c16(14) = single(Kx);

    % reservados
    c16(15) = 0;
    c16(16) = 0;
end

function c16 = pack_tf16_from_z(Cz)
    % Convierte TF en variable z a coeficientes en z^-1 para DF2T
    % DF2T espera: y = (b0 + b1 z^-1 + ...)/(1 + a1 z^-1 + ... ) * x
    [bz, az] = tfdata(Cz, 'v');   % polinomios en z (descendente)

    m = length(az) - 1;          % orden den
    n = length(bz) - 1;          % orden num
    if m > 5, error("TF den orden %d > 5 (no entra en DF2T).", m); end
    if n > 5, error("TF num orden %d > 5 (no entra en DF2T).", n); end

    a_zi = az(:).';
    b_zi = zeros(1, m+1);
    b_zi(end-(n):end) = bz(:).';

    a0 = a_zi(1);
    a_zi = a_zi / a0;
    b_zi = b_zi / a0;

    b6 = zeros(1,6); a6 = zeros(1,6);
    b6(1:m+1) = b_zi;
    a6(1:m+1) = a_zi;

    c16 = single([b6 a6 0 0 0 0]);
end

function [t, y_hist, u_hist] = sim_TF_df2t(A1,B1,C1,D1, Ts, cTF, r)
    N = numel(r);
    t = (0:N-1)*Ts;

    b = double(cTF(1:6));
    a = double(cTF(7:12));
    w = zeros(1,5);

    x = 0; u = 0;
    y_hist = zeros(1,N);
    u_hist = zeros(1,N);

    for k=1:N
        y = C1*x + D1*u;
        e = r(k) - y;

        [u, w] = df2t_step(e, b, a, w);

        x = A1*x + B1*u;

        y_hist(k) = y;
        u_hist(k) = u;
    end
end

function [t, y_hist, u_hist] = sim_SS_mode(A1,B1,C1,D1, Ts, cSS, r, mode)
    N = numel(r);
    t = (0:N-1)*Ts;

    % unpack (2 estados, 2do en cero)
    A = [double(cSS(1)) double(cSS(2));
         double(cSS(3)) double(cSS(4))];
    B = [double(cSS(5)); double(cSS(6))];
    C = [double(cSS(7)) double(cSS(8))];
    D = double(cSS(9));
    L = [double(cSS(10)); double(cSS(11))];
    K = [double(cSS(12)) double(cSS(13))];

    % Nuevo protocolo: c[13] (C) = cSS(14) (MATLAB) = Kx
    Kx = double(cSS(14));

    x = 0;
    xhat = [0;0];
    u = 0;
    vint = 0;

    y_hist = zeros(1,N);
    u_hist = zeros(1,N);

    for k=1:N
        y = C1*x + D1*u;

        switch mode
            case "PRED_NOI"
                Kr = Kx; Ki = 0;
                u = Kr*r(k) - (K*xhat);
                x = A1*x + B1*u;

                z = A*xhat + B*u;
                yhat = (C*z) + D*u;
                innov = y - yhat;
                xhat = z + L*innov;

            case "ACT_NOI"
                Kr = Kx; Ki = 0;
                yhat = (C*xhat) + D*u;
                innov = y - yhat;
                xhat = xhat + L*innov;

                u = Kr*r(k) - (K*xhat);
                x = A1*x + B1*u;

                z = A*xhat + B*u;
                xhat = z;

            case "PRED_I"
                Kr = 0; Ki = Kx;
                e = r(k) - y;
                vint = vint + e*Ts;

                u = Kr*r(k) + Ki*vint - (K*xhat);
                x = A1*x + B1*u;

                z = A*xhat + B*u;
                yhat = (C*z) + D*u;
                innov = y - yhat;
                xhat = z + L*innov;

            case "ACT_I"
                Kr = 0; Ki = Kx;
                yhat = (C*xhat) + D*u;
                innov = y - yhat;
                xhat = xhat + L*innov;

                e = r(k) - y;
                vint = vint + e*Ts;

                u = Kr*r(k) + Ki*vint - (K*xhat);
                x = A1*x + B1*u;

                z = A*xhat + B*u;
                xhat = z;

            otherwise
                error("Modo SS desconocido");
        end

        y_hist(k) = y;
        u_hist(k) = u;
    end
end

function [y, w] = df2t_step(x, b, a, w)
    % DF2T orden 5 como tu C
    y = b(1)*x + w(1);
    w(1) = w(2) + b(2)*x - a(2)*y;
    w(2) = w(3) + b(3)*x - a(3)*y;
    w(3) = w(4) + b(4)*x - a(4)*y;
    w(4) = w(5) + b(5)*x - a(5)*y;
    w(5) =          b(6)*x - a(6)*y;
end

function print_c16(name, c16)
    fprintf("\nstatic const float %s[16] = {\n", name);
    for i=1:16
        if i < 16
            fprintf("    %.9ff,\n", double(c16(i)));
        else
            fprintf("    %.9ff\n", double(c16(i)));
        end
    end
    fprintf("};\n");
end
