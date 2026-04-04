%% make_ctrls_psoc_tau.m
clear all; close all; clc;

%% ====== PLANTA (la que reiteraste) ======
tau = 1e6*2e-9;     % [s]  = 5e-6
Ts  = 1/1e3;      % 100 kHz

s  = tf('s');
G  = 1/(tau*s + 1);

Gd   = c2d(G, Ts, 'zoh');
sysD = ss(Gd);
[A,B,C,D] = ssdata(sysD);
n = size(A,1);      % n=1 para esta planta

fprintf("tau=%.3g s | Ts=%.3g s | A=%.9f B=%.9f\n", tau, Ts, A, B);

%% ====== Polos (no te importan, pero que sean estables) ======
p_tf   = 0.60;   % "TF normal” -> acá va a ser un P puro u=Kp*e
p_ctrl = 0.60;   % polo deseado del lazo cerrado principal (REAL, |p|<1)
p_obs  = 0.0;   % polo del observador (más rápido => más chico)
p_int  = 0.15;   % polo extra para el aumentado con integrador (como tu Ogata)
%% ============================================================
%  (0) CONTROLADOR TF NORMAL (u = Kp * e)
%  Cerrado: x[k+1] = (A - B*Kp*C) x[k] + B*Kp*r
%  => polo = A - B*Kp*C  => Kp = (A - p_tf)/(B*C)
%% ============================================================
Kp = 1;
coeff_tf = tf16_gain(Kp);

%% ============================================================
%  (1) SS SIN INTEGRADOR (tu estructura)
%      u = Kr*r - K*xhat
%      Kr = Nbar por dcgain (como pediste)
%% ============================================================
K = acker(A,B,p_ctrl);

sys_cl = ss(A - B*K, B, C, D, Ts);
gdc = dcgain(sys_cl);
if abs(gdc) < 1e-12, error("dcgain ~ 0 (Nbar revienta)."); end
Nbar = 1/gdc;

L_pred   = acker(A', C',    p_obs).';      % predictivo
L_actual = acker(A', (C*A)', p_obs).';     % actual (tu estilo)

coeff_ss_pred_noi = pack_ss16_1state(A,B,C,D, L_pred,   K,  Nbar,  0  );
coeff_ss_act_noi  = pack_ss16_1state(A,B,C,D, L_actual, K,  Nbar,  0  );

%% ============================================================
%  (2) SS CON INTEGRADOR — Ogata (TU BLOQUE TAL CUAL)
%  OJO: tu firmware integra con Ts: vint += e*Ts
%       Ogata te da K1 para v += e (sin Ts)
%       => Ki_psoc = K1/Ts  (esto no cambia Ogata, adapta tu implementación)
%% ============================================================
m = 1;
Ahat = [A B; zeros(m,n+m)];
Bhat = [zeros(n,m); eye(m)];
Chat = [C zeros(1,m)];

polos_i = [p_ctrl.' p_int];    % para n=1 => [p_ctrl 0.9]
Khat = acker(Ahat,Bhat,polos_i);

Aux  = [A-eye(size(A))  B;
        C*A             C*B];
K2K1 = (Khat + [zeros(1,n) eye(m)]) / Aux;
K2   = K2K1(1,1:n);          % sobre x
K1   = K2K1(1,n+1:end);      % sobre integrador


% Observadores (mismo p_obs)
L_pred_i   = acker(A', C',     p_obs).';
L_actual_i = acker(A', (C*A)', p_obs).';

coeff_ss_pred_i = pack_ss16_1state(A,B,C,D, L_pred_i,   K2, K1, 0);
coeff_ss_act_i  = pack_ss16_1state(A,B,C,D, L_actual_i, K2, K1, 0);

%% ====== Imprimir arrays para C (16 floats) ======
print_c_array("COEFF_TF",          coeff_tf);
print_c_array("COEFF_SS_PRED_NOI", coeff_ss_pred_noi);
print_c_array("COEFF_SS_ACT_NOI",  coeff_ss_act_noi);
print_c_array("COEFF_SS_PRED_I",   coeff_ss_pred_i);
print_c_array("COEFF_SS_ACT_I",    coeff_ss_act_i);

%% ============================================================
%  (3) SIMULACIÓN y GRÁFICOS (5 subplots: y y u en el mismo plot)
%% ============================================================
Nsim   = 40;      % cantidad de muestras a simular
r_step = 1.0;      % step de referencia

[t, y_tf,  u_tf ] = sim_tf_p(A,B,C,D,Ts,Nsim,r_step,Kp);

[t, y_pn,  u_pn ] = sim_ss_pred(A,B,C,D,Ts,Nsim,r_step, K,  Nbar, L_pred,    0);
[t, y_an,  u_an ] = sim_ss_act (A,B,C,D,Ts,Nsim,r_step, K,  Nbar, L_actual,  0);

[t, y_pi,  u_pi ] = sim_ss_pred(A,B,C,D,Ts,Nsim,r_step, K2, 0,    L_pred_i,  K1);
[t, y_ai,  u_ai ] = sim_ss_act (A,B,C,D,Ts,Nsim,r_step, K2, 0,    L_actual_i,K1);

figure('Name','Respuesta a step: y y u (5 modos)','Color','w');
tiledlayout(5,1,'Padding','compact','TileSpacing','compact');

plot_one(t, r_step, y_tf, u_tf,  'TF (P): u=Kp*(r-y)');
plot_one(t, r_step, y_pn, u_pn,  'SS PRED NOI: u=Kr*r - K*xhat');
plot_one(t, r_step, y_an, u_an,  'SS ACT  NOI: u=Kr*r - K*xhat');
plot_one(t, r_step, y_pi, u_pi,  'SS PRED I: u=Ki*v - K*xhat, v+= (r-y)*Ts');
plot_one(t, r_step, y_ai, u_ai,  'SS ACT  I: u=Ki*v - K*xhat, v+= (r-y)*Ts');


%% ================== helpers ==================
function v16 = tf16_gain(Kp)
    % TF puro: u = Kp * e  => b0=Kp, a0=1
    v16 = single([ ...
        Kp 0 0 0 0 0, ...
        1  0 0 0 0 0, ...
        0 0 0 0 ]);
end

function v16 = pack_ss16_1state(A,B,C,D,L,K,Ki,Kr)
    % Tu firmware SS espera 2 estados -> el 2do lo ponemos en 0
    v16 = zeros(1,16,'single');
    v16(1:4)   = single([A 0 0 0]);     % A11 A12 A21 A22
    v16(5:6)   = single([B 0]);         % B1 B2
    v16(7:8)   = single([C 0]);         % C1 C2
    v16(9)     = single(D);             % D
    v16(10:11) = single([L 0]);         % L1 L2
    v16(12:13) = single([K 0]);         % K1 K2
    v16(14)    = single(Ki);            % Ki
    v16(15)    = single(0);            % Kr
    v16(16)    = single(0);             % reservado
end

function print_c_array(name, v)
    fprintf("\nstatic const float %s[16] = {\n", name);
    for i=1:16
        if i<16
            fprintf("    %.9f,\n", v(i));
        else
            fprintf("    %.9ff\n", v(i));
        end
    end
    fprintf("};\n");
end

function plot_one(t, r, y, u, ttl)
    nexttile;

    % ---- Interpolación de y (100 puntos entre muestras) ----
    ti = linspace(t(1), t(end), (numel(t)-1)*100 + 1).';
    yi = interp1(t, y, ti, 'pchip');   % suave, tipo "planta continua"

    yyaxis left
    h1 = plot(ti, yi, 'LineWidth', 1.2); hold on;
    h3 = plot(ti, r*ones(size(ti)), '--', 'LineWidth', 1.0);
    ylabel('y [V]');
    grid on;

    yyaxis right
    h2 = stairs(t, u, 'LineWidth', 1.2);   % u discretizada
    ylabel('u [V]');

    title(ttl);
    legend([h1 h2 h3], {'y','u','r'}, 'Location','best');
end


function [t, y_hist, u_hist] = sim_tf_p(A,B,C,D,Ts,N,r,Kp)
    x = 0; 
    u_prev = 0;

    y_hist = zeros(N,1);
    u_hist = zeros(N,1);
    t = (0:N-1)'*Ts;

    for k = 1:N
        % planta evoluciona con u_prev
        x = A*x + B*u_prev;
        y = C*x + D*u_prev;

        % control
        u = Kp*(r - y);

        % (opcional) saturación física:
        % u = min(max(u, 0), 4.08);

        y_hist(k) = y;
        u_hist(k) = u;
        u_prev = u;
    end
end

function [t, y_hist, u_hist] = sim_ss_pred(A,B,C,D,Ts,N,r,K,Kr,L,Ki)
    x = 0;
    xhat = 0;
    u_prev = 0;
    v = 0;

    y_hist = zeros(N,1);
    u_hist = zeros(N,1);
    t = (0:N-1)'*Ts;

    for k = 1:N
        % planta con u_prev
        x = A*x + B*u_prev;
        y = C*x + D*u_prev;

        % integrador (si Ki!=0)
        if Ki ~= 0
            v = v + (r - y)*Ts;
        end

        % control con xhat(k)
        u = Kr*r + Ki*v - K*xhat;

        % observador predictor:
        innov = y - (C*xhat + D*u_prev);
        xhat = (A*xhat + B*u) + L*innov;

        % (opcional) saturación física:
        % u = min(max(u, 0), 4.08);

        y_hist(k) = y;
        u_hist(k) = u;
        u_prev = u;
    end
end

function [t, y_hist, u_hist] = sim_ss_act(A,B,C,D,Ts,N,r,K,Kr,L,Ki)
    x = 0;
    xhat = 0;
    zhat = 0;
    u_prev = 0;
    v = 0;

    y_hist = zeros(N,1);
    u_hist = zeros(N,1);
    t = (0:N-1)'*Ts;

    for k = 1:N
        % planta con u_prev
        x = A*x + B*u_prev;
        y = C*x + D*u_prev;

        % corrección current: xhat(k) desde zhat(k)
        innov = y - (C*zhat + D*u_prev);
        xhat = zhat + L*innov;

        % integrador (si Ki!=0)
        if Ki ~= 0
            v = v + (r - y)*Ts;
        end

        % control con xhat(k)
        u = Kr*r + Ki*v - K*xhat;

        % predicción para próximo ciclo
        zhat = (A*xhat + B*u);

        % (opcional) saturación física:
        % u = min(max(u, 0), 4.08);

        y_hist(k) = y;
        u_hist(k) = u;
        u_prev = u;
    end
end
