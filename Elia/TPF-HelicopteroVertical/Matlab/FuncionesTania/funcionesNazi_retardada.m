close all; clear; clc
load('planta (1).mat')

%% =========================
% 0) DISCRETIZACION
%% =========================
Ts   = 1/1000;
sysD = c2d(plantaC, Ts, 'zoh');
ssD  = ss(sysD);
[Ad,Bd,Cd,Dd] = ssdata(ssD);
n = size(Ad,1);

fprintf("\n================= BASICO =================\n");
fprintf("Ts = %.9f s | n = %d\n", Ts, n);
disp("Ad="); disp(Ad);
disp("Bd="); disp(Bd);
disp("Cd="); disp(Cd);
disp("Dd="); disp(Dd);

fprintf("\n================= RANKS ==================\n");
rc = rank(ctrb(Ad,Bd));
ro = rank(obsv(Ad,Cd));
fprintf("rank(ctrb) = %d (de %d)\n", rc, n);
fprintf("rank(obsv) = %d (de %d)\n", ro, n);

fprintf("\n============== ESCALAS (magnitudes) ==============\n");
fprintf("||Ad||F = %.3e | ||Bd||2 = %.3e | ||Cd||2 = %.3e\n", norm(Ad,'fro'), norm(Bd,2), norm(Cd,2));
fprintf("Cond(obsv) aprox (svd): ");
sv = svd(obsv(Ad,Bd*0 + Cd)); %#ok<NBRAK> % truquito para no cambiar nada del flujo
sv = svd(obsv(Ad,Cd));
fprintf("sigma_max/sigma_min = %.3e\n", sv(1)/sv(end));

fprintf("y~Cd*x con ||x||=1 => |y| ~ ||Cd||2 = %.3e\n", norm(Cd,2));

%% =========================
% 1) PARAMETROS (LOS TUYOS)
%% =========================
p_ctrl = [0.999 + 0.001i, 0.999 - 0.001i, 0.999];
p_obs  = [0.995 + 0.005i, 0.995 - 0.005i, 0.990];

fprintf("\n================= POLOS DESEADOS =================\n");
disp("p_ctrl = "); disp(p_ctrl);
disp("p_obs  = "); disp(p_obs);

USE_BAL = false;

if USE_BAL
    fprintf("\n========== USANDO ssbal (balanceado) ==========\n");
    [Ab,Bb,Cb,T] = ssbal(Ad,Bd,Cd);
    Kb   = place(Ab, Bb, p_ctrl);
    Keb  = place(Ab', Cb', p_obs).';
    K    = Kb / T;
    Ke_pred = T * Keb;
    Ke_act  = place(Ad', (Cd*Ad)', p_obs).';
else
    K       = place(Ad, Bd, p_ctrl);
    Ke_pred = place(Ad', Cd', p_obs).';        % predictor: A - Ke*C
    Ke_act  = place(Ad', (Cd*Ad)', p_obs).';   % actual:    A - Ke*C*A
end

I = eye(n);
Nbar = 1 / (Cd * ((I - Ad + Bd*K)\Bd));

fprintf("\n================= GANANCIAS =================\n");
disp("K = "); disp(K);
fprintf("||K||2 = %.3e | max|K| = %.3e\n", norm(K,2), max(abs(K)));

disp("Ke_pred = "); disp(Ke_pred);
fprintf("||Ke_pred||2 = %.3e | max|Ke_pred| = %.3e\n", norm(Ke_pred,2), max(abs(Ke_pred)));

disp("Ke_act = "); disp(Ke_act);
fprintf("||Ke_act||2 = %.3e | max|Ke_act| = %.3e\n", norm(Ke_act,2), max(abs(Ke_act)));

disp("Nbar = "); disp(Nbar);

%% =========================
% 2) POLOS RESULTANTES
%% =========================
fprintf("\n================= POLOS RESULTANTES =================\n");
pK   = eig(Ad - Bd*K);
pOp  = eig(Ad - Ke_pred*Cd);
pOa  = eig(Ad - Ke_act*Cd*Ad);

disp("Polos control eig(A-BK) = "); disp(pK.');
disp("Polos obs pred eig(A-KeC) = "); disp(pOp.');
disp("Polos obs act  eig(A-KeCA) = "); disp(pOa.');

%% =========================
% 3) RETARDO REAL (104 us)
%   - Opcion A: THIRAN (all-pass) en LTI (recomendado)
%   - Opcion B: mezcla lineal dentro del loop (mas “firmware-like”)
%% =========================
tau = 104e-6;                 % 104 us
D   = tau/Ts;                 % retardo en samples (0.104)

fprintf("\n================= DELAY =================\n");
fprintf("tau = %.3eus | Ts = %.3eus | D = tau/Ts = %.6f samples\n", tau*1e6, Ts*1e6, D);

% ---- A) Thiran all-pass (retardo fraccional)
Nd = 3;                       % orden del aproximador (2..5 suele ir bien)
Hd = thiran(D, Nd);
Hd.Ts = Ts;

sysD_del = series(Hd, sysD);  % u -> delay -> planta
ssD_del  = ss(sysD_del);
[Ad_d,Bd_d,Cd_d,Dd_d] = ssdata(ssD_del);

% Nota: para no romper tu estructura, mantenemos Cd y K sobre el estado original,
% y usamos el retardo en la “planta” de referencia via sysD_del en los stepplot.
% Para el aumentado (observador) esto no es exacto si lo armas con matrices,
% por eso hacemos dos cosas:
%   1) Stepplot LTI aproximado (usando sysD_del y un controlador/observador en LTI con 'feedback')
%   2) Simulacion en loop (Opcion B) que SI es fiel al firmware.

%% =========================
% 4) SISTEMAS AUMENTADOS (SIN DELAY) (como tu script)
%% =========================
Aaug_pred = [ Ad,         -Bd*K;
              Ke_pred*Cd,  Ad - Ke_pred*Cd - Bd*K ];

Baug_pred = [ Bd*Nbar;
              Bd*Nbar ];

Caug_y = [Cd, zeros(size(Cd))];
sys_aug_pred_y = ss(Aaug_pred, Baug_pred, Caug_y, 0, Ts);

Caug_u = [zeros(1,n), -K];
sys_aug_pred_u = ss(Aaug_pred, Baug_pred, Caug_u, Nbar, Ts);

Aaug_act = [ Ad,              -Bd*K;
             Ke_act*Cd*Ad,     Ad - Ke_act*Cd*Ad - Bd*K ];

Baug_act = [ Bd*Nbar;
             Bd*Nbar ];

sys_aug_act_y = ss(Aaug_act, Baug_act, Caug_y, 0, Ts);
sys_aug_act_u = ss(Aaug_act, Baug_act, Caug_u, Nbar, Ts);

fprintf("\n============ CHECK: polos del AUMENTADO (SIN delay) ============\n");
disp("eig(Aaug_pred) = "); disp(eig(Aaug_pred).');
disp("eig(Aaug_act)  = "); disp(eig(Aaug_act).');

%% =========================
% 5) STEP SIN OBSERVADOR (SIN delay) (x medido)
%% =========================
Acl = Ad - Bd*K;
Bcl = Bd*Nbar;

sys_noobs_y = ss(Acl, Bcl, Cd, 0, Ts);         % r -> y
sys_noobs_u = ss(Acl, Bcl, -K, Nbar, Ts);      % r -> u

%% =========================
% 6) PLOTS SIN DELAY (tus plots)
%% =========================
figure('Name','Predictor (SIN delay)','NumberTitle','off');

ax1 = subplot(2,2,1);
stepplot(ax1, sys_aug_pred_y); grid(ax1,'on'); grid(ax1,'minor');
title(ax1,'Predictor: step r->y');

ax2 = subplot(2,2,2);
stepplot(ax2, sys_aug_pred_u); grid(ax2,'on'); grid(ax2,'minor');
title(ax2,'Predictor: step r->u');

ax3 = subplot(2,2,[3 4]);
pzmap(ax3, sys_aug_pred_y); grid(ax3,'on'); grid(ax3,'minor');
zgrid(ax3); title(ax3,'Predictor: PZMAP (r->y)');
xlim(ax3,[-1.2 1.2]); ylim(ax3,[-1.2 1.2]); axis(ax3,'equal');

figure('Name','Actual (SIN delay)','NumberTitle','off');

ax1 = subplot(2,2,1);
stepplot(ax1, sys_aug_act_y); grid(ax1,'on'); grid(ax1,'minor');
title(ax1,'Actual: step r->y');

ax2 = subplot(2,2,2);
stepplot(ax2, sys_aug_act_u); grid(ax2,'on'); grid(ax2,'minor');
title(ax2,'Actual: step r->u');

ax3 = subplot(2,2,[3 4]);
pzmap(ax3, sys_aug_act_y); grid(ax3,'on'); grid(ax3,'minor');
zgrid(ax3); title(ax3,'Actual: PZMAP (r->y)');
xlim(ax3,[-1.2 1.2]); ylim(ax3,[-1.2 1.2]); axis(ax3,'equal');

figure('Name','Sin observador (SIN delay)','NumberTitle','off');

ax1 = subplot(2,2,1);
stepplot(ax1, sys_noobs_y); grid(ax1,'on'); grid(ax1,'minor');
title(ax1,'No-Obs: step r->y');

ax2 = subplot(2,2,2);
stepplot(ax2, sys_noobs_u); grid(ax2,'on'); grid(ax2,'minor');
title(ax2,'No-Obs: step r->u');

ax3 = subplot(2,2,[3 4]);
pzmap(ax3, sys_noobs_y); grid(ax3,'on'); grid(ax3,'minor');
zgrid(ax3); title(ax3,'No-Obs: PZMAP (r->y)');
xlim(ax3,[-1.2 1.2]); ylim(ax3,[-1.2 1.2]); axis(ax3,'equal');

fprintf("\n============ SIN OBSERVADOR ============\n");
disp("eig(Acl)=eig(A-BK) = "); disp(eig(Acl).');

%% =========================
% 7) RESUMEN COPIABLE
%% =========================
fprintf("\n\n=========== PEGAME ESTE BLOQUE ===========\n");
fprintf("Ts=%.9f n=%d | rankC=%d rankO=%d\n", Ts,n,rc,ro);
fprintf("||Cd||=%.3e  cond(obsv)~%.3e\n", norm(Cd,2), sv(1)/sv(end));
fprintf("max|K|=%.3e  Nbar=%.6g  max|Ke_pred|=%.3e  max|Ke_act|=%.3e\n", ...
    max(abs(K)), Nbar, max(abs(Ke_pred)), max(abs(Ke_act)));
fprintf("eig(A-BK) = "); fprintf("%.6f%+.6fi  ", [real(pK).'; imag(pK).']); fprintf("\n");
fprintf("eig(A-KeC) = "); fprintf("%.6f%+.6fi  ", [real(pOp).'; imag(pOp).']); fprintf("\n");
fprintf("eig(A-KeCA)= "); fprintf("%.6f%+.6fi  ", [real(pOa).'; imag(pOa).']); fprintf("\n");
fprintf("eig(Aaug_pred) max|.| = %.6f\n", max(abs(eig(Aaug_pred))));
fprintf("eig(Aaug_act ) max|.| = %.6f\n", max(abs(eig(Aaug_act))));
fprintf("delay D=tau/Ts = %.6f  (tau=%.1fus)\n", D, tau*1e6);
fprintf("=========================================\n");

%% =========================================================
% 8) SIMULACION “FIEL AL FIRMWARE” CON DELAY FRACCIONAL
%    (loop) para comparar Predictor / Actual / No-Obs
%
%    - usa u_aplicado = (1-alpha)*u_prev + alpha*u   con alpha=1-D
%    - IMPORTANTISIMO: el observador se actualiza con u_aplicado (no con u ideal)
%% =========================================================
alpha = 1 - D;   % con D=0.104 => alpha~0.896
Nsim  = 8000;    % samples para sim (2.5s)

r = ones(1,Nsim);            % step unitario
r(1:20) = 0;                 % arranca en 0 un ratito

% ---- buffers
y_noobs = zeros(1,Nsim);
y_pred  = zeros(1,Nsim);
y_act   = zeros(1,Nsim);

u_noobs = zeros(1,Nsim);
u_pred  = zeros(1,Nsim);
u_act   = zeros(1,Nsim);

% ---- estados reales planta (3 simulaciones separadas)
x_noobs = zeros(n,1);
x_predP = zeros(n,1);
x_actP  = zeros(n,1);

% ---- estados estimados
xhat_pred = zeros(n,1);      % posterior
zhat_act  = zeros(n,1);      % prior
xhat_act  = zeros(n,1);      % posterior

% ---- memoria de u (para delay)
u_prev_noobs = 0;
u_prev_pred  = 0;
u_prev_act   = 0;

uap_prev_noobs = 0;
uap_prev_pred  = 0;
uap_prev_act   = 0;

for k = 1:Nsim-1

    %% -------------------------
    % NO OBS (x medido)
    %% -------------------------
    u_ideal = Nbar*r(k) - K*x_noobs;                  % u[k] ideal
    u_ap    = (1-alpha)*u_prev_noobs + alpha*u_ideal; % aplicado con delay fraccional

    % planta
    x_noobs = Ad*x_noobs + Bd*u_ap;
    y_noobs(k+1) = Cd*x_noobs + Dd*u_ap;

    % logs
    u_noobs(k) = u_ideal;
    u_prev_noobs = u_ideal;
    uap_prev_noobs = u_ap;

    %% -------------------------
    % PREDICTOR (Ke_pred)
    %% -------------------------
    % control con xhat (posterior)
    u_ideal = Nbar*r(k) - K*xhat_pred;
    u_ap    = (1-alpha)*u_prev_pred + alpha*u_ideal;

    % planta real de este caso
    x_predP = Ad*x_predP + Bd*u_ap;
    yk      = Cd*x_predP + Dd*u_ap;
    y_pred(k+1) = yk;

    % observer predictor:
    % xhat(k+1) = A*xhat(k) + B*u_ap + Ke*(y(k) - C*xhat(k))
    xhat_pred = Ad*xhat_pred + Bd*u_ap + Ke_pred*(yk - Cd*xhat_pred);

    % logs
    u_pred(k) = u_ideal;
    u_prev_pred = u_ideal;
    uap_prev_pred = u_ap;

    %% -------------------------
    % ACTUAL (Ke_act)
    %% -------------------------
    % control con xhat_act (posterior)
    u_ideal = Nbar*r(k) - K*xhat_act;
    u_ap    = (1-alpha)*u_prev_act + alpha*u_ideal;

    % planta real de este caso
    x_actP = Ad*x_actP + Bd*u_ap;
    yk1    = Cd*x_actP + Dd*u_ap;
    y_act(k+1) = yk1;

    % observer actual:
    % predict: zhat = A*xhat + B*u_ap
    zhat_act = Ad*xhat_act + Bd*u_ap;
    % correct using y(k+1): xhat = zhat + Kea*(y(k+1) - C*zhat)
    xhat_act = zhat_act + Ke_act*(yk1 - Cd*zhat_act);

    % logs
    u_act(k) = u_ideal;
    u_prev_act = u_ideal;
    uap_prev_act = u_ap;
end

u_noobs(end) = u_noobs(end-1);
u_pred(end)  = u_pred(end-1);
u_act(end)   = u_act(end-1);

t = (0:Nsim-1)*Ts;

%% =========================
% 9) PLOTS LOOP (CON delay)
%% =========================
figure('Name','Comparacion (CON delay 104us) - Salida y','NumberTitle','off');
plot(t, y_noobs, 'LineWidth', 1.5); hold on;
plot(t, y_pred,  'LineWidth', 1.5);
plot(t, y_act,   'LineWidth', 1.5);
grid on; grid minor;
xlabel('t [s]'); ylabel('y');
title(sprintf('Salida y con delay fraccional (tau=%.0fus, D=%.3fTs)', tau*1e6, D));
legend('No-Obs (x medido)','Predictor','Actual','Location','best');

figure('Name','Comparacion (CON delay 104us) - Control u ideal','NumberTitle','off');
stairs(t, u_noobs, 'LineWidth', 1.5); hold on;
stairs(t, u_pred,  'LineWidth', 1.5);
stairs(t, u_act,   'LineWidth', 1.5);
grid on; grid minor;
xlabel('t [s]'); ylabel('u ideal');
title('u[k] ideal (antes del delay)');
legend('No-Obs','Predictor','Actual','Location','best');

%% =========================
% 10) EXTRA: step LTI “planta con delay” (solo para intuicion)
%     (No es exacto para observadores armados por matrices, pero sirve como referencia)
%% =========================
figure('Name','Planta con delay (solo referencia LTI)','NumberTitle','off');
stepplot(sysD, sysD_del);
grid on; grid minor;
title(sprintf('Step planta: sin delay vs con Thiran (tau=%.0fus, Nd=%d)', tau*1e6, Nd));
legend('sin delay','con delay (Thiran)','Location','best');

fprintf("\n================= NOTAS =================\n");
fprintf("Sim loop: u_ap = (1-alpha)*u_prev + alpha*u, alpha=1-D = %.6f\n", alpha);
fprintf("Si con delay el predictor explota y el actual no, el problema suele ser observador + saturacion/delay.\n");
