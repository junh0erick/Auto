close all; clear; clc

%% =========================
% 1) CARGA + DISCRETIZACIÓN
%% =========================
S = load('planta (1).mat');

if isfield(S,'plantaC')
    plantaC = S.plantaC;
elseif isfield(S,'sysC')
    plantaC = S.sysC;
else
    error('No encuentro "plantaC" ni "sysC" dentro de planta (1).mat');
end
Ts   = 1/50;                 % <-- ajustá si querés
sysD = c2d(plantaC, Ts, 'zoh');
[A,B,C,D] = ssdata(ss(sysD));
n = size(A,1);

fprintf('Ts=%.9f | n=%d\n', Ts, n);

%% =========================
% 2) POLOS DESEADOS (AJUSTE AUTOMÁTICO A n)
%% =========================
p_ctrl = [0.95 + 0.15i, 0.95 - 0.15i,0.95];
p_obs  = [0.8 + 0.25i, 0.8 - 0.25i, 0.9];

%% =========================
% 3) GANANCIAS CON place + prec
%% =========================
[K, precK] = place(A,B,p_ctrl);

[KeP, precKeP] = place(A', C', p_obs);      Ke_pred = KeP.';          % predictor: A - Ke*C
[KeA, precKeA] = place(A', (C*A)', p_obs);  Ke_act  = KeA.';          % "actual": A - Ke*C*A

% Nbar (precompensador discreto) para seguimiento de referencia
% OJO: esto asume SISO (una sola salida). Si C tiene varias filas, elegí una salida.

[~,~,Nbar] = refi(A,B,C,K)


fprintf('precK=%.2f dig | precKe_pred=%.2f dig | precKe_act=%.2f dig\n', precK, precKeP, precKeA);
fprintf('Nbar=%.6g\n', Nbar);

%% =========================
% 4) SIMULACIÓN: IDEAL vs REALISTA + float32
%% =========================
N        = 200;
ulim_sat = 300;     % realista (sat ±300)
ulim_inf = Inf;     % ideal (sin sat)

% step 0 -> 25
r = zeros(1,N);
r(2:end) = 20;

% ----- DOUBLE ideal (sin saturación) -----
out_noobs_d_ideal = sim_noobs_loop(A,B,C,D,K,Nbar,r,Ts,ulim_inf,false);
out_pred_d_ideal  = sim_obs_loop  (A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_inf,false,false);
out_act_d_ideal   = sim_obs_loop  (A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_inf,true ,false);

% ----- DOUBLE realista (con saturación) -----
out_noobs_d_sat = sim_noobs_loop(A,B,C,D,K,Nbar,r,Ts,ulim_sat,false);
out_pred_d_sat  = sim_obs_loop  (A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_sat,false,false);
out_act_d_sat   = sim_obs_loop  (A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_sat,true ,false);

% ----- FLOAT32 realista (con saturación) -----
out_noobs_f = sim_noobs_loop(A,B,C,D,K,Nbar,r,Ts,ulim_sat,true);
out_pred_f  = sim_obs_loop  (A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_sat,false,true);
out_act_f   = sim_obs_loop  (A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim_sat,true ,true);

t = (0:N-1)*Ts;

%% =========================
% 5) PLOTS RÁPIDOS (ideal vs real)
%% =========================
figure('Name','IDEAL: y (double, sin sat)');
plot(t, out_noobs_d_ideal.y, t, out_pred_d_ideal.y, '--', t, out_act_d_ideal.y, '-.');
grid on; xlabel('t [s]'); ylabel('y'); legend('NoObs','Pred','Act','Location','best');
title('IDEAL: y(t) sin saturación');

figure('Name','IDEAL: u (double, sin sat)');
plot(t, out_noobs_d_ideal.u, t, out_pred_d_ideal.u, '--', t, out_act_d_ideal.u, '-.');
grid on; xlabel('t [s]'); ylabel('u'); legend('NoObs','Pred','Act','Location','best');
title('IDEAL: u(t) sin saturación');

figure('Name','REAL: y (double, sat ±300)');
plot(t, out_noobs_d_sat.y, t, out_pred_d_sat.y, '--', t, out_act_d_sat.y, '-.');
grid on; xlabel('t [s]'); ylabel('y'); legend('NoObs','Pred','Act','Location','best');
title('REAL: y(t) con saturación ±300');

figure('Name','REAL: u (double, sat ±300)');
plot(t, out_noobs_d_sat.u, t, out_pred_d_sat.u, '--', t, out_act_d_sat.u, '-.');
yline(+ulim_sat,'k--'); yline(-ulim_sat,'k--');
grid on; xlabel('t [s]'); ylabel('u'); legend('NoObs','Pred','Act','Location','best');
title('REAL: u(t) con saturación ±300');

figure('Name','Error float32 vs double (REAL, y)');
plot(t, out_pred_d_sat.y - out_pred_f.y, t, out_act_d_sat.y - out_act_f.y);
grid on; xlabel('t [s]'); ylabel('y_{double} - y_{float32}');
legend('Pred','Act','Location','best');
title('REAL: diferencia por usar float32 (single)');

fprintf('\nMax |u| ideal (double): NoObs=%.3g  Pred=%.3g  Act=%.3g\n', ...
    max(abs(out_noobs_d_ideal.u)), max(abs(out_pred_d_ideal.u)), max(abs(out_act_d_ideal.u)));

fprintf('Muestras saturadas (double realista ±300): NoObs=%d  Pred=%d  Act=%d\n', ...
    sum(abs(out_noobs_d_sat.u) >= ulim_sat-1e-9), ...
    sum(abs(out_pred_d_sat.u)  >= ulim_sat-1e-9), ...
    sum(abs(out_act_d_sat.u)   >= ulim_sat-1e-9));

%% =========================
% 6) Z-PLANE: polos planta + polos CL + polos observador (eig)
%% =========================
p_ol = eig(A);             % planta discreta
p_cl = eig(A - B*K);       % lazo cerrado planta (state-feedback)

p_op = eig(A - Ke_pred*C);     % error obs predictor
p_oa = eig(A - Ke_act*C*A);    % error obs actual

try
    z_plant = tzero(ss(A,B,C,D,Ts));
catch
    z_plant = [];
end

%% =========================
% 7) SUBPLOTS "COMO ANTES": Predictor/Actual (IDEAL y REAL)
%% =========================
plot_obs_fig('Predictor (IDEAL)', t, r, out_pred_d_ideal, ulim_inf, ...
             p_ol, p_cl, p_op, z_plant);

plot_obs_fig('Predictor (REAL sat±300)', t, r, out_pred_d_sat, ulim_sat, ...
             p_ol, p_cl, p_op, z_plant);

plot_obs_fig('Actual (IDEAL)', t, r, out_act_d_ideal, ulim_inf, ...
             p_ol, p_cl, p_oa, z_plant);

plot_obs_fig('Actual (REAL sat±300)', t, r, out_act_d_sat, ulim_sat, ...
             p_ol, p_cl, p_oa, z_plant);

%% =========================
% 8) Dump de polos (por si querés chequear)
%% =========================
fprintf('\nPolos planta (A):\n');           disp(p_ol.');
fprintf('Polos CL (A-BK):\n');            disp(p_cl.');
fprintf('Polos obs pred (A-KeC):\n');     disp(p_op.');
fprintf('Polos obs act  (A-KeCA):\n');    disp(p_oa.');

%% ============================================================
% ===================== FUNCIONES LOCALES ======================


function out = sim_noobs_loop(A,B,C,D,K,Nbar,r,Ts,ulim,use_single)
    if use_single
        A=single(A); B=single(B); C=single(C); D=single(D);
        K=single(K); Nbar=single(Nbar); r=single(r);
        Ts=single(Ts); ulim=single(ulim);
    end
    n = size(A,1); N = numel(r);
    x = zeros(n,N,'like',A);
    y = zeros(1,N,'like',A);
    u = zeros(1,N,'like',A);

    y(1) = C*x(:,1) + D*0;

    for k=1:N-1
        u_unsat = Nbar*r(k) - K*x(:,k);
        u(k)    = sat(u_unsat, ulim);

        x(:,k+1) = A*x(:,k) + B*u(k);
        y(k)     = C*x(:,k)   + D*u(k);
        y(k+1)   = C*x(:,k+1) + D*u(k);  % ZOH: u(k) actúa en [k,k+1)
    end

    u(N) = sat(Nbar*r(N) - K*x(:,N), ulim);
    y(N) = C*x(:,N) + D*u(N);

    out.x = x; out.y = y; out.u = u;
end

function out = sim_obs_loop(A,B,C,D,K,Nbar,Ke_pred,Ke_act,r,Ts,ulim,use_actual,use_single)
    if use_single
        A=single(A); B=single(B); C=single(C); D=single(D);
        K=single(K); Nbar=single(Nbar);
        Ke_pred=single(Ke_pred); Ke_act=single(Ke_act);
        r=single(r); Ts=single(Ts); ulim=single(ulim);
    end
    n = size(A,1); N = numel(r);
    x  = zeros(n,N,'like',A);   % estado real planta
    xh = zeros(n,N,'like',A);   % estado estimado
    y  = zeros(1,N,'like',A);
    u  = zeros(1,N,'like',A);

    y(1) = C*x(:,1) + D*0;

    for k=1:N-1
        u_unsat = Nbar*r(k) - K*xh(:,k);
        u(k)    = sat(u_unsat, ulim);

        % planta
        x(:,k+1) = A*x(:,k) + B*u(k);
        y(k)     = C*x(:,k)   + D*u(k);
        y(k+1)   = C*x(:,k+1) + D*u(k);

        if ~use_actual
            % predictor: corrige con y(k)
            yhat_k = C*xh(:,k) + D*u(k);
            xh(:,k+1) = A*xh(:,k) + B*u(k) + Ke_pred*( y(k) - yhat_k );
        else
            % actual: corrige con y(k+1) usando z = A*xhat + B*u
            z  = A*xh(:,k) + B*u(k);
            yzh = C*z + D*u(k);
            xh(:,k+1) = z + Ke_act*( y(k+1) - yzh );
        end
    end

    u(N) = sat(Nbar*r(N) - K*xh(:,N), ulim);
    y(N) = C*x(:,N) + D*u(N);

    out.x = x; out.xh = xh; out.y = y; out.u = u;
end

function y = sat(u,lim)
    if isinf(lim)
        y = u;      % sin saturación
    else
        y = min(max(u, -lim), lim);
    end
end

function plot_obs_fig(figName, t, r, out, ulim, p_ol, p_cl, p_obs, z_plant)
    figure('Name',figName,'NumberTitle','off');

    ax1 = subplot(2,2,1);
    plot(ax1, t, out.y, 'LineWidth', 1.8); hold(ax1,'on');
    plot(ax1, t, r, 'k--', 'LineWidth', 1.2);
    grid(ax1,'on'); grid(ax1,'minor');
    xlabel(ax1,'t [s]'); ylabel(ax1,'y');
    title(ax1,'step r \rightarrow y');
    legend(ax1,'y','r','Location','best');

    ax2 = subplot(2,2,2);
    plot(ax2, t, out.u, 'LineWidth', 1.8); hold(ax2,'on');
    if ~isinf(ulim)
        yline(ax2, +ulim, 'k--'); yline(ax2, -ulim, 'k--');
        title(ax2,'step r \rightarrow u (saturado)');
    else
        title(ax2,'step r \rightarrow u (sin sat)');
    end
    grid(ax2,'on'); grid(ax2,'minor');
    xlabel(ax2,'t [s]'); ylabel(ax2,'u');

    ax3 = subplot(2,2,[3 4]);
    hold(ax3,'on'); grid(ax3,'on'); grid(ax3,'minor'); axis(ax3,'equal');
    title(ax3,'Z-plane: planta + observador');
    xlabel(ax3,'Re\{z\}'); ylabel(ax3,'Im\{z\}');

    th = linspace(0,2*pi,400);
    plot(ax3, cos(th), sin(th), 'k:'); % unit circle

    plot(ax3, real(p_ol),  imag(p_ol),  'o', 'LineWidth', 1.5);  % poles plant A
    plot(ax3, real(p_cl),  imag(p_cl),  'x', 'LineWidth', 1.8);  % poles CL A-BK
    plot(ax3, real(p_obs), imag(p_obs), '^', 'LineWidth', 1.8);  % poles obs error

    if ~isempty(z_plant)
        plot(ax3, real(z_plant), imag(z_plant), 's', 'LineWidth', 1.5); % zeros plant
        legend(ax3,'unit circle','poles plant (A)','poles CL (A-BK)','poles obs','zeros plant','Location','bestoutside');
    else
        legend(ax3,'unit circle','poles plant (A)','poles CL (A-BK)','poles obs','Location','bestoutside');
    end

    xlim(ax3,[-1.2 1.2]); ylim(ax3,[-1.2 1.2]);
end
function [Nx,Nu,Nbar] = refi(phi,gam,Hr,K)
% REFI.M  computes command input matrices
% # of contols must be equal to # of outputs
% i.e., if gam = n x m, then H must be m x n
% [Nx,Nu,Nbar] = refi(phi,gam,Hr,K)

I=eye(size(phi));
[m,n]=size(Hr);
np=inv([phi-I gam;Hr zeros(m)])*([zeros(n,m);eye(m)]);
Nx=np(1:n,:);
Nu=np(n+1:n+m,:);
Nbar=Nu+K*Nx;
end