%% ============================================================
%  Truxal–Ragazzini (síntesis directa) - versión CORREGIDA
%  - Unifica en variable z para evitar mezclar z^-1 con z
%  - Regulariza causalidad si el esfuerzo queda impropio
% ============================================================

close all; clear; clc;

%% ====== 0) CONFIG: Ts ======
Ts = 1/1000;

%% ====== 1) PLANTA dada en z^-1 (tal como tu identificación) ======
Gm = tf([0 -0.0001205 0.0002415 -0.0001209], ...
        [1 -2.994 2.989 -0.9944], Ts, 'Variable','z^-1');
Gm = minreal(Gm, 1e-12);

disp('=== Planta original (en z^-1) ===');
Gm
zpk(Gm)

%% ====== 2) Convertir a forma equivalente en variable z ======
% Si: G(z) = (b1 z^-1 + b2 z^-2 + b3 z^-3) / (1 + a1 z^-1 + a2 z^-2 + a3 z^-3)
% Multiplico por z^3:
% G(z) = (b1 z^2 + b2 z + b3) / (z^3 + a1 z^2 + a2 z + a3)

b = [-0.0001205  0.0002415  -0.0001209];   % coef de z^-1..z^-3
a = [1 -2.994 2.989 -0.9944];              % 1 + a1 z^-1 + a2 z^-2 + a3 z^-3

num_z = [b 0];          % b1 z^2 + b2 z + b3 + 0*z^3 (para alinear orden)
den_z = a;              % z^3 + a1 z^2 + a2 z + a3  (ya está en orden)

G = tf(num_z, den_z, Ts, 'Variable','z');
G = minreal(G, 1e-12);

disp('=== Planta equivalente (en z) ===');
G
zpk(G)

% Verificación visual: deberían pisarse (misma dinámica, distinto "nombre")
opt = bodeoptions; opt.PhaseWrapping='off'; opt.FreqUnits='rad/s';
w = logspace(0,4,2000);
figure('Name','Chequeo G(z^-1) vs G(z)'); 
bodeplot(Gm, w, opt); hold on; bodeplot(G, w, opt);
grid on; grid minor; legend('G en z^{-1}','G en z');

%% ====== 3) Gráficos de la planta ======
figure('Name','Planta Bode'); bodeplot(G, w, opt); grid on; grid minor;
figure('Name','Planta PZ'); pzmap(G); grid on;
figure('Name','Planta RLocus'); rlocus(G); zgrid on;

%% ============================================================
% MÉTODO 1: Gcl(z)=z^-1  =>  C1 = (1/G) * 1/(z-1)
%% ============================================================
z = tf([1 0], 1, Ts, 'Variable','z');

F1 = 1/(z - 1);
C1 = minreal((1/G) * F1, 1e-9);

disp('=== C1 (Método 1) ===');
C1
zpk(C1)

L1  = minreal(C1*G, 1e-9);
T1y = feedback(L1, 1);
T1u = feedback(C1, G);          % r->u

figure('Name','M1: Bode L'); bodeplot(L1, w, opt); grid on; grid minor;
figure('Name','M1: Step salida'); step(T1y); grid on; grid minor;
figure('Name','M1: Step esfuerzo'); step(T1u); grid on; grid minor;

%% ============================================================
% MÉTODO 2: Deadbeat / ripple-free (regularizado para causalidad)
% Plantilla: se calcula C2 como lo habías hecho, pero:
% - se verifica realizabilidad
% - si el esfuerzo T2u queda impropio, se agrega retardo mínimo a C2
%% ============================================================

% Polos de la planta (en z)
p = zpk(G).P{1};
has_pole_at_1 = any(abs(p - 1) < 1e-6);

% Polinomio D(z) = (z - p2)(z - p3)... sin el (z-1) si existe
p_no1 = p(abs(p - 1) >= 1e-6);
Dz = poly(p_no1);
Dz = real(Dz);

% Coeficientes del numerador "efectivo" (b1,b2,...) para b0,K
% Usamos b1 y b2 (equivalente a lo que querías: orden 2 efectiva)
b1 = b(1);
b2 = b(2);

denomK = (b1 + b2);
if abs(denomK) < 1e-12
    error('b1+b2 ~ 0: no se puede calcular K y b0 con este esquema.');
end

K  = 1/denomK;
b0 = b1/denomK;   % ojo: acá es distinto a tu versión (consistencia con forma en z)

% Denominador base (z + b0)
Bz = [1 b0];

if has_pole_at_1
    denC2 = Bz;
else
    denC2 = conv([1 -1], Bz);
end

C2 = minreal(tf(K*Dz, denC2, Ts, 'Variable','z'), 1e-9);

disp('=== C2 (Método 2, antes de regularizar) ===');
C2
zpk(C2)

L2  = minreal(C2*G, 1e-9);
T2y = feedback(L2, 1);
T2u = feedback(C2, G);

% ---- Regularización de causalidad: si T2u es impropio, agrego retardo a C2
if ~isproper(T2u)
    [nu, du] = tfdata(T2u,'v');
    degN = length(nu)-1;
    degD = length(du)-1;
    d = max(1, degN - degD);   % retardo mínimo (>=1 por seguridad)

    fprintf('T2u impropio. Agrego retardo d=%d muestras a C2 para hacerlo causal.\n', d);
    C2 = C2 / (z^d);

    % Recalcular
    L2  = minreal(C2*G, 1e-9);
    T2y = feedback(L2, 1);
    T2u = feedback(C2, G);
end

disp('--- PROPIEDADES T2u (final) ---')
disp(['isproper: ', num2str(isproper(T2u))]);
disp(['isstable: ', num2str(isstable(T2u))]);

figure('Name','M2: Bode L'); bodeplot(L2, w, opt); grid on; grid minor;
figure('Name','M2: Step salida'); step(T2y); grid on; grid minor;
figure('Name','M2: Step esfuerzo'); step(T2u); grid on; grid minor;

%% ====== Comparaciones ======
figure('Name','Comparación salida'); step(T1y, T2y); grid on; grid minor;
legend('M1','M2','Location','best');

figure('Name','Comparación esfuerzo'); step(T1u, T2u); grid on; grid minor;
legend('u M1','u M2','Location','best');

disp('Listo. Ahora el script no mezcla variables y evita no-causalidad en esfuerzo.');
%% 
