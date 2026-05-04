% Pend_invert_michigan.m
%
% MODO "PID-MICHIGAN" — Opción 2 (fiel al ejemplo de Michigan):
%   PID directo a PWM, bypass del lazo interno de velocidad.
%   El controlador externo recibe error de ángulo y escribe DIRECTO al PWM.
%
% Planta: FT completa de Michigan (3er orden, con fricción b e inercia I).
%   P_pend(s) = (ml/q) s
%               -----------------------------------------------------------
%               s^3 + (b(I+ml^2)/q) s^2 - ((M+m)mgl/q) s - bmgl/q
%   con  q = (M+m)(I+ml^2) - (ml)^2
%
% Conversión PWM_count -> Fuerza al carro [N]:
%   V       = (PWM_count / PWM_MAX) * V_sup
%   T_motor ~= Km*V/Ra      (régimen casi-estacionario, simplificación)
%   F_carro = T_motor / r_pinion
%   =>  F  ~=  kf * PWM_count   con  kf = Km*V_sup / (Ra*r_pinion*PWM_MAX)
%
% Salidas en el workspace (para la GUI psoc_pendulo_gui):
%   C_out_michigan         -> objeto pid() discreto, cargar como Controlador_out
%   Modelo_outer_michigan  -> objeto tf()  discreto, cargar como Modelo_out
%
% USO en psoc_pendulo_gui (con el modo "PID" agregado):
%   1. Marcar el checkbox  "PID  (Michigan: bypass inner)"  en el Lazo Externo.
%   2. "Cargar Controlador_out..."  -> C_out_michigan
%   3. "Cargar Modelo_out..."        -> Modelo_outer_michigan
%   4. Saturación: -1264 .. +1264
%   5. Iniciar -- el PID externo escribe directo al PWM, sin lazo interno.

clear; clc; close all;

%% ====================================================================
%  PARAMETROS FISICOS DEL PENDULO  (TUS valores medidos)
% =====================================================================
M = 0.288;       % [kg]   masa del carro / brazo
m = 0.0935;      % [kg]   masa del pendulo
l = 0.175;       % [m]    longitud al centro de masa
g = 9.8;         % [m/s^2]

% Inercia del pendulo (varilla uniforme respecto al CM)
I = (1/12) * m * (2*l)^2;     % [kg*m^2]

% Friccion del carro (parametrizable -- refinar con free-coast)
b = 0.05;        % [N*s/m]

%% ====================================================================
%  PARAMETROS DEL MOTOR  (Motor_3) -- conversion PWM -> Fuerza
% =====================================================================
Km      = 0.1863;    % [N*m/A]
Ra      = 4.04;      % [Ohm]
V_sup   = 12;        % [V]   tension de alimentacion
PWM_MAX = 1264;      % cuentas

% Radio efectivo piñon/brazo
% Medido por metodo "una vuelta": carro avanza 10.5 cm por vuelta del motor
%   r_pinion = 0.105 / (2*pi) = 0.01671 m
r_pinion = 0.105 / (2*pi);    % [m]  ~ 16.71 mm

kf = Km * V_sup / (Ra * r_pinion * PWM_MAX);   % [N por cuenta de PWM]

fprintf('========================================================\n');
fprintf('  MODELO MICHIGAN COMPLETO + PID DIRECTO A PWM\n');
fprintf('========================================================\n\n');

fprintf('-- Parametros fisicos --\n');
fprintf('  M = %.4f kg     m = %.4f kg     l = %.4f m\n', M, m, l);
fprintf('  I = %.4e kg*m^2  (varilla uniforme alrededor del CM)\n', I);
fprintf('  b = %.4f N*s/m   (estimado -- ajustar con free-coast)\n\n', b);

fprintf('-- Conversion PWM -> Fuerza --\n');
fprintf('  V_sup    = %.2f V\n', V_sup);
fprintf('  r_pinion = %.4f m  (¡AJUSTAR!)\n', r_pinion);
fprintf('  kf       = %.6f N/cuenta\n', kf);
fprintf('  F_max    ~= %.3f N a PWM = %d\n\n', kf*PWM_MAX, PWM_MAX);

%% ====================================================================
%  FT COMPLETA DE MICHIGAN  (3er orden, entrada=F [N], salida=phi [rad])
% =====================================================================
q = (M + m)*(I + m*l^2) - (m*l)^2;
s = tf('s');

num_p = (m*l/q) * [1 0];
den_p = [1, ...
         b*(I + m*l^2)/q, ...
        -(M + m)*m*g*l/q, ...
        -b*m*g*l/q];
P_pend_F = tf(num_p, den_p);          % [rad / N]

fprintf('-- Planta Michigan  P_pend(s)  [rad/N] --\n');
P_pend_F                                                                    %#ok<NOPTS>
fprintf('  Polos en lazo abierto:\n');
disp(pole(P_pend_F));

%% ====================================================================
%  PLANTA  phi / PWM_count  (incluye conversion kf)
% =====================================================================
P_pwm = kf * P_pend_F;                % [rad / cuenta de PWM]

%% ====================================================================
%  PERIODO DE MUESTREO
% =====================================================================
Ts = 4e-3;        % 5 ms -> 200 Hz (igual que tu cascada actual)

%% ====================================================================
%  DISEÑO DEL PID  --  estilo Armstrong/Michigan (manual)
% =====================================================================
% NOTA SOBRE pidtune:
%   La planta Michigan tiene un CERO en s=0 (factor "ml*s") y un POLO
%   inestable en ~+7 rad/s. pidtune entra en conflicto entre:
%     - PDF: no puede mantener ganancia DC (warning "Cannot keep loop gain")
%     - PIDF: el integrador se cancela con el cero del plant (polo marginal)
%   Por eso usamos sintonia MANUAL estilo "Armstrong" -- es la que aplicaron
%   los alumnos del semestre anterior (Completo.cydsn) con planta casi igual,
%   y funciona en hardware aunque la teoria lineal diga "marginal".
%
%   El sistema vive en SATURACION cuando |theta| es grande (PWM en ±1264),
%   y solo es lineal cerca del equilibrio invertido.

% Forma estandar del PID:  C(s) = Kp*(1 + 1/(Ti*s) + Td*s/(Td/N*s + 1))
% Convencion de los alumnos (Armstrong, salida en VOLTIOS): Kp=120, Ti=0.06, Td=0.055.

% Parametros base (en VOLTIOS, como Armstrong)
Kp_volt = 150;        % [V/rad]
Ti      = 0.04*150/120;       % [s]
Td      = 0.09*120/150;      % [s]
N_filt  = 10;         % filtro derivativo

% % Parametros base (en VOLTIOS, como Armstrong)
% Kp_volt = 170;        % [V/rad]
% Ti      = 0.0275*170/120;       % [s]
% Td      = 0.09*120/170;      % [s]
% N_filt  = 10;         % filtro derivativo

% % Parametros base (en VOLTIOS, como Armstrong)
% Kp_volt = 170;        % [V/rad]
% Ti      = 0.0389583;       % [s]
% Td      = 0.0635294;      % [s]
% N_filt  = 10;         % filtro derivativo

% % % % % % % tf(2,1,0.0025)

% Conversion V -> PWM_count:  Kp_pwm = Kp_volt * (PWM_MAX / V_sup)
v2pwm = PWM_MAX / V_sup;
Kp = Kp_volt * v2pwm;
Ki = (Kp_volt / Ti) * v2pwm;
Kd = (Kp_volt * Td) * v2pwm;
Tf = Td / N_filt;     % constante de filtro derivativo

% Construir PID continuo en forma "parallel"
C_cont = pid(Kp, Ki, Kd, Tf);

fprintf('-- Sintonia Armstrong escalada a PWM (planta similar a alumnos) --\n');
fprintf('  Equivalente en V:  Kp=%.1f Ti=%.4f Td=%.4f  N=%d\n', ...
        Kp_volt, Ti, Td, N_filt);
fprintf('  Conversion v2pwm = PWM_MAX/V_sup = %.2f\n', v2pwm);
fprintf('  En PWM_count (forma parallel  C = Kp + Ki/s + Kd*s/(Tf*s+1)):\n');
fprintf('  Kp = %+.6e\n', C_cont.Kp);
fprintf('  Ki = %+.6e\n', C_cont.Ki);
fprintf('  Kd = %+.6e\n', C_cont.Kd);
fprintf('  Tf = %.6e\n\n', C_cont.Tf);

% Verificacion REAL de estabilidad del lazo cerrado.
% NOTA: la funcion margin() puede dar advertencias falsas con plantas
% inestables (la nuestra tiene polo en +7.16 rad/s). El test fiable es
% mirar los polos del lazo cerrado directamente.
L_open = C_cont * P_pwm;
T_cl   = feedback(L_open, 1);
poles_cl = pole(T_cl);

fprintf('-- Polos del LAZO CERRADO (deben tener Re<0) --\n');
for kk = 1:numel(poles_cl)
    fprintf('   p_%d = %+.3f %+.3fi\n', kk, real(poles_cl(kk)), imag(poles_cl(kk)));
end
if all(real(poles_cl) < 0)
    fprintf('   ==> LAZO CERRADO ESTABLE\n');
else
    fprintf('   ==> ATENCION: LAZO CERRADO INESTABLE\n');
end

% Margenes (informativos solamente -- pueden ser enganhosos en plantas inestables)
warning('off','Control:analysis:MarginUnstable');
[Gm, Pm, Wgm, Wpm] = margin(L_open);
warning('on','Control:analysis:MarginUnstable');
fprintf('-- Margenes (informativos, no fiables con planta inestable) --\n');
if isinf(Gm)
    fprintf('  Gm = inf\n');
else
    fprintf('  Gm = %+.2f dB  @ %.2f rad/s\n', 20*log10(Gm), Wgm);
end
fprintf('  Pm = %+.2f deg @ %.2f rad/s\n\n', Pm, Wpm);

%% ====================================================================
%  DISCRETIZACION  Tustin a Ts
% =====================================================================
% c2d con 'tustin' devuelve directamente un pid() discreto VALIDO
% (con DFormula='Trapezoidal' que no tiene la restriccion Ts<2*Tf).
% NO reconstruir con pid(...,Ts,...) porque ese constructor usa
% ForwardEuler por defecto y exige Ts < 2*Tf, que aqui no se cumple.
C_out_michigan = c2d(C_cont, Ts, 'tustin');
C_d = C_out_michigan;

fprintf('-- PID discreto (Tustin, Ts = %.1f ms) --\n', Ts*1e3);
fprintf('  Kp = %+.6e\n', C_out_michigan.Kp);
fprintf('  Ki = %+.6e   (en convencion continua; PSoC usa Ki*Ts -> %+.6e)\n', ...
        C_out_michigan.Ki, C_out_michigan.Ki*Ts);
fprintf('  Kd = %+.6e\n', C_out_michigan.Kd);
fprintf('  Tf = %+.6e\n\n', C_out_michigan.Tf);

%% ====================================================================
%  MODELO DE SIMULACION DISCRETO  phi / PWM_count
% =====================================================================
Modelo_outer_michigan = c2d(P_pwm, Ts, 'zoh');

fprintf('-- Modelo outer discreto  phi/PWM (Ts = %.1f ms) --\n', Ts*1e3);
Modelo_outer_michigan                                                       %#ok<NOPTS>

%% ====================================================================
%  COMPARACION:  planta actual (2do orden, sin b sin I)  vs  Michigan
% =====================================================================
P_actual_F_equiv = tf(-1/(M*l), [1 0 -((M+m)*g/(M*l))]);   % phi vs ¿v?
% Para comparar Bode con misma entrada (forzamos a phi/F equivalente):
%   tu planta original tiene la entrada en velocidad/aceleracion del carro,
%   no en fuerza, asi que esta comparacion es solo cualitativa.

figure('Name','Bode comparativo: planta actual vs Michigan');
bode(P_actual_F_equiv, P_pend_F);
grid on;
legend('P_{actual}(s)  (2do orden, sin b sin I)', ...
       'P_{Michigan}(s)  (3er orden, con b e I)', ...
       'Location','best');
title('Pendulo invertido -- comparacion de modelos');

%% ====================================================================
%  RESPUESTA ANTE PERTURBACION IMPULSIVA  (estilo Michigan)
% =====================================================================
%   ref = 0,  perturbacion de fuerza F al carro,
%   T_dist(s) = phi/F = P_pend_F / (1 + C_cont * kf * P_pend_F)
T_dist = P_pend_F / (1 + C_cont * kf * P_pend_F);

t = 0:Ts:5;
figure('Name','Respuesta a impulso de fuerza -- PID Michigan');
impulse(T_dist, t); grid on;
ylabel('\phi  [rad]'); xlabel('Tiempo [s]');
title(sprintf(['Pendulo phi ante impulso F = 1 N*s\n' ...
               'Sintonia Armstrong:  Kp_V=%.0f  Ti=%.3fs  Td=%.3fs  (escalado a PWM)'], ...
              Kp_volt, Ti, Td));

%% ====================================================================
%  INSTRUCCIONES PARA USAR EN LA GUI
% =====================================================================
fprintf('\n========================================================\n');
fprintf(' COMO USAR EN  psoc_pendulo_gui\n');
fprintf('========================================================\n');
fprintf(' Modo PID  (Michigan: bypass del lazo interno de velocidad)\n\n');
fprintf('   1. Marcar checkbox  "PID  (Michigan: bypass inner)"\n');
fprintf('      en el panel del Lazo Externo.\n');
fprintf('   2. "Cargar Controlador_out..."  ->  C_out_michigan\n');
fprintf('   3. "Cargar Modelo_out..."        ->  Modelo_outer_michigan\n');
fprintf('   4. Saturacion:  sat_min = -%d   sat_max = +%d\n', PWM_MAX, PWM_MAX);
fprintf('   5. Iniciar.\n\n');

fprintf(' AJUSTES si la respuesta no es satisfactoria:\n');
fprintf('   - Kp_volt (linea 123):  subir si lento, bajar si oscila.  Default 120.\n');
fprintf('   - Ti       (linea 124):  bajar para corregir error mas rapido.\n');
fprintf('   - Td       (linea 125):  subir para mas amortiguamiento.\n');
fprintf('   - Para Michigan literal (Kp=100 Ki=1 Kd=20 sobre F):\n');
fprintf('       Kp = 100/kf  Ki = 1/kf  Kd = 20/kf  (sin v2pwm).\n');
fprintf('========================================================\n');
