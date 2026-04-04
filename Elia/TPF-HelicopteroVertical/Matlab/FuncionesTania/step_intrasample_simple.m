function out = step_intrasample_simple(plantaC, plantaD, compensador, varargin)
% step_intrasample_simple(plantaC, plantaD, compensador, ...)
% - esfuerzo: u[k] = step( feedback(C, Pd) )  (transfer R->U)
% - salida discreta: y[k] = step( feedback(C*Pd,1) ) (transfer R->Y)
% - intramuestra: arma u(t) por ZOH desde u[k] y simula y(t)=lsim(Pc,u(t))
%
% Name-Value:
%   'Tfinal'     (default 5)
%   'Oversample' (default 50)   % puntos por Ts para ver intramuestra
%   'StepAmp'    (default 1)
%   'Plot'       (default true)

p = inputParser;
p.addParameter('Tfinal', 5, @(x)isnumeric(x)&&isscalar(x)&&x>0);
p.addParameter('Oversample', 50, @(x)isnumeric(x)&&isscalar(x)&&x>=2);
p.addParameter('StepAmp', 1, @(x)isnumeric(x)&&isscalar(x));
p.addParameter('Plot', true, @(x)islogical(x)&&isscalar(x));
p.parse(varargin{:});
opt = p.Results;

% --- Ts ---
Ts = plantaD.Ts;
if isempty(Ts) || Ts<=0
    error('plantaD debe tener Ts válido.');
end
if compensador.Ts ~= Ts
    error('Ts del compensador (%.6g) != Ts de plantaD (%.6g).', compensador.Ts, Ts);
end

Tu = feedback(compensador, plantaD);
Ty = feedback(compensador*plantaD, 1);

% --- FIX: volverlos causales si quedaron impropios ---
[Tu, dTu] = make_causal_discrete(Tu);
[Ty, dTy] = make_causal_discrete(Ty);

fprintf('Causalizado: Tu delay=%d muestras, Ty delay=%d muestras\n', dTu, dTy);

% --- Simulación discreta a instantes kTs ---
t_s = (0:Ts:opt.Tfinal)';     % tiempos de muestreo
u_k = opt.StepAmp * step(Tu, t_s);
y_k = opt.StepAmp * step(Ty, t_s);

% --- Construir u(t) con ZOH (intramuestra) ---
M  = opt.Oversample;
dt = Ts/M;

t  = (0:dt:opt.Tfinal)';      % tiempo continuo fino

% ZOH: u(t) = u_k(k) para t en [kTs, (k+1)Ts)
u_t = interp1(t_s, u_k, t, 'previous', 'extrap');

% --- Simular planta continua con ese u(t) ---
y_t = lsim(plantaC, u_t, t);

% --- Salida ---
out = struct();
out.Ts  = Ts;
out.t   = t;
out.u_t = u_t;
out.y_t = y_t;
out.t_s = t_s;
out.u_k = u_k;
out.y_k = y_k;

% --- Plots ---
if opt.Plot
    figure('Name','Esfuerzo: u[k] y u(t) ZOH');
    plot(t, u_t, 'LineWidth', 1.2); hold on;
    stem(t_s, u_k, 'filled');
    grid on;
    xlabel('Tiempo [s]'); ylabel('u');
    legend('u(t) ZOH','u[k]','Location','best');

    figure('Name','Salida: y(t) intramuestra + y[k] discreto');
    plot(t, y_t, 'LineWidth', 1.2); hold on;
    stem(t_s, y_k, 'filled');
    grid on;
    xlabel('Tiempo [s]'); ylabel('y');
    legend('y(t) (planta continua con u(t))','y[k] (lazo discreto)','Location','best');
end
end
