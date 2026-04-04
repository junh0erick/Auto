function out = pid_digital_from_mat(matFile, Ts, Kp, Ti, Td, N, varargin)
% pid_digital_from_mat
%  - Carga un .mat y detecta una planta LTI G (tf/ss/zpk).
%  - Discretiza la planta con ZOH (modelo de retenedor).
%  - Construye el PID continuo tipo Åström (con filtro derivativo N) y lo discretiza
%    con Tustin (opcional prewarp).
%  - Grafica step cerrado y esfuerzo u(t).
%  - Exporta (guarda) G, Gd, C, Cd, T, U y señales en un .mat de log.
%
% Uso:
%   out = pid_digital_from_mat("planta.mat", 0.01, -15.5, 2.5, 0.4, 10);
%
% Opciones (name-value):
%   "PrewarpHz"   : frecuencia (Hz) para prewarp de Tustin (default: [])
%   "RefAmp"      : amplitud del step de referencia (default: 1)
%   "Plot"        : true/false (default: true)
%   "LogFile"     : nombre de archivo de salida (default: <matFile>_pid_digital_log.mat)

    p = inputParser;
    addParameter(p, "PrewarpHz", [], @(x) isempty(x) || (isscalar(x) && x>0));
    addParameter(p, "RefAmp", 1, @(x) isscalar(x));
    addParameter(p, "Plot", true, @(x) islogical(x) && isscalar(x));
    addParameter(p, "LogFile", "", @(x) isstring(x) || ischar(x));
    parse(p, varargin{:});
    opt = p.Results;

    if ~isfile(matFile)
        error("No existe el archivo: %s", matFile);
    end
    if Ts <= 0
        error("Ts debe ser > 0.");
    end
    if Ti <= 0 || N <= 0 || Td < 0
        error("Revisá parámetros: Ti>0, N>0, Td>=0.");
    end

    S = load(matFile);
    [G, gName] = find_first_lti(S);
    fprintf("\n[OK] Planta detectada: %s (%s)\n", gName, class(G));

    % --- Discretización de planta (modelo de retenedor) ---
    Gd = c2d(G, Ts, "zoh");

    % --- PID Åström continuo ---
    % C(s) = Kp * (Ti*Td*s^2 + (Ti + Td/N)*s + 1) / (Ti*Td/N*s^2 + Ti*s)
    numC = Kp * [Ti*Td, (Ti + Td/N), 1];
    denC = [Ti*Td/N, Ti, 0];
    C = tf(numC, denC);

    % --- Discretización del controlador (algoritmo digital) ---
    if isempty(opt.PrewarpHz)
        Cd = c2d(C, Ts, "tustin");
        fprintf("[OK] Controlador discretizado con Tustin (sin prewarp).\n");
    else
        wc = 2*pi*opt.PrewarpHz; % rad/s
        Cd = c2d(C, Ts, "tustin", wc);
        fprintf("[OK] Controlador discretizado con Tustin + prewarp a %.3f Hz.\n", opt.PrewarpHz);
    end

    % --- Lazo cerrado digital: y/r y u/r ---
    % y/r = feedback(Gd*Cd, 1)
    Td_cl = feedback(Gd*Cd, 1);
    % u/r = feedback(Cd, Gd)  (porque u = Cd*(r - y), y = Gd*u)
    Ud_cl = feedback(Cd, Gd);

    % --- Simulación discreta (lsim sirve para sistemas discretos) ---
    tFinal = auto_tfinal(Td_cl, Ts);
    t = (0:Ts:tFinal).';
    r = opt.RefAmp * ones(size(t));

    y = lsim(Td_cl, r, t);
    u = lsim(Ud_cl, r, t);

    % Métricas rápidas
    u_peak = max(abs(u));
    fprintf("[INFO] u(t): min=%.6g | max=%.6g | pico_abs=%.6g\n", min(u), max(u), u_peak);

    % --- Plots ---
    if opt.Plot
        figure("Name","Step lazo cerrado (digital) y(t)");
        stairs(t, y, "LineWidth", 1.3); grid on;
        xlabel("t [s]"); ylabel("y[k]");
        title("Salida lazo cerrado (digital) ante step r");

        figure("Name","Esfuerzo de control (digital) u(t)");
        stairs(t, u, "LineWidth", 1.3); grid on;
        xlabel("t [s]"); ylabel("u[k]");
        title("Esfuerzo de control (digital) ante step r");
    end

    % --- Export / Log ---
    out = struct();
    out.matFile = string(matFile);
    out.plantVarName = gName;
    out.Ts = Ts;

    out.Kp = Kp; out.Ti = Ti; out.Td = Td; out.N = N;
    out.PrewarpHz = opt.PrewarpHz;

    out.G  = G;
    out.Gd = Gd;
    out.C  = C;
    out.Cd = Cd;

    out.Td_cl = Td_cl;
    out.Ud_cl = Ud_cl;

    out.t = t;
    out.r = r;
    out.y = y;
    out.u = u;

    out.u_min = min(u);
    out.u_max = max(u);
    out.u_peak_abs = u_peak;

    if strlength(string(opt.LogFile)) == 0
        logFile = replace(string(matFile), ".mat", "") + "_pid_digital_log.mat";
    else
        logFile = string(opt.LogFile);
    end

    save(logFile, "-struct", "out");
    fprintf("[OK] Log guardado en: %s\n\n", logFile);
end

% ========================= Helpers =========================

function [G, name] = find_first_lti(S)
    fn = fieldnames(S);

    preferred = ["G","P","plant","sys","SYSTEM","System","Planta","Gp"];
    for k = 1:numel(preferred)
        if isfield(S, preferred(k)) && isa(S.(preferred(k)), "lti")
            G = S.(preferred(k));
            name = string(preferred(k));
            return;
        end
    end

    for k = 1:numel(fn)
        v = S.(fn{k});
        if isa(v, "lti")
            G = v;
            name = string(fn{k});
            return;
        end
    end

    error("No encontré ninguna planta LTI (tf/ss/zpk) dentro del .mat. Variables: %s", ...
          strjoin(string(fn), ", "));
end

function tFinal = auto_tfinal(Td_cl, Ts)
    % Estima horizonte por polos (discretos)
    try
        p = pole(Td_cl);
        mag = abs(p);
        mag = mag(mag < 1); % estables
        if isempty(mag)
            tFinal = 5;
        else
            % polo más cercano al 1 => más lento
            a = max(mag);
            % aprox: a^k ~ exp(-k*(1-a)) para a~1 -> tau_k ~ 1/(1-a)
            tau_samples = 1/max(1e-6, (1 - a));
            tFinal = min(max(8*tau_samples*Ts, 1), 60);
        end
    catch
        tFinal = 5;
    end
end
