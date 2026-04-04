%% probar_todas_tecnicas_diseno_robusto.m
% NO cierra figuras. NO hace clear agresivo.

clc;

% ======= CONFIG =======
fs   = 500;     % Hz
Nsim = 400;
subN = 30;

% Ajustá u_sat a tu actuador real:
%  - duty 0..1: [0 1]
%  - PWM 0..255: [0 255]
%  - si querés ver lineal sin sat: [-Inf Inf]
u_sat = [-Inf Inf];

% Técnicas a probar (sin Control System Designer)
tecs = { ...
    'opt_intra', ...
    'opt_no_intra', ...
    'pp_obs', ...
    'pp_obs_int', ...
    'dlqr', ...
    'lqg', ...
    'dlqr_int', ...
    'lqg_int' ...
};

% ======= TOMAR PLANTA DESDE tf4 =======
if ~exist('modelo','var')
    error('No existe modelo en workspace. Definila primero (tf/ss/zpk o idtf/idss).');
end

G_in = modelo;
fprintf('\n=== Diagnóstico planta ===\n');
fprintf('class(tf4) = %s\n', class(G_in));

% Convertir a tf/ss/zpk si viene de identificación (idtf/idss)
G = convert_to_lti(G_in);

fprintf('Planta convertida: %s | order=%d | SISO=%d\n', class(G), order(G), isSISO(G));

% ======= LOOP =======
S_all = cell(numel(tecs),1);

for i = 1:numel(tecs)
    tec = tecs{i};

    fprintf('\n------------------------------------------------------------\n');
    fprintf('[%d/%d] tecnica = %s\n', i, numel(tecs), tec);
    fprintf('ENTER = diseñar e imprimir coeficientes | q = salir\n');
    s = input('> ', 's');
    if strcmpi(strtrim(s),'q')
        fprintf('Cancelado.\n');
        break;
    end

    try
        % Para métodos SS tu GUI es 2 estados. Si planta >2, reducimos.
        G_use = G;
        if is_ss_method(tec)
            if order(G_use) ~= 2
                fprintf('Nota: planta orden %d -> reduzco a orden 2 (balred) para compatibilidad con GUI SS.\n', order(G_use));
                try
                    G_use = balred(ss(G_use), 2);
                catch
                    % fallback: tomar modo dominante por minreal/ss (si balred no está)
                    G_use = ss(G_use);
                    warning('balred falló/no disponible. Queda ss sin reducción (puede romper GUI SS).');
                end
            else
                G_use = ss(G_use);
            end
        end

        % Diseñar con tu función
        S = disenar_control(G_use, fs, ...
            'tecnica', tec, ...
            'N', Nsim, ...
            'subN', subN, ...
            'u_sat', u_sat ...
        );

        S_all{i} = S;

        % ==== IMPRESIÓN COPY/PASTE ====
        fprintf('\n=== RESULTADO: %s ===\n', tec);
        fprintf('fs=%.6g Hz | Ts=%.9g s\n', S.fs, S.Ts);

        % Mostrar Gd
        try
            Gd_tf = tf(S.Gd);
            [numG, denG] = tfdata(Gd_tf,'v');
            fprintf('\nGd(z) (ZOH):\n');
            fprintf('numG = %s;\n', vec2str(numG));
            fprintf('denG = %s;\n', vec2str(denG));
        catch
        end

        if ~isempty(S.Cd)
            print_tf_for_psoc(S.Cd);

        elseif ~isempty(S.ctrl)
            print_ss_for_psoc(S);

        else
            fprintf('(Sin controlador en S.Cd ni S.ctrl — en esa técnica quizá solo abrió herramienta.)\n');
        end

        % Dejar el último en workspace
        S_last = S; %#ok<NASGU>
        assignin('base','S_last',S);
        assignin('base','Cd_last',S.Cd);
        assignin('base','ctrl_last',S.ctrl);

        fprintf('\nListo. (S_last/Cd_last/ctrl_last en workspace)\n');

    catch e
        fprintf(2,'\nERROR en %s: %s\n', tec, e.message);
        fprintf(2,'Tip: mirá class(tf4) y que sea tf/ss/zpk (o idtf/idss convertible).\n');
    end

    fprintf('\n(ENTER para seguir al siguiente)\n');
    input('');
end

assignin('base','S_all',S_all);
fprintf('\nFin. S_all guardado en workspace.\n');

%% ================= helpers =================

function G = convert_to_lti(Gin)
    % Acepta tf/ss/zpk o modelos de identificación: idtf/idss/idpoly
    if isa(Gin,'tf') || isa(Gin,'ss') || isa(Gin,'zpk')
        G = Gin;
        return;
    end

    if isa(Gin,'idtf')
        G = tf(Gin);
        return;
    end

    if isa(Gin,'idss')
        G = ss(Gin);
        return;
    end

    if isa(Gin,'idpoly')
        % 1) convertir idpoly -> idtf (más limpio para SISO)
        try
            Gid = idtf(Gin);    % requiere SysID Toolbox
            G   = tf(Gid);      % ya queda como LTI
            return;
        catch
            % 2) fallback: usar polydata y armar tf manual
            [A,B,C,D,F,Ts] = polydata(Gin);
            % modelo ARX/ARMAX típico: y = (B/F)u + ...  (ignoramos ruido)
            % Construimos G(z) = B(z^-1)/A(z^-1) por default si no hay F.
            if isempty(F) || all(F==0)
                num = B; den = A;
            else
                num = conv(B, A*0 + 1); %#ok<NASGU>
                % más conservador: usar B/F y A en serie suele confundir, así que:
                num = B; den = conv(A, F);
            end

            Ts_ = Gin.Ts;
            if Ts_ == 0
                error('idpoly tiene Ts=0 (continua). Tu disenar_control espera planta continua, pero luego discretiza; revisá esto.');
            end

            % Ojo: polydata devuelve polinomios en z^-1. tf usa por defecto z^-1 en discreto, OK.
            G = tf(num, den, Ts_);
            return;
        end
    end

    error('tf4 no es tf/ss/zpk ni idtf/idss/idpoly. class=%s', class(Gin));
end

function yes = isSISO(sys)
    try
        [ny,nu] = size(sys);
        yes = (ny==1 && nu==1);
    catch
        yes = false;
    end
end

function yes = is_ss_method(tec)
    tec = lower(string(tec));
    yes = any(tec == ["pp_obs","pp_obs_int","dlqr","lqg","dlqr_int","lqg_int"]);
end

function s = vec2str(v)
    v = double(v(:).');
    s = ['[', sprintf(' %.12g', v), ' ]'];
end

function v6 = pad6(v)
    v = double(v(:).');
    if numel(v) >= 6
        v6 = v(1:6);
    else
        v6 = [v, zeros(1,6-numel(v))];
    end
end

function print_tf_for_psoc(Cd)
    Cd = tf(Cd);
    [num, den] = tfdata(Cd,'v');

    % Normalizar a0=1 si se puede
    if abs(den(1)) > 0
        num = num/den(1);
        den = den/den(1);
    end

    b6 = pad6(num);
    a6 = pad6(den);

    fprintf('\n--- Cd(z) TF (para pegar en GUI TF) ---\n');
    fprintf('b0..b5 = %s;\n', vec2str(b6));
    fprintf('a0..a5 = %s;\n', vec2str(a6));
    fprintf('Nota: a0 debería ser 1.\n');
end

function print_ss_for_psoc(S)
    [A,B,C,D] = ssdata(ss(S.Gd));

    ctrl = S.ctrl;

    fprintf('\n--- Planta SS discreta (para pegar en GUI SS) ---\n');
    fprintf('A = \n'); disp(A);
    fprintf('B = \n'); disp(B);
    fprintf('C = \n'); disp(C);
    fprintf('D = \n'); disp(D);

    fprintf('\n--- Ganancias control (para pegar) ---\n');

    if isfield(ctrl,'K') && ~isempty(ctrl.K)
        fprintf('K = \n'); disp(ctrl.K);
    else
        fprintf('K = []\n');
    end

    if isfield(ctrl,'L') && ~isempty(ctrl.L)
        fprintf('L = \n'); disp(ctrl.L);
    else
        fprintf('L = []\n');
    end

    if isfield(ctrl,'Ki') && ~isempty(ctrl.Ki)
        fprintf('Ki = %g\n', double(ctrl.Ki));
    else
        fprintf('Ki = 0\n');
    end

    if isfield(ctrl,'Kr') && ~isempty(ctrl.Kr)
        fprintf('Kr = %g\n', double(ctrl.Kr));
    else
        fprintf('Kr = 1\n');
    end
end
