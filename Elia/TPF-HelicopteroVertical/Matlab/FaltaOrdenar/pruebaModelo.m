S = tfmini_plus_calibrate_models();   % o con ruta si querés
T = S.T;

% --- sacar el modelo "best" ---
% (ajustá esta línea si tu struct usa otro campo)
mdl = S.best.model;   % debería ser un LinearModel (fitlm)

% Ruta de export (cambiala a tu proyecto si querés)
out_h = fullfile(pwd, 'tfmini_calib_coeffs.h');

export_fitlm_to_c_header(mdl, out_h, 'tfmini_correct_distance_cm');
fprintf('OK: Header exportado en: %s\n', out_h);

% --- predicción del modelo ---
Dist_pred = predict(mdl, T);

% --- vectores ---
Dist_real = T.Dist_real;
Dist_cm   = T.Dist_cm;

% --- ordenar por distancia real para que se vea “limpio” ---
[Dist_real_s, idx] = sort(Dist_real);
Dist_cm_s   = Dist_cm(idx);
Dist_pred_s = Dist_pred(idx);

% --- (opcional) downsample si tenés miles de puntos y se ve pesado ---
step = max(1, floor(numel(idx)/2000));   % deja ~2000 puntos
ii = 1:step:numel(idx);

figure; hold on; grid on;

% 1) sensor vs real (línea)
plot(Dist_real_s(ii), Dist_cm_s(ii), '.', 'MarkerSize', 8);

% 2) modelo vs real (línea)
plot(Dist_real_s(ii), Dist_pred_s(ii), '.', 'MarkerSize', 8);

% 3) línea ideal y=x
mn = min(Dist_real_s(ii));
mx = max(Dist_real_s(ii));
plot([mn mx], [mn mx], 'k--', 'LineWidth', 1.5);

xlabel('Dist\_real [cm]');
ylabel('Distancia [cm]');
title('Comparación: real vs sensor vs modelo');
legend({'Sensor (Dist\_cm)', 'Modelo (pred)', 'Ideal y=x'}, 'Location', 'best');



function export_fitlm_to_c_header(mdl, out_h, funcName)
%EXPORT_FITLM_TO_C_HEADER Exporta un LinearModel (fitlm) a un .h con macros + función C.
%
% Soporta:
%   - Términos lineales: Dist_cm, Freq_prom, Temp_C, Lux, Lux_log, Lux_log1p
%   - Interacciones: A:B (producto)
%   - Términos I(var.^2) (cuadráticos) generados por fitlm con fórmulas tipo I(Dist_cm.^2)
%
% Importante:
%   - Si tu modelo incluye Lux_log o Lux_log1p, NO calculamos log en C.
%     La idea es que en runtime le pases ya calculado Lux_log/Lux_log1p
%     (o elijas un modelo sin log). Por eso el wrapper tiene parámetros extra.

    if nargin < 3 || isempty(funcName)
        funcName = 'tfmini_correct_distance_cm';
    end

    % --- coeficientes ---
    coefNames = string(mdl.Coefficients.Properties.RowNames);
    coefVals  = mdl.Coefficients.Estimate;

    fid = fopen(out_h, 'w');
    if fid < 0
        error('No pude abrir para escritura: %s', out_h);
    end
    cleanup = onCleanup(@() fclose(fid));

    fprintf(fid, "/* Auto-generated from MATLAB fitlm */\n");
    fprintf(fid, "/* Model: %s */\n", string(mdl.Formula));
    fprintf(fid, "/* Notes:\n");
    fprintf(fid, " * - Supports linear terms, interactions (A:B), and I(var.^2).\n");
    fprintf(fid, " * - If model uses Lux_log / Lux_log1p, you must provide them (precomputed).\n");
    fprintf(fid, " */\n\n");

    fprintf(fid, "#ifndef TFMINI_CALIB_COEFFS_H\n#define TFMINI_CALIB_COEFFS_H\n\n");
    fprintf(fid, "#include <stdint.h>\n\n");

    % ---- macros de coeficientes ----
    fprintf(fid, "/* Coefficients (float) */\n");
    for i = 1:numel(coefVals)
        nm = coef_to_macro(coefNames(i));
        fprintf(fid, "#define TFMC_%s (%.9gf)\n", nm, single(coefVals(i)));
    end
    fprintf(fid, "\n");

    % ---- inputs detectados por el modelo ----
    used = detect_used_inputs(coefNames);

    % Generar firma con TODOS, pero documentar cuáles usa.
    % (Así no te rompe si el best cambia.)
    fprintf(fid, "/* Model expects these inputs (some may be unused depending on model):\n");
    fprintf(fid, " *   Dist_cm, Freq_prom, Temp_C, Lux, Lux_log, Lux_log1p\n");
    fprintf(fid, " * Used by this model:\n");
    fprintf(fid, " *   %s\n", strjoin(used, ", "));
    fprintf(fid, " */\n");

    % ---- función C ----
    fprintf(fid, "/* Returns predicted corrected distance (cm) */\n");
    fprintf(fid, "static inline float %s(float Dist_cm, float Freq_prom, float Temp_C, float Lux, float Lux_log, float Lux_log1p)\n{\n", funcName);
    fprintf(fid, "    float y = 0.0f;\n");
    fprintf(fid, "    y += TFMC_INTERCEPT;\n");

    for i = 1:numel(coefNames)
        cn = coefNames(i);
        if cn == "(Intercept)", continue; end
        macro = coef_to_macro(cn);
        expr  = term_to_expr(cn);
        fprintf(fid, "    y += TFMC_%s * (%s);\n", macro, expr);
    end

    fprintf(fid, "    return y;\n}\n\n");
    fprintf(fid, "#endif /* TFMINI_CALIB_COEFFS_H */\n");
end


function used = detect_used_inputs(coefNames)
% Devuelve lista de inputs mencionados por los términos del modelo
    terms = coefNames(coefNames ~= "(Intercept)");
    usedSet = strings(0,1);

    for i = 1:numel(terms)
        s = char(terms(i));

        % sacar wrapper I(...)
        s = regexprep(s, '^I\((.*)\)$', '$1');

        % detectar variables conocidas
        vars = ["Dist_cm","Freq_prom","Temp_C","Lux","Lux_log","Lux_log1p"];
        for v = vars
            if contains(s, v)
                usedSet(end+1) = v; %#ok<AGROW>
            end
        end
    end

    usedSet = unique(usedSet, 'stable');
    if isempty(usedSet)
        usedSet = "Dist_cm";
    end
    used = cellstr(usedSet);
end


function nm = coef_to_macro(cn)
% Convierte nombre de coeficiente de MATLAB a macro C válida
    s = char(cn);

    if strcmp(s,'(Intercept)')
        s = 'INTERCEPT';
    end

    % Normalizar:
    % - Interacciones ":" -> _X_
    % - I(Dist_cm.^2) -> I_DIST_CM_POW2 (aprox)
    s = strrep(s,':','_X_');

    % I(var.^2) o I(var^2) u otras variantes -> I_<...>
    % Dejamos I(....) pero limpiamos símbolos
    s = upper(regexprep(s,'[^A-Z0-9_]', '_'));
    s = regexprep(s, '_+', '_');      % colapsar ___
    s = regexprep(s, '^_|_$', '');    % trim _

    nm = s;
end


function expr = term_to_expr(cn)
% Mapea términos de MATLAB a expresiones C
% Soporta:
%   - var
%   - var1:var2
%   - I(var.^2) / I(var^2)
    s = char(cn);

    % --- interacción A:B ---
    if contains(s, ':')
        parts = split(string(s), ":");
        if numel(parts) == 2
            a = strtrim(parts(1));
            b = strtrim(parts(2));
            ea = base_var_expr(char(a));
            eb = base_var_expr(char(b));
            expr = sprintf('(%s*%s)', ea, eb);
            return;
        end
    end

    % --- término I(...) ---
    if startsWith(s, 'I(') && endsWith(s, ')')
        inside = extractBetween(string(s), "I(", ")");
        inside = char(inside);

        % casos típicos: Dist_cm.^2, Lux.^2, Lux_log1p.^2 ...
        % Convertimos "X.^2" o "X^2" a (X*X)
        inside = strrep(inside, ".^", "^"); % unificar
        inside = strrep(inside, " ", "");

        % Match: <var>^2
        tok = regexp(inside, '^([A-Za-z0-9_]+)\^2$', 'tokens', 'once');
        if ~isempty(tok)
            v = tok{1};
            ev = base_var_expr(v);
            expr = sprintf('(%s*%s)', ev, ev);
            return;
        end

        % Si aparece algo más raro, lo anulamos explícitamente
        expr = '0.0f /* unsupported I(...) */';
        return;
    end

    % --- término simple ---
    expr = base_var_expr(s);
end


function expr = base_var_expr(v)
% Variables permitidas -> nombre en C
    v = char(string(v)); % asegurar char limpio

    switch v
        case 'Dist_cm'
            expr = 'Dist_cm';
        case 'Freq_prom'
            expr = 'Freq_prom';
        case 'Temp_C'
            expr = 'Temp_C';
        case 'Lux'
            expr = 'Lux';
        case 'Lux_log'
            expr = 'Lux_log';
        case 'Lux_log1p'
            expr = 'Lux_log1p';
        otherwise
            expr = '0.0f /* unsupported var */';
    end
end

