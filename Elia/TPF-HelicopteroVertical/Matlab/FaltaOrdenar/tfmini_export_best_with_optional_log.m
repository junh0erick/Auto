function tfmini_export_best_with_optional_log()
% TFMINI_EXPORT_BEST_WITH_OPTIONAL_LOG
% 1) Corre tfmini_plus_calibrate_models()
% 2) Toma el best fitlm
% 3) Si el modelo usa Lux_log o Lux_log1p -> genera LUT ln() y exporta header
% 4) Exporta header del modelo y plotea comparaciones

    % ========= 1) correr calibración =========
    S = tfmini_plus_calibrate_models();   % o pasale ruta/sheet
    T = S.T;
    mdl = S.best.model;

    if isempty(mdl)
        error("No hay modelo best válido en S.best.model");
    end

    % ========= 2) paths de salida =========
    out_dir = pwd;
    out_model_h = fullfile(out_dir, 'tfmini_calib_coeffs.h');
    out_lut_h   = fullfile(out_dir, 'tfmini_ln_lut.h');   % solo si hace falta

    % ========= 3) detectar uso de log =========
    coefNames = string(mdl.Coefficients.Properties.RowNames);
    usesLuxLog   = any(coefNames == "Lux_log" | contains(coefNames, "Lux_log:") | contains(coefNames, ":Lux_log") | contains(coefNames, "Lux_log.^2") | contains(coefNames,"Lux_log"));
    usesLuxLog1p = any(coefNames == "Lux_log1p" | contains(coefNames, "Lux_log1p:") | contains(coefNames, ":Lux_log1p") | contains(coefNames, "Lux_log1p.^2") | contains(coefNames,"Lux_log1p"));

    needLUT = usesLuxLog || usesLuxLog1p;

    % ========= 4) si hace falta, generar LUT =========
    if needLUT
        % 4096 bins -> STEP=16, Q8.8 (muy seguro en uint16)
        generate_tfmini_ln_lut_header(out_lut_h, 12, 8);
        fprintf("OK: LUT header generado: %s\n", out_lut_h);
    end

    % ========= 5) exportar modelo a header =========
    export_fitlm_to_c_header_best(mdl, out_model_h, 'tfmini_correct_distance_cm', needLUT);
    fprintf("OK: Modelo exportado en: %s\n", out_model_h);

    % ========= 6) predicción para plot =========
    Dist_pred = predict(mdl, T);
    Dist_real = T.Dist_real;
    Dist_cm   = T.Dist_cm;

    [Dist_real_s, idx] = sort(Dist_real);
    Dist_cm_s   = Dist_cm(idx);
    Dist_pred_s = Dist_pred(idx);

    step = max(1, floor(numel(idx)/2000));
    ii = 1:step:numel(idx);

    figure; hold on; grid on;
    plot(Dist_real_s(ii), Dist_cm_s(ii), '.', 'MarkerSize', 8);
    plot(Dist_real_s(ii), Dist_pred_s(ii), '.', 'MarkerSize', 8);
    mn = min(Dist_real_s(ii));
    mx = max(Dist_real_s(ii));
    plot([mn mx], [mn mx], 'k--', 'LineWidth', 1.5);

    xlabel('Dist\_real [cm]');
    ylabel('Distancia [cm]');
    title('Comparación: real vs sensor vs modelo');
    legend({'Sensor (Dist\_cm)', 'Modelo (pred)', 'Ideal y=x'}, 'Location', 'best');
end


% =====================================================================
%  LUT header generator: ln(x) con interpolación lineal
% =====================================================================
function generate_tfmini_ln_lut_header(out_h, lut_bits, qshift)
% Genera tfmini_ln_lut.h con LUT de ln(x) en Q(qshift) e interpolación.
%
% lut_bits=12 -> N=4096, STEP=16
% qshift=8    -> Q8.8

    if nargin < 2 || isempty(lut_bits), lut_bits = 12; end
    if nargin < 3 || isempty(qshift), qshift = 8; end

    N     = 2^lut_bits;              % 4096
    STEP  = 2^(16 - lut_bits);       % 16
    SCALE = 2^qshift;

    tab = zeros(N+1,1,'uint16');
    tab(1) = uint16(0); % ln(0) clamped

    for i = 1:N
        x = double(i*STEP);
        y = log(x); % ln()
        q = round(y * SCALE);
        if q < 0, q = 0; end
        if q > 65535, q = 65535; end
        tab(i+1) = uint16(q);
    end

    fid = fopen(out_h,'w');
    if fid < 0
        error("No pude abrir para escribir: %s", out_h);
    end
    c = onCleanup(@() fclose(fid));

    fprintf(fid, "/* Auto-generated LUT: ln(x) with linear interpolation */\n");
    fprintf(fid, "/* LUT_BITS=%d -> N=%d, STEP=%d, Q=%d (scale=%d) */\n\n", lut_bits, N, STEP, qshift, SCALE);

    fprintf(fid, "#ifndef TFMINI_LN_LUT_H\n#define TFMINI_LN_LUT_H\n\n");
    fprintf(fid, "#include <stdint.h>\n");
    fprintf(fid, '#include "arm_math.h"   /* float32_t */\n');

    fprintf(fid, "#define TFM_LUT_BITS   (%du)\n", lut_bits);
    fprintf(fid, "#define TFM_LUT_N      (1u << TFM_LUT_BITS)\n");
    fprintf(fid, "#define TFM_LUT_STEP   (1u << (16u - TFM_LUT_BITS))\n");
    fprintf(fid, "#define TFM_LN_Q_SHIFT (%du)\n", qshift);
    fprintf(fid, "#define TFM_LN_Q_SCALE (1u << TFM_LN_Q_SHIFT)\n\n");

    fprintf(fid, "/* ln(i*STEP) in Q format. Index 0 is clamped (ln(0)->0). */\n");
    fprintf(fid, "static const uint16_t tfm_ln_lut_q[] = {\n");

    % volcar tabla en filas de 8-12 valores para que quede legible
    perLine = 10;
    for i = 1:numel(tab)
        if mod(i-1, perLine) == 0
            fprintf(fid, "  ");
        end
        fprintf(fid, "0x%04Xu", tab(i));
        if i ~= numel(tab)
            fprintf(fid, ", ");
        end
        if mod(i, perLine) == 0 || i == numel(tab)
            fprintf(fid, "\n");
        end
    end
    fprintf(fid, "};\n\n");

    fprintf(fid, "/* ln(lux) approx in Q format (Q%d.%d stored as uint16) */\n", 16-qshift, qshift);
    fprintf(fid, "static inline uint16_t tfm_ln_lux_q(uint16_t lux)\n{\n");
    fprintf(fid, "    if (lux == 0u) return 0u;\n");
    fprintf(fid, "    uint16_t i = (uint16_t)(lux >> (16u - TFM_LUT_BITS));\n");
    fprintf(fid, "    uint16_t f = (uint16_t)(lux & (TFM_LUT_STEP - 1u));\n");
    fprintf(fid, "    if (i >= TFM_LUT_N) return tfm_ln_lut_q[TFM_LUT_N];\n");
    fprintf(fid, "    uint16_t y0 = tfm_ln_lut_q[i];\n");
    fprintf(fid, "    uint16_t y1 = tfm_ln_lut_q[i + 1u];\n");
    fprintf(fid, "    uint16_t dy = (uint16_t)(y1 - y0);\n");
    fprintf(fid, "    return (uint16_t)(y0 + (uint16_t)((dy * f) / TFM_LUT_STEP));\n");
    fprintf(fid, "}\n\n");

    fprintf(fid, "/* ln(1+lux) approx in Q format */\n");
    fprintf(fid, "static inline uint16_t tfm_ln1p_lux_q(uint16_t lux)\n{\n");
    fprintf(fid, "    uint32_t x = (uint32_t)lux + 1u;\n");
    fprintf(fid, "    if (x > 65535u) x = 65535u;\n");
    fprintf(fid, "    return tfm_ln_lux_q((uint16_t)x);\n");
    fprintf(fid, "}\n\n");

    fprintf(fid, "/* float32_t helpers */\n");
    fprintf(fid, "static inline float32_t tfm_ln_lux_f(uint16_t lux)\n{\n");
    fprintf(fid, "    return ((float32_t)tfm_ln_lux_q(lux)) / (float32_t)TFM_LN_Q_SCALE;\n");
    fprintf(fid, "}\n\n");
    fprintf(fid, "static inline float32_t tfm_ln1p_lux_f(uint16_t lux)\n{\n");
    fprintf(fid, "    return ((float32_t)tfm_ln1p_lux_q(lux)) / (float32_t)TFM_LN_Q_SCALE;\n");
    fprintf(fid, "}\n\n");

    fprintf(fid, "#endif /* TFMINI_LN_LUT_H */\n");
end


% =====================================================================
%  Exportador del modelo: si usa log -> incluye LUT y reemplaza Lux_log
% =====================================================================
function export_fitlm_to_c_header_best(mdl, out_h, funcName, needLUT)
    if nargin < 3 || isempty(funcName)
        funcName = 'tfmini_correct_distance_cm';
    end
    if nargin < 4, needLUT = false; end

    coefNames = string(mdl.Coefficients.Properties.RowNames);
    coefVals  = mdl.Coefficients.Estimate;

    fid = fopen(out_h, 'w');
    if fid < 0
        error('No pude abrir para escritura: %s', out_h);
    end
    cleanup = onCleanup(@() fclose(fid));

    fprintf(fid, "/* Auto-generated from MATLAB fitlm */\n");
    fprintf(fid, "/* Model: %s */\n\n", string(mdl.Formula));

    fprintf(fid, "#ifndef TFMINI_CALIB_COEFFS_H\n#define TFMINI_CALIB_COEFFS_H\n\n");
    fprintf(fid, "#include <stdint.h>\n");
    fprintf(fid, '#include \"arm_math.h\"   /* float32_t */\n');
    if needLUT
        fprintf(fid, '#include \"tfmini_ln_lut.h\"  /* tfm_ln_lux_f / tfm_ln1p_lux_f */\n');
    end
    fprintf(fid, "\n");

    % macros coef
    fprintf(fid, "/* Coefficients (float32_t) */\n");
    for i = 1:numel(coefVals)
        nm = coef_to_macro(coefNames(i));
        fprintf(fid, "#define TFMC_%s ((float32_t)%.9gf)\n", nm, single(coefVals(i)));
    end
    fprintf(fid, "\n");

    % Firma: si hay LUT, Lux es uint16 (porque LUT indexa 16b)
    % Si no hay LUT, Lux va como float32_t como antes
    if needLUT
        fprintf(fid, "/* Inputs:\n");
        fprintf(fid, " *   Dist_cm, Freq_prom, Temp_C as float32_t\n");
        fprintf(fid, " *   Lux as uint16_t (ADC / sensor raw) used to compute ln(Lux) via LUT\n");
        fprintf(fid, " */\n");
        fprintf(fid, "static inline float32_t %s(float32_t Dist_cm, float32_t Freq_prom, float32_t Temp_C, uint16_t Lux)\n{\n", funcName);
        fprintf(fid, "    float32_t y = 0.0f;\n");
        fprintf(fid, "    y += TFMC_INTERCEPT;\n");
    else
        fprintf(fid, "/* Inputs: all float32_t */\n");
        fprintf(fid, "static inline float32_t %s(float32_t Dist_cm, float32_t Freq_prom, float32_t Temp_C, float32_t Lux)\n{\n", funcName);
        fprintf(fid, "    float32_t y = 0.0f;\n");
        fprintf(fid, "    y += TFMC_INTERCEPT;\n");
    end

    for i = 1:numel(coefNames)
        cn = coefNames(i);
        if cn == "(Intercept)", continue; end
        macro = coef_to_macro(cn);
        expr  = term_to_expr_best(cn, needLUT);
        fprintf(fid, "    y += TFMC_%s * (%s);\n", macro, expr);
    end

    fprintf(fid, "    return y;\n}\n\n");
    fprintf(fid, "#endif /* TFMINI_CALIB_COEFFS_H */\n");
end


% ================= helpers de export =================
function nm = coef_to_macro(cn)
    s = char(cn);
    if strcmp(s,'(Intercept)'), s = 'INTERCEPT'; end
    s = strrep(s,':','_X_');
    s = upper(regexprep(s,'[^A-Z0-9_]', '_'));
    s = regexprep(s,'_+', '_');
    s = regexprep(s,'^_|_$', '');
    nm = s;
end


function expr = term_to_expr_best(cn, needLUT)
    s = char(cn);

    % Interacción A:B
    if contains(s, ':')
        parts = split(string(s), ":");
        if numel(parts)==2
            ea = base_var_expr_best(char(strtrim(parts(1))), needLUT);
            eb = base_var_expr_best(char(strtrim(parts(2))), needLUT);
            expr = sprintf('(%s*%s)', ea, eb);
            return;
        end
    end

    % I(...) cuadráticos
    if startsWith(s, 'I(') && endsWith(s, ')')
        inside = extractBetween(string(s),"I(",")");
        inside = char(inside);
        inside = strrep(inside,".^","^");
        inside = strrep(inside," ","");
        tok = regexp(inside,'^([A-Za-z0-9_]+)\^2$','tokens','once');
        if ~isempty(tok)
            ev = base_var_expr_best(tok{1}, needLUT);
            expr = sprintf('(%s*%s)', ev, ev);
            return;
        end
        expr = '0.0f /* unsupported I(...) */';
        return;
    end

    % Simple
    expr = base_var_expr_best(s, needLUT);
end

function expr = base_var_expr_best(v, needLUT)
    v = char(string(v));

    switch v
        case 'Dist_cm'
            expr = 'Dist_cm';

        case 'Freq_prom'
            expr = 'Freq_prom';

        case 'Temp_C'
            expr = 'Temp_C';

        case 'Lux'
            expr = 'Lux'; % float si no hay LUT, uint16 si hay LUT (se promueve cuando haga falta)

        % ====== NUEVOS: columnas cuadráticas explícitas ======
        case 'Dist_cm2'
            expr = '(Dist_cm*Dist_cm)';

        case 'Freq_prom2'
            expr = '(Freq_prom*Freq_prom)';

        case 'Temp_C2'
            expr = '(Temp_C*Temp_C)';

        case 'Lux2'
            if needLUT
                expr = '((float32_t)Lux*(float32_t)Lux)';
            else
                expr = '(Lux*Lux)';
            end

        case 'Lux_log2'
            if ~needLUT
                expr = '0.0f /* Lux_log2 requires LUT */';
            else
                expr = '(tfm_ln_lux_f(Lux)*tfm_ln_lux_f(Lux))';
            end

        case 'Lux_log1p2'
            if ~needLUT
                expr = '0.0f /* Lux_log1p2 requires LUT */';
            else
                expr = '(tfm_ln1p_lux_f(Lux)*tfm_ln1p_lux_f(Lux))';
            end

        % ====== log base ======
        case 'Lux_log'
            if ~needLUT
                expr = '0.0f /* Lux_log requires LUT */';
            else
                expr = 'tfm_ln_lux_f(Lux)';
            end

        case 'Lux_log1p'
            if ~needLUT
                expr = '0.0f /* Lux_log1p requires LUT */';
            else
                expr = 'tfm_ln1p_lux_f(Lux)';
            end

        otherwise
            expr = '0.0f /* unsupported var */';
    end
end
