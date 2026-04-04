function calib_export_simple_h(hFile, out, opts)
% Exporta header .h desde un modelo fitlm (out.mdl) o desde cfit (out.fit).
%
% Genera macros float32_t y una función inline.
% Soporta el modelo:
% y = A + B*d + C*d2 + K_LNQ*lnq + K_T*T + K_F*F + K_DLNQ*(d*lnq) + K_DT*(d*T) + K_DF*(d*F)
%
% opts:
%   opts.guard  = "TFM_CALIB_SIMPLE_H_"
%   opts.prefix = "TFM"
%   opts.fn     = "tfmini_correct_distance_cm_simple"

if nargin < 3, opts = struct(); end
if ~isfield(opts,'guard'),  opts.guard  = "TFM_CALIB_SIMPLE_H_"; end
if ~isfield(opts,'prefix'), opts.prefix = "TFM"; end
if ~isfield(opts,'fn'),     opts.fn     = "tfmini_correct_distance_cm_simple"; end

guard  = char(opts.guard);
prefix = char(opts.prefix);
fnname = char(opts.fn);

% ========= 1) Obtener coeficientes =========
% default 0
A=0; B=0; C=0; K_LNQ=0; K_T=0; K_F=0; K_DLNQ=0; K_DT=0; K_DF=0;

if isfield(out,'mdl') && ~isempty(out.mdl)
    % ---- fitlm / LinearModel ----
    coefTbl = out.mdl.Coefficients;
    names = string(coefTbl.Properties.RowNames);
    vals  = coefTbl.Estimate;

    get = @(name) local_get_coef(names, vals, name);

    A      = get("(Intercept)");
    B      = get("d");
    C      = get("d2");
    K_LNQ  = get("lnq");
    K_T    = get("T");
    K_F    = get("F");
    K_DLNQ = get("d:lnq");
    K_DT   = get("d:T");
    K_DF   = get("d:F");

elseif isfield(out,'fit') && ~isempty(out.fit)
    % ---- cfit (caso viejo) ----
    % y = a + b*x + c*x^2
    if isprop(out.fit,'a'), A = out.fit.a; end
    if isprop(out.fit,'b'), B = out.fit.b; end
    if isprop(out.fit,'c'), C = out.fit.c; end
else
    error("out no tiene out.mdl (fitlm) ni out.fit (cfit).");
end

% ========= 2) Escribir .h =========
fid = fopen(hFile,'w');
if fid < 0, error("No pude abrir: %s", hFile); end

fprintf(fid, "/* Auto-generated calibration header (MATLAB) */\n");
fprintf(fid, "/* Model: y = A + B*d + C*d2 + K_LNQ*lnq + K_T*T + K_F*F + K_DLNQ*(d*lnq) + K_DT*(d*T) + K_DF*(d*F) */\n\n");

fprintf(fid, "#ifndef %s\n#define %s\n\n", guard, guard);
fprintf(fid, "#include <stdint.h>\n");
fprintf(fid, '#include \"arm_math.h\" /* float32_t */\n');
fprintf(fid, "#include <math.h>     /* logf (si usás la función con light_raw) */\n\n");

% macros
fprintf(fid, "#define %s_CAL_A      ((float32_t)%s)\n", prefix, local_cfloat(A));
fprintf(fid, "#define %s_CAL_B      ((float32_t)%s)\n", prefix, local_cfloat(B));
fprintf(fid, "#define %s_CAL_C      ((float32_t)%s)\n", prefix, local_cfloat(C));
fprintf(fid, "#define %s_CAL_K_LNQ  ((float32_t)%s)\n", prefix, local_cfloat(K_LNQ));
fprintf(fid, "#define %s_CAL_K_T    ((float32_t)%s)\n", prefix, local_cfloat(K_T));
fprintf(fid, "#define %s_CAL_K_F    ((float32_t)%s)\n", prefix, local_cfloat(K_F));
fprintf(fid, "#define %s_CAL_K_DLNQ ((float32_t)%s)\n", prefix, local_cfloat(K_DLNQ));
fprintf(fid, "#define %s_CAL_K_DT   ((float32_t)%s)\n", prefix, local_cfloat(K_DT));
fprintf(fid, "#define %s_CAL_K_DF   ((float32_t)%s)\n\n", prefix, local_cfloat(K_DF));

% -------- Variante A: SIN logf (vos pasás lnq) --------
fprintf(fid, "/*\n");
fprintf(fid, " * Variante SIN logf: pasás lnq = log(1+light_raw) ya calculado.\n");
fprintf(fid, " */\n");
fprintf(fid, "static inline float32_t %s_lnq(float32_t dis_sensor_cm,\n", fnname);
fprintf(fid, "                                      float32_t lnq,\n");
fprintf(fid, "                                      float32_t temp_c,\n");
fprintf(fid, "                                      float32_t freq_prom)\n{\n");
fprintf(fid, "    float32_t d  = dis_sensor_cm;\n");
fprintf(fid, "    float32_t d2 = d*d;\n");
fprintf(fid, "    float32_t y = %s_CAL_A\n", prefix);
fprintf(fid, "                + %s_CAL_B*d\n", prefix);
fprintf(fid, "                + %s_CAL_C*d2\n", prefix);
fprintf(fid, "                + %s_CAL_K_LNQ*lnq\n", prefix);
fprintf(fid, "                + %s_CAL_K_T*temp_c\n", prefix);
fprintf(fid, "                + %s_CAL_K_F*freq_prom\n", prefix);
fprintf(fid, "                + %s_CAL_K_DLNQ*(d*lnq)\n", prefix);
fprintf(fid, "                + %s_CAL_K_DT*(d*temp_c)\n", prefix);
fprintf(fid, "                + %s_CAL_K_DF*(d*freq_prom);\n", prefix);
fprintf(fid, "    return y;\n");
fprintf(fid, "}\n\n");

% -------- Variante B: CON logf (si querés usar light_raw directo) --------
fprintf(fid, "/*\n");
fprintf(fid, " * Variante con logf: calcula lnq = log(1+light_raw).\n");
fprintf(fid, " * OJO: requiere linkear libm (si tu toolchain no lo hace, usá la _lnq).\n");
fprintf(fid, " */\n");
fprintf(fid, "static inline float32_t %s(float32_t dis_sensor_cm,\n", fnname);
fprintf(fid, "                                 float32_t light_raw,\n");
fprintf(fid, "                                 float32_t temp_c,\n");
fprintf(fid, "                                 float32_t freq_prom)\n{\n");
fprintf(fid, "    if (light_raw < 0.0f) light_raw = 0.0f;\n");
fprintf(fid, "    float32_t lnq = logf(1.0f + light_raw);\n");
fprintf(fid, "    return %s_lnq(dis_sensor_cm, lnq, temp_c, freq_prom);\n", fnname);
fprintf(fid, "}\n\n");

fprintf(fid, "#endif /* %s */\n", guard);
fclose(fid);

fprintf("OK: generado %s\n", hFile);

end

% ===== helpers =====
function v = local_get_coef(names, vals, key)
idx = find(names == string(key), 1);
if isempty(idx)
    v = 0;
else
    v = vals(idx);
    if ~isfinite(v), v = 0; end
end
end

function s = local_cfloat(x)
% Convierte número MATLAB -> literal C float válido (siempre con .0f o e...f)
if ~isfinite(x) || isnan(x)
    s = "0.0f";
    return;
end

% imprimir con buena precisión sin "0f"
str = sprintf('%.12g', x);

% asegurar que tenga punto decimal si es entero y no usa exponente
hasE = contains(str,'e') || contains(str,'E');
hasDot = contains(str,'.');

if ~hasE && ~hasDot
    str = [str '.0'];
end

s = string([str 'f']);
end
