function calib_export_header(hFile, out, opts)
% Exporta: beta + LUT ln(1+Lux) + tfm_ln1p_lux_f() + tfmini_correct_distance_cm()
%
% Requiere en 'out':
%   out.beta (7x1)
%   out.lut (Nx1)  single/double
%   out.lut_max (uint32 o double)
%   out.lut_shift (uint32 o double)
%
% 'opts' campos:
%   opts.guard, opts.prefix, opts.array_name_beta, opts.array_name_lut

if nargin < 3, opts = struct(); end
if ~isfield(opts,'guard'),          opts.guard = 'TFM_CALIB_LN_LUT_H_'; end
if ~isfield(opts,'prefix'),         opts.prefix = 'TFM'; end
if ~isfield(opts,'array_name_beta'),opts.array_name_beta = 'tfm_calib_beta'; end
if ~isfield(opts,'array_name_lut'), opts.array_name_lut  = 'tfm_ln1p_lux_lut'; end

assert(isfield(out,'beta') && ~isempty(out.beta), 'out.beta no existe o está vacío');
assert(isfield(out,'lut')  && ~isempty(out.lut),  'out.lut no existe o está vacío');
assert(isfield(out,'lut_max') && ~isempty(out.lut_max), 'out.lut_max no existe');
assert(isfield(out,'lut_shift') && ~isempty(out.lut_shift), 'out.lut_shift no existe');

beta = out.beta(:);
lut  = out.lut(:);

lut_max   = uint32(out.lut_max);
lut_shift = uint32(out.lut_shift);
lut_size  = uint32(numel(lut));

f = fopen(hFile, 'w');
if f < 0, error('No pude abrir %s para escritura', hFile); end
cleanup = onCleanup(@() fclose(f));

fprintf(f, "/* Auto-generated calibration header */\n");
fprintf(f, "#ifndef %s\n#define %s\n\n", opts.guard, opts.guard);
fprintf(f, '#include <stdint.h>\n#include \"arm_math.h\"   /* float32_t */\n\n');

% --- beta ---
fprintf(f, "#define %s_CALIB_NBETA (%d)\n\n", opts.prefix, numel(beta));
fprintf(f, "static const float32_t %s[%d] = {\n", opts.array_name_beta, numel(beta));
for i = 1:numel(beta)
    s = float_to_c(beta(i));
    if i < numel(beta), s = [s ',']; end %#ok<AGROW>
    fprintf(f, "    %s\n", s);
end
fprintf(f, "};\n\n");

% --- LUT ---
fprintf(f, "#define %s_LN_LUX_MAX      (%uu)\n", opts.prefix, lut_max);
fprintf(f, "#define %s_LN_LUT_SHIFT    (%uu)\n", opts.prefix, lut_shift);
fprintf(f, "#define %s_LN_LUT_STEP     (1u << %s_LN_LUT_SHIFT)\n", opts.prefix, opts.prefix);
fprintf(f, "#define %s_LN_LUT_SIZE     (%uu)\n\n", opts.prefix, lut_size);

fprintf(f, "static const float32_t %s[%u] = {\n", opts.array_name_lut, lut_size);
for i = 1:numel(lut)
    s = float_to_c(lut(i));
    if i < numel(lut), s = [s ',']; end %#ok<AGROW>
    fprintf(f, "    %s\n", s);
end
fprintf(f, "};\n\n");

% --- ln1p(Lux) ---
pfxlow = lower(opts.prefix);
fprintf(f, "static inline float32_t %s_ln1p_lux_f(uint16_t Lux)\n{\n", pfxlow);
fprintf(f, "    uint32_t x = (uint32_t)Lux;\n");
fprintf(f, "    if (x > %s_LN_LUX_MAX) x = %s_LN_LUX_MAX;\n", opts.prefix, opts.prefix);
fprintf(f, "    uint32_t idx = (x >> %s_LN_LUT_SHIFT);\n", opts.prefix);
fprintf(f, "    if (idx >= (%s_LN_LUT_SIZE - 1u)) {\n", opts.prefix);
fprintf(f, "        return %s[%s_LN_LUT_SIZE - 1u];\n", opts.array_name_lut, opts.prefix);
fprintf(f, "    }\n");
fprintf(f, "    uint32_t x0 = (idx << %s_LN_LUT_SHIFT);\n", opts.prefix);
fprintf(f, "    float32_t y0 = %s[idx];\n", opts.array_name_lut);
fprintf(f, "    float32_t y1 = %s[idx + 1u];\n", opts.array_name_lut);
fprintf(f, "    float32_t t  = (float32_t)(x - x0) * (1.0f / (float32_t)%s_LN_LUT_STEP);\n", opts.prefix);
fprintf(f, "    return y0 + (y1 - y0) * t;\n");
fprintf(f, "}\n\n");

% --- clamp ---
fprintf(f, "static inline float32_t %s_clampf(float32_t v, float32_t lo, float32_t hi)\n{\n", pfxlow);
fprintf(f, "    if (v < lo) return lo;\n");
fprintf(f, "    if (v > hi) return hi;\n");
fprintf(f, "    return v;\n");
fprintf(f, "}\n\n");

% --- función final (modelo 7 términos) + saturación física 12..134 ---
fprintf(f, "static inline float32_t tfmini_correct_distance_cm(float32_t Dist_cm, float32_t Freq_prom, float32_t Temp_C, uint16_t Lux)\n{\n");
fprintf(f, "    const float32_t lnq = %s_ln1p_lux_f(Lux);\n", pfxlow);
fprintf(f, "    const float32_t b0 = %s[0];\n", opts.array_name_beta);
fprintf(f, "    const float32_t b1 = %s[1];\n", opts.array_name_beta);
fprintf(f, "    const float32_t b2 = %s[2];\n", opts.array_name_beta);
fprintf(f, "    const float32_t b3 = %s[3];\n", opts.array_name_beta);
fprintf(f, "    const float32_t b4 = %s[4];\n", opts.array_name_beta);
fprintf(f, "    const float32_t b5 = %s[5];\n", opts.array_name_beta);
fprintf(f, "    const float32_t b6 = %s[6];\n", opts.array_name_beta);

fprintf(f, "    float32_t y = b0;\n");
fprintf(f, "    y += b1 * Freq_prom;\n");
fprintf(f, "    y += b2 * Dist_cm;\n");
fprintf(f, "    y += b3 * Temp_C;\n");
fprintf(f, "    y += b4 * lnq;\n");
fprintf(f, "    y += b5 * (Dist_cm * Temp_C);\n");
fprintf(f, "    y += b6 * (Dist_cm * lnq);\n");
fprintf(f, "    return %s_clampf(y, 12.0f, 134.0f);\n", pfxlow);
fprintf(f, "}\n\n");

fprintf(f, "#endif /* %s */\n", opts.guard);

fprintf("OK: exportado %s\n", hFile);

end

% ===== helpers =====
function s = float_to_c(x)
% Siempre termina en 'f' y evita 'ff'
x = double(x);
if ~isfinite(x), x = 0; end
if x == 0
    s = '0.0f';
    return;
end
s = sprintf('%.9g', x);
% asegura punto decimal cuando hace falta (opcional)
% if isempty(strfind(s,'.')) && isempty(strfind(lower(s),'e'))
%     s = [s '.0'];
% end
s = [s 'f'];
end
