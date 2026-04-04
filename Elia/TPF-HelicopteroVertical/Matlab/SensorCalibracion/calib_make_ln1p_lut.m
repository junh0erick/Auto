function calib_make_ln1p_lut(hFile, Q_MAX, N, guard, array_name)
% Genera LUT para ln(1+q) evitando logf en C.
% q se satura a [0..Q_MAX]
%
% hFile: "tfm_ln1p_lut.h"
% Q_MAX: ej 20000 (si tu luz llega ~19081)
% N    : ej 2048 o 4096
%
% Salida:
%  - defines Q_MAX, N
%  - array float32_t lut[N] con ln(1+q_i)
%
if nargin < 2, Q_MAX = 20000; end
if nargin < 3, N = 2048; end
if nargin < 4, guard = "TFM_LN1P_LUT_H_"; end
if nargin < 5, array_name = "tfm_ln1p_lut"; end

q = linspace(0, Q_MAX, N);
lut = log1p(q); % NATURAL ln(1+q)

fid = fopen(hFile,'w');
if fid < 0, error("No pude abrir: %s", hFile); end

fprintf(fid, "/* Auto-generated LUT: ln(1+q), q in [0..%d] */\n", Q_MAX);
fprintf(fid, "#ifndef %s\n#define %s\n\n", guard, guard);
fprintf(fid, '#include \"arm_math.h\" /* float32_t */\n#include <stdint.h>\n\n');
fprintf(fid, "#define TFM_LN1P_Q_MAX   (%du)\n", Q_MAX);
fprintf(fid, "#define TFM_LN1P_LUT_N   (%du)\n\n", N);

fprintf(fid, "static const float32_t %s[TFM_LN1P_LUT_N] = {\n", array_name);

for i = 1:N
    if mod(i-1, 6) == 0
        fprintf(fid, "    ");
    end
    fprintf(fid, "%.9gf", lut(i));
    if i ~= N, fprintf(fid, ", "); end
    if mod(i, 6) == 0 || i == N
        fprintf(fid, "\n");
    end
end

fprintf(fid, "};\n\n");
fprintf(fid, "#endif /* %s */\n", guard);
fclose(fid);

fprintf("OK: generado %s (Q_MAX=%d, N=%d)\n", hFile, Q_MAX, N);
end
