% tfmini_plus_calibrate.m
% ------------------------------------------------------------
% OBJETIVO
%   Leer un Excel/CSV con columnas:
%     - Freq_prom (programada)    [Hz]
%     - Freq_medida              [Hz]
%     - dis_calc                 [cm]  (distancia del sensor)
%     - dist_medida              [cm]  (ground-truth medido por vos)
%     - Temperatura              [°C]
%     - Intensidad de Luz        [uint16 strength, SIN log]
%
%   Ajustar un modelo embebible para corregir:
%     d_corr = dis_calc + Delta(...)
%
%   Considera mediciones repetidas: agrupa escenarios y usa pesos (1/sigma^2)
%   y además hace validación por grupos (GroupKFold) para evitar overfit.
%
%   Exporta:
%     - Coeficientes del mejor modelo
%     - Header C con coeficientes + LUT log10(uint16) para PSoC
%
% NOTAS IMPORTANTES (realistas para PSoC)
%   - log10() es caro; se exporta LUT de 65536 entradas (float) por defecto.
%     Si querés LUT más chica, más abajo tenés opción "cfg.LUT.compact".
%   - Todo asumido 16-bit en datos: se procesa como double en MATLAB, pero
%     el modelo se entrena para ese rango.
%
% Requiere: Statistics and Machine Learning Toolbox (fitlm)
% ------------------------------------------------------------

clear; clc;

%% =================== CONFIG ===================
cfg.useFreqMeasured = true;          % true: usa Freq_medida; false: usa Freq_prom

% Agrupación de "mismo escenario" (repetibilidad):
% Ajustá estos bins según tu resolución real.
cfg.bin.d_meas_cm   = 1.0;           % 1 cm
cfg.bin.temp_c      = 0.5;           % 0.5 C
cfg.bin.freq_hz     = 1.0;           % 1 Hz
cfg.bin.strength    = 50;            % 50 counts (raw)

% Sanitización
cfg.minFreqHz       = 0.1;           % para log10(freq) si lo usás
cfg.minStrength     = 1;             % para log10(strength)
cfg.maxStrength     = 65535;

% Cross-validation
cfg.Kfold           = 5;             % GroupKFold sobre escenarios

% Modelos candidatos (todos embebibles, bajo orden)
% Variables:
%   d = dis_calc
%   T = temp
%   S = strength (raw)
%   x = log10(strength)
%   y = log10(freq)
%
% Delta = dist_medida - dis_calc
%
% OJO: fuera de rango real del sensor, los términos de interacción suelen ser clave.
models = {
    struct('name','M1_lin_logS_logF',  'formula','Delta ~ 1 + d + T + x + y')
    struct('name','M2_lin_logS_logF_dx','formula','Delta ~ 1 + d + T + x + y + d:x')
    struct('name','M3_lin_logS_logF_xy','formula','Delta ~ 1 + d + T + x + y + x:y')
    struct('name','M4_lin_logS_logF_full','formula','Delta ~ 1 + d + T + x + y + d:x + x:y')
    % Variante usando strength RAW en vez de log (por si log no ayuda)
    struct('name','M5_lin_rawS_logF',  'formula','Delta ~ 1 + d + T + S + y')
    struct('name','M6_lin_rawS_logF_dS','formula','Delta ~ 1 + d + T + S + y + d:S')
};

% Export C / LUT
cfg.export.header   = 'tfmini_calib_coeffs.h';
cfg.export.funcName = 'tfmini_correct_distance_cm';

% LUT log10(uint16):
cfg.LUT.enable      = true;
cfg.LUT.compact     = false;   % false: 65536 entradas; true: LUT compacta (por tramos)
cfg.LUT.compactN    = 1024;    % si compact=true, tamaño de LUT (interpolación lineal)

%% =================== SELECT FILE ===================
[file, path] = uigetfile({'*.xlsx;*.xls;*.csv','Data files (*.xlsx,*.xls,*.csv)'}, ...
                         'Seleccioná tu Excel/CSV de mediciones');
if isequal(file,0), error('No seleccionaste archivo.'); end
fname = fullfile(path,file);

%% =================== READ TABLE ===================
[~,~,ext] = fileparts(fname);
switch lower(ext)
    case {'.xlsx','.xls'}
        Traw = readtable(fname, 'PreserveVariableNames', true);
    case '.csv'
        Traw = readtable(fname, 'PreserveVariableNames', true);
    otherwise
        error('Extensión no soportada: %s', ext);
end

vars = string(Traw.Properties.VariableNames);

% Helper: encontrar columnas por patrones (tolerante a nombres)
findcol = @(patterns) find(contains(lower(vars), lower(patterns), 'IgnoreCase', true), 1, 'first');

iFreqProm = findcol(["freq_prom","freq_prog","freq_programada","freq_programada"]);
iFreqMeas = findcol(["freq_medida","freq_meas","freq_measure","freq_med"]);
iDisCalc  = findcol(["dis_calc","dist_calc","d_calc","dis_calculada"]);
iDisMeas  = findcol(["dist_medida","dist_real","d_real","ground","medida"]);
iTemp     = findcol(["temperatura","temp"]);
iStrength = findcol(["intensidad","strength","intensity","luz"]);

need = [iFreqProm,iFreqMeas,iDisCalc,iDisMeas,iTemp,iStrength];
if any(isnan(need)) || any(need==0)
    disp("Columnas detectadas:");
    disp(vars');
    error(['No pude mapear todas las columnas necesarias. ' ...
           'Asegurate que existan: Freq_prom, Freq_medida, dis_calc, dist_medida, Temperatura, Intensidad/Strength']);
end

Freq_prom = double(Traw{:,iFreqProm});
Freq_meas = double(Traw{:,iFreqMeas});
d_calc    = double(Traw{:,iDisCalc});
d_meas    = double(Traw{:,iDisMeas});
Temp      = double(Traw{:,iTemp});
Strength  = double(Traw{:,iStrength});

if cfg.useFreqMeasured
    Freq = Freq_meas;
else
    Freq = Freq_prom;
end

%% =================== CLEAN ===================
ok = isfinite(Freq) & isfinite(d_calc) & isfinite(d_meas) & isfinite(Temp) & isfinite(Strength);
Freq=Freq(ok); d_calc=d_calc(ok); d_meas=d_meas(ok); Temp=Temp(ok); Strength=Strength(ok);

% Clamp a rango 16-bit (vos dijiste que todo es 16-bit)
Strength = min(max(Strength,0), cfg.maxStrength);
Freq     = max(Freq, cfg.minFreqHz);

% Variables derivadas
S = Strength;                               % raw
x = log10(max(Strength, cfg.minStrength));  % log10(strength)
y = log10(max(Freq, cfg.minFreqHz));        % log10(freq)
d = d_calc;
T = Temp;

Delta = d_meas - d_calc;

%% =================== GROUP "SCENARIOS" for repeats ===================
% Para pesar por repetibilidad y para GroupKFold

q = @(v, step) round(v./step).*step;

key_dmeas = q(d_meas, cfg.bin.d_meas_cm);
key_temp  = q(T,      cfg.bin.temp_c);
key_freq  = q(Freq,   cfg.bin.freq_hz);
key_S     = q(S,      cfg.bin.strength);

G = findgroups(key_dmeas, key_temp, key_freq, key_S);

% Repetibilidad por grupo (sigma de Delta)
nG  = splitapply(@numel, Delta, G);
sdG = splitapply(@std,   Delta, G);
sdG(~isfinite(sdG) | sdG==0) = NaN;
sd_floor = nanmedian(sdG);
if ~isfinite(sd_floor) || sd_floor<=0
    sd_floor = 1.0;  % cm (fallback si no hay repetidos)
end

% Peso por muestra = 1/(sigma_grupo^2), con sigma de grupo y piso
sigma_i = sdG(G);
sigma_i(~isfinite(sigma_i)) = sd_floor;
sigma_i = max(sigma_i, sd_floor);
w = 1 ./ (sigma_i.^2);

%% =================== BUILD DATA TABLE for fitlm ===================
Dtbl = table(Delta, d, T, S, x, y, G, w, ...
    'VariableNames', {'Delta','d','T','S','x','y','G','w'});

%% =================== GROUP K-FOLD EVAL ===================
groupIDs = unique(G);
nGroups  = numel(groupIDs);

K = min(cfg.Kfold, nGroups);
cv = cvpartition(nGroups, 'KFold', K);

results = struct([]);
for m = 1:numel(models)
    mdlSpec = models{m};
    rmseFold = zeros(K,1);
    maeFold  = zeros(K,1);

    for k = 1:K
        testGroupIdx = test(cv,k);            % logical over groups
        testGroups   = groupIDs(testGroupIdx);

        isTest = ismember(G, testGroups);
        isTrain= ~isTest;

        trainTbl = Dtbl(isTrain,:);
        testTbl  = Dtbl(isTest,:);

        lm = fitlm(trainTbl, mdlSpec.formula, 'Weights', trainTbl.w);

        predDelta = predict(lm, testTbl);
        predCorr  = testTbl.d + predDelta;
        trueCorr  = testTbl.d + testTbl.Delta;   % = dist_medida

        e = predCorr - trueCorr;
        rmseFold(k) = sqrt(mean(e.^2));
        maeFold(k)  = mean(abs(e));
    end

    results(m).name     = mdlSpec.name;
    results(m).formula  = mdlSpec.formula;
    results(m).rmseMean = mean(rmseFold);
    results(m).rmseStd  = std(rmseFold);
    results(m).maeMean  = mean(maeFold);
    results(m).maeStd   = std(maeFold);
end

% Selección por RMSE medio
[~,bestIdx] = min([results.rmseMean]);
best = results(bestIdx);

fprintf('\n=== RESULTADOS CV (GroupKFold) ===\n');
for m = 1:numel(results)
    fprintf('%-22s  RMSE=%.3f±%.3f  MAE=%.3f±%.3f\n', ...
        results(m).name, results(m).rmseMean, results(m).rmseStd, results(m).maeMean, results(m).maeStd);
end
fprintf('\n>>> MEJOR: %s  |  %s\n', best.name, best.formula);

%% =================== FIT FINAL MODEL on ALL DATA ===================
lmBest = fitlm(Dtbl, best.formula, 'Weights', Dtbl.w);
disp(lmBest);

% Coeficientes
coef = lmBest.Coefficients;
coefNames = string(coef.Properties.RowNames);
coefVals  = coef.Estimate;

%% =================== QUICK SANITY PLOTS ===================
predDeltaAll = predict(lmBest, Dtbl);
d_corr = d + predDeltaAll;

figure; histogram(d_corr - d_meas, 60);
grid on; title('Error residual (d\_corr - dist\_medida) [cm]');

figure; scatter(d_meas, (d_corr - d_meas), 8, S, 'filled');
grid on; xlabel('dist\_medida [cm]'); ylabel('error residual [cm]');
title('Residual vs distancia (color = strength)'); colorbar;

%% =================== EXPORT C HEADER ===================
export_tfmini_header(cfg, best, lmBest, coefNames, coefVals);

fprintf('\nOK. Header exportado: %s\n', fullfile(pwd, cfg.export.header));

%% =================== LOCAL FUNCTIONS ===================
function export_tfmini_header(cfg, best, lm, coefNames, coefVals)
    hdr = cfg.export.header;
    fid = fopen(hdr,'w');
    if fid<0, error('No pude abrir %s para escribir.', hdr); end

    fprintf(fid, '/* Auto-generated by tfmini_plus_calibrate.m */\n');
    fprintf(fid, '/* Best model: %s */\n', best.name);
    fprintf(fid, '/* Formula: %s */\n', best.formula);
    fprintf(fid, '#ifndef TFMINI_CALIB_COEFFS_H\n#define TFMINI_CALIB_COEFFS_H\n\n');
    fprintf(fid, '#include <stdint.h>\n\n');

    % Emitir coeficientes como macros
    fprintf(fid, '/* Coefficients (double -> float for PSoC) */\n');
    for i=1:numel(coefVals)
        name = matlabCoefToC(coefNames(i));
        fprintf(fid, '#define TFMC_%s (%.9gf)\n', name, single(coefVals(i)));
    end
    fprintf(fid, '\n');

    % LUT log10(strength)
    if cfg.LUT.enable
        if ~cfg.LUT.compact
            fprintf(fid, '/* LUT log10(strength) for uint16 [0..65535].\n');
            fprintf(fid, '   Note: log10(0) is clamped to log10(1)=0.\n');
            fprintf(fid, '*/\n');
            fprintf(fid, 'static const float TFMC_LOG10_LUT[65536] = {\n');
            for s=0:65535
                v = log10(max(double(s),1.0));
                if mod(s,8)==0, fprintf(fid,'    '); end
                fprintf(fid,'%.6gf', single(v));
                if s~=65535, fprintf(fid,', '); end
                if mod(s,8)==7, fprintf(fid,'\n'); end
            end
            fprintf(fid, '\n};\n\n');

            fprintf(fid, 'static inline float tfmc_log10_strength(uint16_t strength)\n{\n');
            fprintf(fid, '    return TFMC_LOG10_LUT[strength];\n}\n\n');
        else
            N = cfg.LUT.compactN;
            fprintf(fid, '/* Compact LUT log10(strength) with linear interpolation.\n');
            fprintf(fid, '   Size: %d. strength in [0..65535], clamp 0->1.\n', N);
            fprintf(fid, '*/\n');
            fprintf(fid, 'static const float TFMC_LOG10_LUT[%d] = {\n', N);
            % Map index i -> strength value
            for i=0:N-1
                s = round(i*(65535/(N-1)));
                v = log10(max(double(s),1.0));
                if mod(i,8)==0, fprintf(fid,'    '); end
                fprintf(fid,'%.6gf', single(v));
                if i~=N-1, fprintf(fid,', '); end
                if mod(i,8)==7, fprintf(fid,'\n'); end
            end
            fprintf(fid, '\n};\n\n');

            fprintf(fid, 'static inline float tfmc_log10_strength(uint16_t strength)\n{\n');
            fprintf(fid, '    if(strength==0u) strength = 1u;\n');
            fprintf(fid, '    const float pos = ( (float)strength ) * ( (float)(%d - 1) / 65535.0f );\n', N);
            fprintf(fid, '    int32_t i = (int32_t)pos;\n');
            fprintf(fid, '    if(i < 0) i = 0;\n');
            fprintf(fid, '    if(i >= %d-1) return TFMC_LOG10_LUT[%d-1];\n', N, N);
            fprintf(fid, '    const float t = pos - (float)i;\n');
            fprintf(fid, '    return TFMC_LOG10_LUT[i] + t * (TFMC_LOG10_LUT[i+1] - TFMC_LOG10_LUT[i]);\n');
            fprintf(fid, '}\n\n');
        end
    else
        fprintf(fid, '/* log10 disabled: provide your own implementation */\n');
        fprintf(fid, 'static inline float tfmc_log10_strength(uint16_t strength)\n{\n');
        fprintf(fid, '    (void)strength; return 0.0f;\n}\n\n');
    end

    % Función de corrección en C (aplica el modelo best.formula)
    fprintf(fid, '/* Apply correction: returns corrected distance (cm) */\n');
    fprintf(fid, 'static inline float %s(float dis_calc_cm, uint16_t strength, float temp_c, float freq_hz)\n{\n', cfg.export.funcName);
    fprintf(fid, '    const float d = dis_calc_cm;\n');
    fprintf(fid, '    const float T = temp_c;\n');
    fprintf(fid, '    const float S_raw = (float)strength;\n');
    fprintf(fid, '    const float x = tfmc_log10_strength(strength);\n');
    fprintf(fid, '    /* freq clamp to avoid log10(0) */\n');
    fprintf(fid, '    if(freq_hz < %.6gf) freq_hz = %.6gf;\n', single(cfg.minFreqHz), single(cfg.minFreqHz));
    fprintf(fid, '    const float y = log10f(freq_hz);\n\n');

    % Construir Delta según la fórmula con los coeficientes estimados
    % Nota: usamos los nombres en coefNames para generar términos.
    fprintf(fid, '    float Delta = 0.0f;\n');

    % Intercept
    idxIntercept = find(coefNames == "(Intercept)", 1);
    if ~isempty(idxIntercept)
        fprintf(fid, '    Delta += TFMC_INTERCEPT;\n');
    end

    % Para cada coeficiente (except intercept), sumar término
    for i=1:numel(coefNames)
        cn = coefNames(i);
        if cn == "(Intercept)", continue; end
        cMacro = matlabCoefToC(cn);
        term = coefTermToCExpr(cn);
        fprintf(fid, '    Delta += TFMC_%s * (%s);\n', cMacro, term);
    end

    fprintf(fid, '\n    return d + Delta;\n}\n\n');

    fprintf(fid, '#endif /* TFMINI_CALIB_COEFFS_H */\n');
    fclose(fid);
end

function out = matlabCoefToC(name)
    % Convierte nombres tipo "(Intercept)" "d" "x" "d:x" a macros
    s = char(name);
    s = strrep(s,'(Intercept)','INTERCEPT');
    s = strrep(s,':','_X_');
    s = strrep(s,' ','');
    s = upper(s);
    out = s;
end

function expr = coefTermToCExpr(name)
    % Mapea términos a expresiones C válidas
    switch char(name)
        case 'd'
            expr = 'd';
        case 'T'
            expr = 'T';
        case 'S'
            expr = 'S_raw';
        case 'x'
            expr = 'x';
        case 'y'
            expr = 'y';
        case 'd:x'
            expr = '(d * x)';
        case 'x:y'
            expr = '(x * y)';
        case 'd:S'
            expr = '(d * S_raw)';
        otherwise
            % Por si agregás más términos luego
            expr = '0.0f /* unsupported term */';
    end
end
