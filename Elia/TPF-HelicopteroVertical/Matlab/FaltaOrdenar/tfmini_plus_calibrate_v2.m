% tfmini_plus_calibrate_v2.m
% ------------------------------------------------------------
% Lee Excel/CSV con:
%   Freq_prom, Freq_medida, dis_calc, dist_medida, Temperatura, Intensidad (Strength uint16)
%
% Hace:
%   1) Verificaciones (rangos, constantes, correlación, duplicados)
%   2) Construye variables: d, Delta, S(raw), x=log10(S), y=log10(freq)
%   3) Agrupa "escenarios" para repetibilidad + GroupKFold por grupos
%   4) Prueba una batería de modelos de complejidad creciente
%   5) Reporta métricas: RMSE/MAE (CV), RMSE/MAE (train), y "complejidad"
%   6) Selecciona el mejor por RMSE_CV (tie-break por complejidad)
%   7) Grafica antes/después (para el mejor)
%   8) Exporta header C con coeficientes + LUT log10(strength) (compacta)
%
% Requiere: Statistics and Machine Learning Toolbox
% ------------------------------------------------------------

clear; clc;

%% =================== CONFIG ===================
cfg.useFreqMeasured = true;

% Bins para agrupar escenario (ajustables)
cfg.bin.d_meas_cm = 1.0;
cfg.bin.temp_c    = 0.5;
cfg.bin.freq_hz   = 1.0;
cfg.bin.strength  = 50;

% Sanitización
cfg.minFreqHz   = 0.1;
cfg.minStrength = 1;
cfg.maxStrength = 65535;

% CV
cfg.Kfold = 5;

% Pesos por repetibilidad
cfg.useRepeatWeights = true;    % si no hay repetidos, cae a piso
cfg.sigmaFloorCm     = 1.0;     % fallback si no hay suficientes repetidos

% Export
cfg.exportHeader = 'tfmini_calib_coeffs.h';
cfg.exportFunc   = 'tfmini_correct_distance_cm';

% LUT log10(strength) para PSoC (compacta por defecto)
cfg.LUT.enable   = true;
cfg.LUT.compact  = true;
cfg.LUT.N        = 1024;  % 1024 suele ser buen compromiso memoria/precisión

% Selección del mejor:
% score = RMSE_CV + lambda*complexity (lambda chico, para desempatar)
cfg.lambdaComplexity = 0.005;

%% =================== SELECT FILE ===================
[file, path] = uigetfile({'*.xlsx;*.xls;*.csv','Data files (*.xlsx,*.xls,*.csv)'}, ...
                         'Seleccioná tu Excel/CSV de mediciones');
if isequal(file,0), error('No seleccionaste archivo.'); end
fname = fullfile(path,file);

%% =================== READ TABLE ===================
Traw = read_any_table(fname);
vars = string(Traw.Properties.VariableNames);

findcol = @(patterns) find(contains(lower(vars), lower(patterns), 'IgnoreCase', true), 1, 'first');

iFreqProm = findcol(["freq_prom","freq_prog","freq_program","freq_programada","freq_set"]);
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

Freq = cfg.useFreqMeasured * Freq_meas + (~cfg.useFreqMeasured) * Freq_prom;

%% =================== CLEAN + CLAMP ===================
ok = isfinite(Freq) & isfinite(d_calc) & isfinite(d_meas) & isfinite(Temp) & isfinite(Strength);
Freq=Freq(ok); d_calc=d_calc(ok); d_meas=d_meas(ok); Temp=Temp(ok); Strength=Strength(ok);

Strength = min(max(Strength, 0), cfg.maxStrength);
Freq     = max(Freq, cfg.minFreqHz);

% Derived
d = d_calc;
S = Strength;
x = log10(max(S, cfg.minStrength));     % log10(strength)
y = log10(max(Freq, cfg.minFreqHz));    % log10(freq)
T = Temp;

Delta = d_meas - d_calc;

%% =================== VERIFICATIONS ===================
fprintf('\n=== VERIFICACIONES RAPIDAS ===\n');
quick_stats('d_calc', d_calc);
quick_stats('d_meas', d_meas);
quick_stats('Delta',  Delta);
quick_stats('Strength(raw)', S);
quick_stats('log10(Strength)=x', x);
quick_stats('Freq(Hz)', Freq);
quick_stats('log10(Freq)=y', y);
quick_stats('Temp(C)', T);

% chequeo constantes (varianza ~ 0)
check_almost_constant('Freq', Freq);
check_almost_constant('Temp', T);
check_almost_constant('Strength', S);

% correlación simple (solo para diagnóstico)
fprintf('\nCorrelaciones (Pearson) para diagnostico:\n');
print_corr('Delta vs d', Delta, d);
print_corr('Delta vs x', Delta, x);
print_corr('Delta vs y', Delta, y);
print_corr('Delta vs T', Delta, T);
print_corr('x vs y', x, y);

% duplicados exactos (mismo set de entradas)
Xkey = [Freq(:), d(:), T(:), S(:)];
[~,~,ic] = unique(round(Xkey,6),'rows');  %#ok<*RNU>
dupCounts = accumarray(ic, 1);
nDups = sum(dupCounts>1);
fprintf('\nDuplicados exactos (mismas entradas): %d grupos (de %d muestras)\n', nDups, numel(d));

%% =================== GROUP SCENARIOS (repeats) ===================
q = @(v, step) round(v./step).*step;

key_dmeas = q(d_meas, cfg.bin.d_meas_cm);
key_temp  = q(T,      cfg.bin.temp_c);
key_freq  = q(Freq,   cfg.bin.freq_hz);
key_S     = q(S,      cfg.bin.strength);
G = findgroups(key_dmeas, key_temp, key_freq, key_S);

groupIDs = unique(G);
nGroups  = numel(groupIDs);
fprintf('Escenarios (grupos) detectados: %d\n', nGroups);

% repeatability sigma por grupo
nG  = splitapply(@numel, Delta, G);
sdG = splitapply(@std,   Delta, G);

% piso sigma
sd_floor = nanmedian(sdG(sdG>0 & isfinite(sdG)));
if ~isfinite(sd_floor) || sd_floor<=0
    sd_floor = cfg.sigmaFloorCm;
end

sigma_i = sdG(G);
sigma_i(~isfinite(sigma_i) | sigma_i<=0) = sd_floor;
sigma_i = max(sigma_i, sd_floor);

if cfg.useRepeatWeights
    w = 1 ./ (sigma_i.^2);
else
    w = ones(size(d));
end

%% =================== DATA TABLE ===================
Dtbl = table(Delta, d, T, S, x, y, G, w, d_meas, d_calc, Freq, ...
    'VariableNames', {'Delta','d','T','S','x','y','G','w','d_meas','d_calc','Freq'});

%% =================== MODEL SET (complexity ladder) ===================
% Complejidad = cantidad de términos (incluye intercept)
% Nota: demasiados términos con pocos datos -> rank deficiency (esperable).
models = {
  struct('name','C01_base_d',            'formula','Delta ~ 1 + d',                                   'complexity',2)
  struct('name','C02_d_x',               'formula','Delta ~ 1 + d + x',                               'complexity',3)
  struct('name','C03_d_x_T',             'formula','Delta ~ 1 + d + x + T',                           'complexity',4)
  struct('name','C04_d_x_y',             'formula','Delta ~ 1 + d + x + y',                           'complexity',4)
  struct('name','C05_d_x_T_y',           'formula','Delta ~ 1 + d + x + T + y',                       'complexity',5)

  % Interacciones moderadas
  struct('name','C06_d_x_dx',            'formula','Delta ~ 1 + d + x + d:x',                         'complexity',4)
  struct('name','C07_d_x_y_xy',          'formula','Delta ~ 1 + d + x + y + x:y',                     'complexity',5)
  struct('name','C08_d_x_y_dx',          'formula','Delta ~ 1 + d + x + y + d:x',                     'complexity',5)
  struct('name','C09_d_x_T_y_dx',        'formula','Delta ~ 1 + d + x + T + y + d:x',                 'complexity',6)
  struct('name','C10_d_x_T_y_xy',        'formula','Delta ~ 1 + d + x + T + y + x:y',                 'complexity',6)
  struct('name','C11_d_x_T_y_dx_xy',     'formula','Delta ~ 1 + d + x + T + y + d:x + x:y',           'complexity',7)

  % Variante con strength RAW (por si log no ayuda)
  struct('name','C12_d_S_y',             'formula','Delta ~ 1 + d + S + y',                           'complexity',4)
  struct('name','C13_d_S_y_dS',          'formula','Delta ~ 1 + d + S + y + d:S',                     'complexity',5)

  % “Mucho combo” (alto riesgo de rank deficiency con pocos datos)
  struct('name','C14_big_combo',         'formula','Delta ~ 1 + d + x + y + T + d:x + x:y + d:y + d:T + x:T', 'complexity',11)
};

%% =================== EVALUATION (GroupKFold) ===================
K = min(cfg.Kfold, nGroups);
if K < 2
    warning('Muy pocos grupos para CV. Se evaluará sin CV (train only).');
end

res = [];
for m = 1:numel(models)
    spec = models{m};

    if K >= 2
        [rmseCV, maeCV, nBadFits] = eval_model_groupkfold(Dtbl, spec.formula, K);
    else
        rmseCV = NaN; maeCV = NaN; nBadFits = 0;
    end

    % Fit final en todo (para tener coef y train error)
    [lm, warnRank] = safe_fitlm(Dtbl, spec.formula);

    predDelta = predict(lm, Dtbl);
    d_corr = Dtbl.d_calc + predDelta;
    e = d_corr - Dtbl.d_meas;

    rmseTrain = sqrt(mean(e.^2));
    maeTrain  = mean(abs(e));

    score = (rmseCV) + cfg.lambdaComplexity * spec.complexity;
    if isnan(score)
        score = rmseTrain + cfg.lambdaComplexity * spec.complexity;
    end

    res = [res; table(string(spec.name), string(spec.formula), spec.complexity, ...
        rmseCV, maeCV, rmseTrain, maeTrain, score, warnRank, nBadFits, ...
        'VariableNames', {'Name','Formula','Complexity','RMSE_CV','MAE_CV','RMSE_Train','MAE_Train','Score','RankDef','BadFits'})]; %#ok<AGROW>
end

res = sortrows(res, 'Score', 'ascend');

fprintf('\n=== RANKING DE MODELOS (menor Score mejor) ===\n');
disp(res(:,{'Name','Complexity','RMSE_CV','MAE_CV','RMSE_Train','MAE_Train','RankDef','BadFits','Score'}));

bestName = res.Name(1);
bestFormula = res.Formula(1);
fprintf('>>> MEJOR: %s | %s\n', bestName, bestFormula);

%% =================== FIT BEST + PLOTS ===================
[lmBest, warnRank] = safe_fitlm(Dtbl, char(bestFormula));
disp(lmBest);

predDeltaAll = predict(lmBest, Dtbl);
d_raw  = Dtbl.d_calc;
d_true = Dtbl.d_meas;
d_corr = d_raw + predDeltaAll;

e_raw  = d_raw  - d_true;
e_corr = d_corr - d_true;

figure; scatter(d_true, d_raw, 20, 'filled'); hold on;
plot([min(d_true) max(d_true)], [min(d_true) max(d_true)], 'k--', 'LineWidth', 2);
grid on; xlabel('dist\_medida [cm]'); ylabel('dis\_calc [cm]');
title('Antes: sensor vs ground truth');

figure; scatter(d_true, d_corr, 20, 'filled'); hold on;
plot([min(d_true) max(d_true)], [min(d_true) max(d_true)], 'k--', 'LineWidth', 2);
grid on; xlabel('dist\_medida [cm]'); ylabel('d\_corr [cm]');
title(sprintf('Despues (mejor modelo: %s)', bestName));

figure; histogram(e_raw, 30); grid on;
xlabel('error = dis\_calc - dist\_medida [cm]'); title('Error antes');

figure; histogram(e_corr, 30); grid on;
xlabel('error = d\_corr - dist\_medida [cm]'); title('Error despues');

figure; scatter(Dtbl.d_meas, e_corr, 18, Dtbl.S, 'filled');
grid on; xlabel('dist\_medida [cm]'); ylabel('error residual [cm]');
title('Residual vs distancia (color = Strength)'); colorbar;

%% =================== EXPORT BEST ===================
coefNames = string(lmBest.Coefficients.Properties.RowNames);
coefVals  = lmBest.Coefficients.Estimate;

export_tfmini_header_v2(cfg, char(bestName), char(bestFormula), coefNames, coefVals);
fprintf('\nOK. Header exportado: %s\n', fullfile(pwd, cfg.exportHeader));

%% =================== FUNCTIONS ===================

function T = read_any_table(fname)
    [~,~,ext] = fileparts(fname);
    switch lower(ext)
        case {'.xlsx','.xls'}
            T = readtable(fname, 'PreserveVariableNames', true);
        case '.csv'
            T = readtable(fname, 'PreserveVariableNames', true);
        otherwise
            error('Extensión no soportada: %s', ext);
    end
end

function quick_stats(name, v)
    v = v(:);
    fprintf('%-16s: n=%d  min=%.4g  max=%.4g  mean=%.4g  std=%.4g\n', ...
        name, numel(v), min(v), max(v), mean(v), std(v));
end

function check_almost_constant(name, v)
    if std(v) < 1e-12
        fprintf('!!! ALERTA: %s parece constante (std≈0). Terminos con %s no se pueden identificar.\n', name, name);
    end
end

function print_corr(name, a, b)
    a=a(:); b=b(:);
    if numel(a) ~= numel(b)
        fprintf('%s: size mismatch\n', name);
        return;
    end
    c = corr(a, b, 'type', 'Pearson', 'rows', 'complete');
    fprintf('  %-16s: %.4f\n', name, c);
end

function [rmseMean, maeMean, nBadFits] = eval_model_groupkfold(Dtbl, formula, K)
    groupIDs = unique(Dtbl.G);
    nGroups  = numel(groupIDs);
    cv = cvpartition(nGroups, 'KFold', K);

    rmse = zeros(K,1);
    mae  = zeros(K,1);
    nBadFits = 0;

    for k = 1:K
        testGroupIdx = test(cv,k);
        testGroups   = groupIDs(testGroupIdx);

        isTest = ismember(Dtbl.G, testGroups);
        isTrain= ~isTest;

        trainTbl = Dtbl(isTrain,:);
        testTbl  = Dtbl(isTest,:);

        [lm, warnRank] = safe_fitlm(trainTbl, formula);
        nBadFits = nBadFits + warnRank;

        predDelta = predict(lm, testTbl);
        d_corr = testTbl.d_calc + predDelta;
        e = d_corr - testTbl.d_meas;

        rmse(k) = sqrt(mean(e.^2));
        mae(k)  = mean(abs(e));
    end

    rmseMean = mean(rmse);
    maeMean  = mean(mae);
end

function [lm, warnRank] = safe_fitlm(T, formula)
    warnRank = 0;
    warnId = 'stats:LinearModel:RankDefDesignMat';
    st = warning('query', warnId);
    warning('off', warnId);
    cleanup = onCleanup(@() warning(st.state, warnId));

    lastwarn('');
    lm = fitlm(T, formula, 'Weights', T.w);
    [msg, id] = lastwarn;

    if strcmp(id, warnId)
        warnRank = 1;
    end
end

function export_tfmini_header_v2(cfg, bestName, bestFormula, coefNames, coefVals)
    fid = fopen(cfg.exportHeader,'w');
    if fid<0, error('No pude escribir %s', cfg.exportHeader); end

    fprintf(fid, '/* Auto-generated by tfmini_plus_calibrate_v2.m */\n');
    fprintf(fid, '/* Best model: %s */\n', bestName);
    fprintf(fid, '/* Formula: %s */\n', bestFormula);
    fprintf(fid, '#ifndef TFMINI_CALIB_COEFFS_H\n#define TFMINI_CALIB_COEFFS_H\n\n');
    fprintf(fid, '#include <stdint.h>\n#include <math.h>\n\n');

    fprintf(fid, '/* Coefficients (float) */\n');
    for i=1:numel(coefVals)
        nm = coef_to_macro(coefNames(i));
        fprintf(fid, '#define TFMC_%s (%.9gf)\n', nm, single(coefVals(i)));
    end
    fprintf(fid, '\n');

    % LUT log10(strength)
    if cfg.LUT.enable
        if cfg.LUT.compact
            N = cfg.LUT.N;
            fprintf(fid, '/* Compact LUT log10(strength) with linear interpolation.\n');
            fprintf(fid, '   Size: %d. strength in [0..65535], clamp 0->1.\n', N);
            fprintf(fid, '*/\n');
            fprintf(fid, 'static const float TFMC_LOG10_LUT[%d] = {\n', N);
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
        else
            % (no compact) omitido en v2, porque casi seguro es pesado en PSoC
            fprintf(fid, '/* Full LUT omitted (cfg.LUT.compact=true recommended) */\n\n');
        end
    end

    % Función de corrección (soporta términos más comunes)
    fprintf(fid, '/* Apply correction: returns corrected distance (cm) */\n');
    fprintf(fid, 'static inline float %s(float dis_calc_cm, uint16_t strength, float temp_c, float freq_hz)\n{\n', cfg.exportFunc);
    fprintf(fid, '    const float d = dis_calc_cm;\n');
    fprintf(fid, '    const float T = temp_c;\n');
    fprintf(fid, '    const float S = (float)strength;\n');
    fprintf(fid, '    const float x = tfmc_log10_strength(strength);\n');
    fprintf(fid, '    if(freq_hz < %.6gf) freq_hz = %.6gf;\n', single(cfg.minFreqHz), single(cfg.minFreqHz));
    fprintf(fid, '    const float y = log10f(freq_hz);\n');
    fprintf(fid, '    float Delta = 0.0f;\n');
    fprintf(fid, '    Delta += TFMC_INTERCEPT;\n');

    for i=1:numel(coefNames)
        cn = coefNames(i);
        if cn == "(Intercept)", continue; end
        macro = coef_to_macro(cn);
        term  = term_to_expr(cn);
        fprintf(fid, '    Delta += TFMC_%s * (%s);\n', macro, term);
    end
    fprintf(fid, '    return d + Delta;\n}\n\n');

    fprintf(fid, '#endif /* TFMINI_CALIB_COEFFS_H */\n');
    fclose(fid);
end

function nm = coef_to_macro(cn)
    s = char(cn);
    s = strrep(s,'(Intercept)','INTERCEPT');
    s = strrep(s,':','_X_');
    s = upper(regexprep(s,'[^A-Z0-9_]', ''));
    nm = s;
end

function expr = term_to_expr(cn)
    switch char(cn)
        case 'd'
            expr = 'd';
        case 'T'
            expr = 'T';
        case 'S'
            expr = 'S';
        case 'x'
            expr = 'x';
        case 'y'
            expr = 'y';
        case 'd:x'
            expr = '(d*x)';
        case 'x:y'
            expr = '(x*y)';
        case 'd:y'
            expr = '(d*y)';
        case 'd:T'
            expr = '(d*T)';
        case 'x:T'
            expr = '(x*T)';
        case 'd:S'
            expr = '(d*S)';
        otherwise
            expr = '0.0f /* unsupported */';
    end
end
