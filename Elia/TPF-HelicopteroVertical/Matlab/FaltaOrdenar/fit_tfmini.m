xlsx_path = "D:\auto\TPF-HelicopteroVertical\DatosLeidos.xlsx";
T = readtable(xlsx_path, "Sheet", 1, "VariableNamingRule","preserve");

% Si tus headers son estos:
%  dis_sensor, dist_real
x = T.("dis_sensor");     % Dist_cm del sensor
y = T.("dist_real");      % Dist_real

% limpiar NaN/Inf
m = isfinite(x) & isfinite(y);
x = x(m); y = y(m);
maxOrd = 6;
RMSE = nan(maxOrd,1);
models = cell(maxOrd,1);

for ord = 1:maxOrd
    ft = fittype(sprintf('poly%d', ord));
    models{ord} = fit(x, y, ft);                 % y = f(x)
    yhat = models{ord}(x);
    RMSE(ord) = sqrt(mean((y - yhat).^2));
end

disp(table((1:maxOrd)', RMSE, 'VariableNames', {'Order','RMSE'}))

[~,bestOrd] = min(RMSE);
best = models{bestOrd};
fprintf("Mejor orden = %d (RMSE=%.4f)\n", bestOrd, RMSE(bestOrd));
% ordenar por real para que se vea prolijo
[ys, idx] = sort(y);
xs = x(idx);
yh = best(xs);   % ojo: best recibe x, no y. (ver abajo)

% Mejor: evaluemos en x ordenado, y comparemos contra y real ordenado:
x_sorted = x(idx);
y_sorted = y(idx);
y_pred   = best(x_sorted);

figure; hold on; grid on;
plot(y_sorted, x_sorted, '.', 'MarkerSize', 7);      % sensor vs real
plot(y_sorted, y_pred,   '.', 'MarkerSize', 7);      % modelo vs real
mn = min(y_sorted); mx = max(y_sorted);
plot([mn mx],[mn mx],'k--','LineWidth',1.2);         % ideal
xlabel('Dist\_real [cm]');
ylabel('Distancia [cm]');
legend('Sensor (Dist\_cm)','Modelo (fit)','Ideal y=x','Location','best');
title(sprintf('fit(): poly%d', bestOrd));


% Asume que ya tenés: best = fit(x,y,'polyK');
K = bestOrd;                 % orden
p = coeffvalues(best);       % [p1 p2 ... p(K+1)] alta potencia -> constante

out_h = fullfile(pwd, "tfmini_poly_calib.h");
export_poly_to_h(out_h, p, "tfmini_correct_distance_cm_poly");
fprintf("OK: %s\n", out_h);

function export_poly_to_h(out_h, p, funcName)
    K = numel(p)-1;

    fid = fopen(out_h,'w'); assert(fid>0);
    c = onCleanup(@() fclose(fid));

    fprintf(fid,"#ifndef TFMINI_POLY_CALIB_H\n#define TFMINI_POLY_CALIB_H\n\n");
    fprintf(fid,"#include <stdint.h>\n");
    fprintf(fid,'#include \"arm_math.h\"   /* float32_t */\n\n');

    fprintf(fid,"/* y = p1*x^%d + p2*x^%d + ... + p%d*x + p%d */\n", K, K-1, K, K+1);
    fprintf(fid,"#define TFM_POLY_ORDER (%du)\n", K);
    for i=1:numel(p)
        fprintf(fid,"#define TFM_P%d ((float32_t)%.9gf)\n", i, single(p(i)));
    end
    fprintf(fid,"\n");

    % Horner: y = (...((p1*x + p2)*x + p3)*x ... ) + p_{K+1}
    fprintf(fid,"static inline float32_t %s(float32_t x)\n{\n", funcName);
    fprintf(fid,"    float32_t y = TFM_P1;\n");
    for i=2:numel(p)
        fprintf(fid,"    y = y * x + TFM_P%d;\n", i);
    end
    fprintf(fid,"    return y;\n}\n\n");

    fprintf(fid,"static inline uint16_t tfmini_calibrate_cm_poly(uint16_t dist_cm)\n{\n");
    fprintf(fid,"    float32_t y = %s((float32_t)dist_cm);\n", funcName);
    fprintf(fid,"    if (y < 0.0f) y = 0.0f;\n");
    fprintf(fid,"    if (y > 65535.0f) y = 65535.0f;\n");
    fprintf(fid,"    return (uint16_t)(y + 0.5f);\n}\n\n");

    fprintf(fid,"#endif /* TFMINI_POLY_CALIB_H */\n");
end
