function out = calib_poly4_plot_byD(xlsxFile, sheetName)
% Calibración correcta para implementación: y = f(d)
% - Agrupa por dis_sensor (d) y promedia dist_real (y)
% - Fit poly4: y ≈ polyval(p, d)
% - Plots + residuales
%
% out.p  = [p4 p3 p2 p1 p0]
% out.f  = @(d) polyval(out.p, d)

if nargin < 2, sheetName = 1; end

T = readtable(xlsxFile, "Sheet", sheetName);
T.Properties.VariableNames = lower(string(T.Properties.VariableNames));

req = ["dist_real","dis_sensor"];
for k = 1:numel(req)
    if ~ismember(req(k), T.Properties.VariableNames)
        error("Falta columna: %s", req(k));
    end
end

y = double(T.dist_real);      % real (cm)
d = double(T.dis_sensor);     % sensor (cm)

ok = isfinite(y) & isfinite(d);
y = y(ok); d = d(ok);

% =========================================================
% 1) AGRUPAR POR d (dis_sensor)
%    - Si d es entero: agrupá directo
%    - Si d tiene decimales: redondeá o binning
% =========================================================

d_key = round(d);  % si querés más fino: floor(d) o bins custom

[dg,~,g] = unique(d_key);

% promedio robusto: podés cambiar @mean por @median si hay outliers
yM = accumarray(g, y, [], @mean);
nG = accumarray(g, 1);

minPerGroup = 5;
keep = nG >= minPerGroup;

dg = dg(keep);
yM = yM(keep);
nG = nG(keep);

% ordenar por d
[dg, idx] = sort(dg);
yM = yM(idx);
nG = nG(idx);

% =========================================================
% 2) FIT poly4 y = f(d)
% =========================================================
deg = 4;
p = polyfit(dg, yM, deg);
yhat = polyval(p, dg);

e = yhat - yM;
rmse = sqrt(mean(e.^2));
mae  = mean(abs(e));

fprintf("Poly%d (MEANS by dis_sensor) RMSE=%.4f | MAE=%.4f | Nd=%d\n", deg, rmse, mae, numel(dg));
disp("p = [p4 p3 p2 p1 p0]"); disp(p);

% debug: ejemplo en 110
fprintf("Predicción en d=110: %.3f cm\n", polyval(p, 110));

% =========================================================
% 3) PLOTS
% =========================================================
figure('Name','TFMini calib poly4 (y vs d means)','Color','w');
scatter(dg, yM, 35, 'filled'); hold on; grid on;
xlabel('dis\_sensor (cm)');
ylabel('dist\_real promedio (cm)');
title(sprintf('Poly%d fit on means by dis\\_sensor | RMSE=%.3f | MAE=%.3f', deg, rmse, mae));

dx = linspace(min(dg), max(dg), 400);
plot(dx, polyval(p, dx), 'LineWidth', 2);
legend('Means by d','Poly4 fit','Location','best');

figure('Name','Residuals (poly4 by d)','Color','w');
stem(dg, e, 'filled'); grid on;
xlabel('dis\_sensor (cm)');
ylabel('error (yhat - ymean) [cm]');
title('Residuals on means by dis\_sensor');

% =========================================================
% 4) OUTPUT
% =========================================================
out = struct();
out.p = p;
out.deg = deg;
out.f = @(din) polyval(p, double(din));
out.rmse_means = rmse;
out.mae_means  = mae;
out.means_table = table(dg, yM, nG, yhat, e, 'VariableNames', ...
    {'dis_sensor_cm','dist_real_mean','N','yhat','err'});
end
