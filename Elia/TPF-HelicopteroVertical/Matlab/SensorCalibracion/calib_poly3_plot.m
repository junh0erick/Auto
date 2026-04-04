function out = calib_poly3_plot(xlsxFile, sheetName)
% dist_real = poly3(dis_sensor)
% - Agrupa por dis_sensor (d) y promedia dist_real (y)
% - Fit de 3er orden (polyfit)
% - Plotea puntos + curva + residuales
%
% out.p  = [p3 p2 p1 p0]
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

y = double(T.dist_real);      % cm real
d = double(T.dis_sensor);     % cm sensor

ok = isfinite(y) & isfinite(d);
y = y(ok); d = d(ok);

% ===== promedios por dis_sensor =====
d_key = round(d);                 % agrupar por cm entero (más robusto)
[dg,~,g] = unique(d_key);

yM = accumarray(g, y, [], @mean); % promedio de real para cada d
nG = accumarray(g, 1);

minPerGroup = 5;
keep = nG >= minPerGroup;

dg = dg(keep);
yM = yM(keep);
nG = nG(keep);

% ordenar
[dg, idx] = sort(dg);
yM = yM(idx);
nG = nG(idx);

% ===== fit poly3 =====
deg = 3;
p = polyfit(dg, yM, deg);      % yM ≈ polyval(p, dg)
yhat = polyval(p, dg);

e = yhat - yM;
rmse = sqrt(mean(e.^2));
mae  = mean(abs(e));

fprintf("Poly%d (MEANS by dis_sensor) RMSE=%.4f | MAE=%.4f | Nd=%d\n", deg, rmse, mae, numel(dg));
disp("p = [p3 p2 p1 p0]"); disp(p);

% ===== plot =====
figure('Name','TFMini calib poly3 (means by dis_sensor)','Color','w');
scatter(dg, yM, 35, 'filled'); hold on; grid on;
xlabel('dis\_sensor (cm)');
ylabel('dist\_real mean (cm)');
title(sprintf('Poly%d fit | RMSE=%.3f cm | MAE=%.3f cm', deg, rmse, mae));

dx = linspace(min(dg), max(dg), 400);
plot(dx, polyval(p, dx), 'LineWidth', 2);
legend('Means by d','Poly3 fit','Location','best');

figure('Name','Residuals (poly3)','Color','w');
stem(dg, e, 'filled'); grid on;
xlabel('dis\_sensor (cm)');
ylabel('error (yhat - ymean) [cm]');
title('Residuals on means by dis\_sensor');

% ===== salida =====
out = struct();
out.p = p;
out.deg = deg;
out.f = @(din) polyval(p, double(din));
out.rmse_means = rmse;
out.mae_means  = mae;
out.means = table(dg, yM, nG, yhat, e, 'VariableNames', ...
    {'dis_sensor','dist_real_mean','N','yhat','err'});
end
