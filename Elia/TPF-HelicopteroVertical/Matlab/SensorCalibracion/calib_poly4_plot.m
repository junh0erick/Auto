function out = calib_poly4_plot(xlsxFile, sheetName)
% Modelo MUY simple: dist_real = poly4(dis_sensor)
% - Usa promedios por dist_real (para sacar ruido)
% - Fit de 4to orden (polyfit)
% - Plotea puntos + curva + residuales
%
% out.p  = [p4 p3 p2 p1 p0]
% out.f  = @(d) polyval(out.p, d)
% out.rmse_groups / out.mae_groups sobre los promedios

if nargin < 2, sheetName = 1; end

T = readtable(xlsxFile, "Sheet", sheetName);
T.Properties.VariableNames = lower(string(T.Properties.VariableNames));

req = ["dist_real","dis_sensor"];
for k = 1:numel(req)
    if ~ismember(req(k), T.Properties.VariableNames)
        error("Falta columna: %s", req(k));
    end
end

y = double(T.dist_real);      % cm (real)
d = double(T.dis_sensor);     % cm (sensor)

ok = isfinite(y) & isfinite(d);
y = y(ok); d = d(ok);

% ===== promedios por dist_real (tu estilo) =====
[yg,~,g] = unique(y);
dM = accumarray(g, d, [], @mean);
nG = accumarray(g, 1);

minPerGroup = 5;
keep = nG >= minPerGroup;
yg = yg(keep);
dM = dM(keep);
nG = nG(keep);

% ===== fit poly 4 =====
deg = 4;
p = polyfit(dM, yg, deg);         % yg ≈ polyval(p, dM)
yhat = polyval(p, dM);

e = yhat - yg;
rmse = sqrt(mean(e.^2));
mae  = mean(abs(e));

fprintf("Poly%d (GROUP MEANS) RMSE=%.4f | MAE=%.4f | Ng=%d\n", deg, rmse, mae, numel(yg));
disp("p = [p4 p3 p2 p1 p0]"); disp(p);

% ===== plot =====
figure('Name','TFMini calib poly4 (dist_real vs dis_sensor)','Color','w');
scatter(dM, yg, 35, 'filled'); hold on; grid on;
xlabel('dis\_sensor (cm)');
ylabel('dist\_real (cm)');
title(sprintf('Poly%d fit on group means | RMSE=%.3f cm | MAE=%.3f cm', deg, rmse, mae));

% curva suave
dx = linspace(min(dM), max(dM), 400);
plot(dx, polyval(p, dx), 'LineWidth', 2);

legend('Group means','Poly4 fit','Location','best');

% residuales
figure('Name','Residuals (poly4)','Color','w');
stem(dM, e, 'filled'); grid on;
xlabel('dis\_sensor mean (cm)');
ylabel('error (yhat - y) [cm]');
title('Residuals on group means');

% ===== salida =====
out = struct();
out.p = p;
out.deg = deg;
out.f = @(din) polyval(p, double(din));
out.rmse_groups = rmse;
out.mae_groups  = mae;
out.groups = table(yg, dM, nG, yhat, e, 'VariableNames', ...
    {'dist_real','dis_sensor_mean','N','yhat','err'});
end
