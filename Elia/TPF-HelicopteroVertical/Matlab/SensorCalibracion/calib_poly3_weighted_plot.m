function out = calib_poly3_weighted_plot(xlsxFile, sheetName)
% Polinomio orden 3 con CASTIGO FUERTE en el extremo alto.
% - Promedios por dist_real (reduce ruido)
% - Ajuste: y = p0 + p1*d + p2*d^2 + p3*d^3   (d = dis_sensor)
% - Weighted Least Squares: pesos crecen con dist_real (prioriza 12..134 final)
% - Plots + métricas

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

% ===== promedios por dist_real =====
[yg,~,g] = unique(y);
dM = accumarray(g, d, [], @mean);
nG = accumarray(g, 1);

minPerGroup = 5;
keep = nG >= minPerGroup;
yg = yg(keep);
dM = dM(keep);
nG = nG(keep);

% ===== pesos: castigar fuerte cerca del máximo real =====
ymin = min(yg);
ymax = max(yg);
r = (yg - ymin) / max(1e-9, (ymax - ymin));   % 0..1

% --- Ajustables ---
alpha = 200;      % fuerza del castigo (subí a 50..200 si querés más agresivo)
pow   = 5;       % curvatura del castigo (3..8 típico)
w = 1 + alpha*(r.^pow);

% opcional: todavía más peso al último 15% del rango
boostFrac = 0.15;
w(r > 1-boostFrac) = w(r > 1-boostFrac) * 3;

% ===== armar Vandermonde para poly3 =====
X = [ones(size(dM)), dM, dM.^2, dM.^3];  % [p0 p1 p2 p3]
W = diag(w);

% Weighted LS: p = (X' W X)^(-1) X' W y
p = (X' * W * X) \ (X' * W * yg);

% predicción sobre grupos
yhat = X * p;
e = yhat - yg;

% métricas globales
rmse = sqrt(mean(e.^2));
mae  = mean(abs(e));

% métricas “alto”: últimos 20% por dist_real
hi = r > 0.80;
rmse_hi = sqrt(mean(e(hi).^2));
mae_hi  = mean(abs(e(hi)));

fprintf("Poly3 WLS: RMSE=%.4f | MAE=%.4f | RMSE_hi=%.4f | MAE_hi=%.4f | Ng=%d\n", ...
    rmse, mae, rmse_hi, mae_hi, numel(yg));
disp("p = [p0 p1 p2 p3] para y = p0 + p1*d + p2*d^2 + p3*d^3");
disp(p.');

% ===== plot datos + curva =====
figure('Name','TFMini calib poly3 WLS','Color','w');
scatter(dM, yg, 35, 'filled'); hold on; grid on;
xlabel('dis\_sensor mean (cm)');
ylabel('dist\_real (cm)');
title(sprintf('Poly3 WLS (heavy tail) | RMSE=%.3f | RMSE_hi=%.3f', rmse, rmse_hi));

dx = linspace(min(dM), max(dM), 400);
Xdx = [ones(size(dx(:))), dx(:), dx(:).^2, dx(:).^3];
plot(dx, Xdx*p, 'LineWidth', 2);
legend('Group means','Poly3 WLS','Location','best');

% ===== residuales =====
figure('Name','Residuals poly3 WLS','Color','w');
stem(dM, e, 'filled'); grid on;
xlabel('dis\_sensor mean (cm)');
ylabel('error (yhat - y) [cm]');
title('Residuals on group means (poly3 WLS)');

% ===== salida =====
out = struct();
out.p = p(:);  % [p0 p1 p2 p3]
out.f = @(din) local_eval_poly3(p, din);
out.weights = w;
out.rmse_groups = rmse;
out.mae_groups = mae;
out.rmse_hi = rmse_hi;
out.mae_hi = mae_hi;
out.groups = table(yg, dM, nG, w, yhat, e, ...
    'VariableNames', {'dist_real','dis_sensor_mean','N','w','yhat','err'});

end

function y = local_eval_poly3(p, din)
d = double(din);
y = p(1) + p(2)*d + p(3)*d.^2 + p(4)*d.^3;
end
