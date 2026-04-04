function out = calib_quad_lnq_plot_byD(xlsxFile, sheetName)
% y = a + b*d + c*d^2 + k*ln(1+strength)
% Agrupa por dis_sensor (d) para entrenar y=f(d,lnq)

if nargin < 2, sheetName = 1; end

T = readtable(xlsxFile, "Sheet", sheetName);
T.Properties.VariableNames = lower(string(T.Properties.VariableNames));

req = ["dist_real","dis_sensor","intensidaddeluz"];
for k = 1:numel(req)
    if ~ismember(req(k), T.Properties.VariableNames)
        error("Falta columna: %s", req(k));
    end
end

y = double(T.dist_real);
d = double(T.dis_sensor);
q = double(T.intensidaddeluz); q(q<0)=0;
lnq = log1p(q);

ok = isfinite(y) & isfinite(d) & isfinite(lnq);
y=y(ok); d=d(ok); lnq=lnq(ok);

% ===== Agrupar por d (dis_sensor) =====
d_key = round(d);
[dg,~,g] = unique(d_key);

yM   = accumarray(g, y,   [], @mean);
lnqM = accumarray(g, lnq, [], @mean);
nG   = accumarray(g, 1);

minPerGroup = 5;
keep = nG >= minPerGroup;

dg   = dg(keep);
yM   = yM(keep);
lnqM = lnqM(keep);
nG   = nG(keep);

% ordenar
[dg, idx] = sort(dg);
yM   = yM(idx);
lnqM = lnqM(idx);
nG   = nG(idx);

% ===== Fit lineal en parámetros (no necesita fitnlm) =====
Tbl = table(yM, dg, lnqM, 'VariableNames', {'y','d','lnq'});
Tbl.d2 = Tbl.d.^2;

formula = 'y ~ 1 + d + d2 + lnq';

% Si tu MATLAB soporta RobustOpts en fitlm, dejalo:
% mdl = fitlm(Tbl, formula, 'RobustOpts','bisquare');

% Si te da error con RobustOpts, usá este:
mdl = fitlm(Tbl, formula);

yhat = predict(mdl, Tbl);
e = yhat - Tbl.y;
rmse = sqrt(mean(e.^2));
mae  = mean(abs(e));

fprintf("Quad+lnq (means by d) RMSE=%.4f | MAE=%.4f | Nd=%d\n", rmse, mae, height(Tbl));
disp(mdl)

fprintf("Ejemplo d=110: %.3f cm (usando lnq promedio del grupo)\n", ...
    predict(mdl, table(110, mean(lnqM), 110^2, 'VariableNames', {'d','lnq','d2'})));

% ===== Plots =====
figure('Name','Quad+lnq (means by d)','Color','w');
scatter(dg, yM, 35, 'filled'); hold on; grid on;
xlabel('dis\_sensor (cm)'); ylabel('dist\_real mean (cm)');
title(sprintf('y=a+b*d+c*d^2+k*lnq | RMSE=%.3f MAE=%.3f', rmse, mae));

% curva “típica”: usando lnq medio global
lnq_typ = mean(lnqM);
dx = linspace(min(dg), max(dg), 400)';
Tblx = table(dx, repmat(lnq_typ,size(dx)), dx.^2, 'VariableNames', {'d','lnq','d2'});
plot(dx, predict(mdl, Tblx), 'LineWidth', 2);
legend('Means','Curve (lnq típico)','Location','best');

figure('Name','Residuals Quad+lnq','Color','w');
stem(dg, e, 'filled'); grid on;
xlabel('dis\_sensor (cm)'); ylabel('error (yhat-ymean) [cm]');
title('Residuals on means by dis\_sensor');

% salida
out = struct();
out.mdl = mdl;
out.formula = string(formula);
out.metrics = struct("RMSE",rmse,"MAE",mae);
out.coeff = mdl.Coefficients;
out.lnq_typ = lnq_typ;
end
