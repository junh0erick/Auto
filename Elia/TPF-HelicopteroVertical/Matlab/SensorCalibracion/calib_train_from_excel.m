function out = calib_train_from_excel(xlsxFile, sheetName)

if nargin < 2, sheetName = 1; end

T = readtable(xlsxFile, "Sheet", sheetName);
T.Properties.VariableNames = lower(string(T.Properties.VariableNames));

req = ["freq_prom","dist_real","dis_sensor","intensidaddeluz","temperatura"];
for k = 1:numel(req)
    if ~ismember(req(k), T.Properties.VariableNames)
        error("Falta columna: %s", req(k));
    end
end

% --- Limpieza básica ---
T = T(all(isfinite(T{:,req}),2),:);

% RANGOS FÍSICOS / SANITY (ajustá si querés)
T = T(T.dist_real >= 12 & T.dist_real <= 134, :);
T = T(T.dis_sensor >= 0  & T.dis_sensor <= 250, :);
T = T(T.temperatura >= 0 & T.temperatura <= 100, :);   % mata el 1910

% lnq = ln(1+Lux) (NATURAL, no log10)
q = double(T.intensidaddeluz);
q(q < 0) = 0;
T.lnq = log(q + 1);

% --- Holdout ---
N = height(T);
cv = cvpartition(N,'HoldOut',0.2);
idxTr = training(cv);
idxVa = test(cv);

mdl = fitlm( ...
    T(idxTr,:), ...
    'dist_real ~ freq_prom + dis_sensor + temperatura + lnq + dis_sensor:temperatura + dis_sensor:lnq', ...
    'RobustOpts','on' );

% Métricas
yhat = predict(mdl, T(idxVa,:));
y    = T.dist_real(idxVa);
e    = yhat - y;

out = struct();
out.mdl = mdl;
out.beta = mdl.Coefficients.Estimate(:);
out.coeffNames = mdl.CoefficientNames(:);

out.metrics.RMSE = sqrt(mean(e.^2));
out.metrics.MAE  = mean(abs(e));
out.metrics.Ntrain = sum(idxTr);
out.metrics.Nvalid = sum(idxVa);

% ---- PARAMETROS LUT SEGUN EXCEL ----
out.lut_max   = uint32(max(q));     % <-- CLAVE: no 16383 si tu excel llega a 23432
out.lut_shift = uint32(7);          % step=128 -> 256 puntos aprox si max~32768
out.lut_size  = uint32(floor(double(out.lut_max)/double(bitshift(1,out.lut_shift))) + 1);

% Generar LUT ln(1+x) en escalones
step = double(bitshift(1,out.lut_shift));
x0   = (0:double(out.lut_size-1))*step;
x0(x0 > double(out.lut_max)) = double(out.lut_max);
out.lut = single(log(x0 + 1));

fprintf('Holdout RMSE = %.4f | MAE = %.4f | Ntr=%d Nva=%d\n', ...
    out.metrics.RMSE, out.metrics.MAE, out.metrics.Ntrain, out.metrics.Nvalid);
disp(mdl)

end
