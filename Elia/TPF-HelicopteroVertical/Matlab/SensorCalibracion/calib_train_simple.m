function out = calib_train_simple(xlsxFile, sheetName)
% PROMEDIOS por dist_real + regresión NO lineal acotada (fitnlm) con pesos.
% - Penaliza más los errores en valores altos (y grandes).
% - No permite predicciones por encima del límite físico: y in (YMIN, YMAX).
%
% Variables:
%   y   = dist_real (cm)
%   d   = dis_sensor (cm)
%   q   = intensidaddeluz
%   lnq = log1p(q)
%   T   = temperatura
%   F   = freq_prom  (ojo: si en runtime no es "fps", renombralo)
%   d2  = d.^2
%   interacciones: d*lnq, d*T, d*F
%
% Modelo acotado:
%   y = YMIN + (YMAX-YMIN) * sigmoid( b0 + b1*d + b2*d2 + b3*lnq + b4*T + b5*F
%                                      + b6*d*lnq + b7*d*T + b8*d*F )
%
% Pesos:
%   w(y) crece fuerte con y -> errores altos pesan más.

if nargin < 2, sheetName = 1; end

% ===== Parámetros "físicos" del rango =====
YMIN = 12;     % cm (ajustá si querés)
YMAX = 134;    % cm

% ===== Fuerza de penalización a altos =====
% p controla qué tan agresivo (2..6 típico). gain escala.
p = 4;         % más alto => más peso en distancias grandes
gain = 30;     % escala del peso (10..80 típico)

T = readtable(xlsxFile, "Sheet", sheetName);
T.Properties.VariableNames = lower(string(T.Properties.VariableNames));

req = ["dist_real","dis_sensor","intensidaddeluz","temperatura","freq_prom"];
for k = 1:numel(req)
    if ~ismember(req(k), T.Properties.VariableNames)
        error("Falta columna: %s", req(k));
    end
end

% --- variables ---
y  = double(T.dist_real);
d  = double(T.dis_sensor);
q  = double(T.intensidaddeluz); q(q<0)=0;
tC = double(T.temperatura);
F  = double(T.freq_prom);

lnq = log1p(q);

% limpiar NaN/Inf
ok = isfinite(y) & isfinite(d) & isfinite(lnq) & isfinite(tC) & isfinite(F);
y=y(ok); d=d(ok); lnq=lnq(ok); tC=tC(ok); F=F(ok);

% ===== PROMEDIOS por dist_real =====
[yg,~,g] = unique(y);

dM   = accumarray(g, d,   [], @mean);
lnqM = accumarray(g, lnq, [], @mean);
tM   = accumarray(g, tC,  [], @mean);
FM   = accumarray(g, F,   [], @mean);
nG   = accumarray(g, 1);

% tirar grupos con pocas muestras
minPerGroup = 5;
keep = nG >= minPerGroup;

yg   = yg(keep);
dM   = dM(keep);
lnqM = lnqM(keep);
tM   = tM(keep);
FM   = FM(keep);
nG   = nG(keep);

Ng = numel(yg);

% ===== HOLDOUT por grupos =====
cv = cvpartition(Ng,'HoldOut',0.2);
tr = training(cv);
va = test(cv);

Tbl_tr = table(yg(tr), dM(tr), lnqM(tr), tM(tr), FM(tr), ...
    'VariableNames', {'y','d','lnq','T','F'});
Tbl_va = table(yg(va), dM(va), lnqM(va), tM(va), FM(va), ...
    'VariableNames', {'y','d','lnq','T','F'});

Tbl_tr.d2 = Tbl_tr.d.^2;
Tbl_va.d2 = Tbl_va.d.^2;

% ===== PESOS (penaliza alto) =====
% normalizar y al rango [0..1]
yn_tr = (Tbl_tr.y - YMIN) / (YMAX - YMIN);
yn_tr = max(0, min(1, yn_tr)); % solo para pesos, no afecta el modelo
w_tr = 1 + gain * (yn_tr.^p);

% ===== Modelo NO lineal acotado =====
% z = b0 + b1*d + b2*d2 + b3*lnq + b4*T + b5*F + b6*d*lnq + b7*d*T + b8*d*F
% y = YMIN + (YMAX-YMIN)*sigmoid(z)
modelfun = @(b, X) local_model_bounded(b, X, YMIN, YMAX);

% armar matriz X (orden fijo)
Xtr = [Tbl_tr.d, Tbl_tr.d2, Tbl_tr.lnq, Tbl_tr.T, Tbl_tr.F, ...
       Tbl_tr.d.*Tbl_tr.lnq, Tbl_tr.d.*Tbl_tr.T, Tbl_tr.d.*Tbl_tr.F];

Xva = [Tbl_va.d, Tbl_va.d2, Tbl_va.lnq, Tbl_va.T, Tbl_va.F, ...
       Tbl_va.d.*Tbl_va.lnq, Tbl_va.d.*Tbl_va.T, Tbl_va.d.*Tbl_va.F];

% start point (evita warning random start)
b0 = zeros(9,1);  % [b0..b8]
% truco: arrancar cerca del centro del rango
b0(1) = 0;        % intercept en z

opts = statset('fitnlm');
opts.MaxIter = 500;
opts.TolFun  = 1e-10;
opts.TolX    = 1e-10;

mdl = fitnlm(Xtr, Tbl_tr.y, modelfun, b0, ...
    'Weights', w_tr, ...
    'Options', opts);


% ===== Validación =====
yhat = predict(mdl, Xva);
err = yhat - Tbl_va.y;
rmse = sqrt(mean(err.^2));
mae  = mean(abs(err));

% métricas ponderadas (para ver si realmente mejora arriba)
yn_va = (Tbl_va.y - YMIN) / (YMAX - YMIN);
yn_va = max(0, min(1, yn_va));
w_va = 1 + gain * (yn_va.^p);
rmse_w = sqrt(sum(w_va.*(err.^2))/sum(w_va));
mae_w  = sum(w_va.*abs(err))/sum(w_va);

fprintf("Holdout(GROUPS) RMSE=%.4f | MAE=%.4f | wRMSE=%.4f | wMAE=%.4f | Ng_tr=%d Ng_va=%d\n", ...
    rmse, mae, rmse_w, mae_w, sum(tr), sum(va));
disp(mdl)

% ===== salida =====
out = struct();
out.mdl = mdl;
out.YMIN = YMIN;
out.YMAX = YMAX;
out.weights = struct("p",p,"gain",gain);
out.metrics = struct("RMSE",rmse,"MAE",mae,"wRMSE",rmse_w,"wMAE",mae_w,"Ng_train",sum(tr),"Ng_valid",sum(va));
out.coeff = mdl.Coefficients;
out.groups = struct("yg",yg,"dM",dM,"lnqM",lnqM,"tM",tM,"fM",FM,"nG",nG);

% predict SIN clamp (acotado por construcción)
out.predict = @(d_in, q_in, t_in, f_in) local_predict(mdl, d_in, q_in, t_in, f_in, YMIN, YMAX);

end

% ===== helpers =====

function y = local_model_bounded(b, X, YMIN, YMAX)
% X: [d d2 lnq T F d*lnq d*T d*F] (8 cols)
d    = X(:,1);
d2   = X(:,2);
lnq  = X(:,3);
T    = X(:,4);
F    = X(:,5);
dlnq = X(:,6);
dT   = X(:,7);
dF   = X(:,8);

z = b(1) ...
  + b(2)*d ...
  + b(3)*d2 ...
  + b(4)*lnq ...
  + b(5)*T ...
  + b(6)*F ...
  + b(7)*dlnq ...
  + b(8)*dT ...
  + b(9)*dF;

% sigmoid estable
s = 1 ./ (1 + exp(-z));

y = YMIN + (YMAX - YMIN) * s;
end

function y = local_predict(mdl, d_in, q_in, t_in, f_in, YMIN, YMAX)
d_in = double(d_in);
q_in = double(q_in); q_in(q_in<0)=0;
lnq_in = log1p(q_in);
t_in = double(t_in);
f_in = double(f_in);

d2 = d_in.^2;
X = [d_in(:), d2(:), lnq_in(:), t_in(:), f_in(:), ...
     (d_in(:).*lnq_in(:)), (d_in(:).*t_in(:)), (d_in(:).*f_in(:))];

y = predict(mdl, X);
y = reshape(y, size(d_in));

% NO clamp: el modelo ya está en (YMIN,YMAX) por construcción
% Nota: numéricamente puede acercarse muchísimo a los bordes pero no pasarse.
end
