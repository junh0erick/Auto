function coeffs = uartp_make_tf(num, den, arg3, arg4, arg5)
% uartp_make_tf  Arma PAQUETE FIJO de 25 floats para TF IIR DF2T hasta orden 10.
%
% Layout (1-based):
%   coeffs( 1..11) = b0..b10
%   coeffs(12..22) = a0..a10
%   coeffs(23)     = order  (0..10)
%   coeffs(24)     = N      (float pero representa entero)
%   coeffs(25)     = FsHz   (Hz)
%
% USO NUEVO (recomendado):
%   coeffs = uartp_make_tf(b, a, order, N, FsHz)
%   - b y a pueden venir como vectores de largo (order+1). Lo demás se rellena con 0.
%
% COMPATIBILIDAD (viejo):
%   coeffs = uartp_make_tf(num6, den6, N, FsHz)  % 4 args -> order se infiere
%
% Notas:
% - "order" define hasta qué término se usa: b0..b_order y a0..a_order. El resto se fuerza a 0.
% - Si no pasás order explícito, se infiere como max(length(num),length(den))-1.
% - Máximo order = 10 (=> 11 coef por num/den).

TF_MAX_ORDER = 10;

% -------------------------
% Defaults / parse args
% -------------------------
if nargin < 2
    error("uartp_make_tf requiere al menos num y den.");
end

% Normalizar a fila
num = double(num(:).');
den = double(den(:).');

if isempty(num), num = 0; end
if isempty(den), den = 0; end

order = []; N = 0; FsHz = 0;

if nargin == 2
    % Solo coef -> inferir order
elseif nargin == 3
    % Ambiguo: puede ser order o N
    if isscalar(arg3) && isfinite(arg3) && arg3 >= 0 && arg3 <= TF_MAX_ORDER && abs(arg3 - round(arg3)) < 1e-6
        order = round(arg3);
    else
        N = arg3;
    end
elseif nargin == 4
    % Modo compat (viejo): (num6, den6, N, Fs/period)
    N    = arg3;
    FsHz = arg4;
else
    % Modo nuevo explícito: (num, den, order, N, FsHz)
    order = arg3;
    N     = arg4;
    FsHz  = arg5;
end

% Inferir order si no vino explícito
if isempty(order)
    order = max(numel(num), numel(den)) - 1;
end

% Sanitizar order
if ~isscalar(order) || ~isfinite(order)
    error("order inválido.");
end
order = round(double(order));
order = max(0, min(TF_MAX_ORDER, order));

% Sanitizar N y FsHz
if isempty(N) || ~isscalar(N) || ~isfinite(N), N = 0; end
if isempty(FsHz) || ~isscalar(FsHz) || ~isfinite(FsHz), FsHz = 0; end

% -------------------------
% Construir paquete 25
% -------------------------
coeffs = single(zeros(25,1));

% Rellenar b y a hasta 11 con 0
b = zeros(1, TF_MAX_ORDER+1);
a = zeros(1, TF_MAX_ORDER+1);

nb = min(numel(num), TF_MAX_ORDER+1);
na = min(numel(den), TF_MAX_ORDER+1);

b(1:nb) = num(1:nb);
a(1:na) = den(1:na);

% Forzar a 0 todo lo que esté por encima del order (para no enviar basura)
if order < TF_MAX_ORDER
    b(order+2:end) = 0;
    a(order+2:end) = 0;
end

coeffs(1:11)   = single(b(:));
coeffs(12:22)  = single(a(:));
coeffs(23)     = single(order);
coeffs(24)     = single(N);
coeffs(25)     = single(FsHz);

end
