function coeffs = uartp_make_ss(A,B,C,D,L,K,Ki,N,FsHz)
% uartp_make_ss  Arma PAQUETE FIJO de 25 floats para SS de 3 estados.
%
% Layout (1-based):
%  1..9   = A11 A12 A13 A21 A22 A23 A31 A32 A33
%  10..12 = B1 B2 B3
%  13..15 = C1 C2 C3
%  16     = D
%  17..19 = L1 L2 L3
%  20..22 = K1 K2 K3
%  23     = Ki
%  24     = N      (float, representa entero)
%  25     = FsHz   (Hz)

if nargin < 8 || isempty(N),    N = 0;    end
if nargin < 9 || isempty(FsHz), FsHz = 0; end

A  = single(A);
B  = single(B(:));
C  = single(C(:));   % aceptamos 1x3 o 3x1
D  = single(D);
L  = single(L(:));
K  = single(K(:));
Ki = single(Ki);

assert(isequal(size(A),[3 3]), "A debe ser 3x3");
assert(numel(B)==3, "B debe tener 3 elementos");
assert(numel(C)==3, "C debe tener 3 elementos (1x3 o 3x1)");
assert(numel(L)==3, "L debe tener 3 elementos");
assert(numel(K)==3, "K debe tener 3 elementos");

assert(isfinite(N) && N>=0, "N inválido");
assert(isfinite(FsHz) && FsHz>=0, "FsHz inválido");

coeffs = single(zeros(25,1));

% A (fila-major)
coeffs(1:9) = reshape(A.', 9, 1);   % OJO: reshape(A.',...) para fila-major

% B
coeffs(10:12) = B;

% C (si viene 3x1, lo ponemos como [C1 C2 C3])
coeffs(13:15) = C(:);

% D
coeffs(16) = D;

% L
coeffs(17:19) = L;

% K
coeffs(20:22) = K;

% Ki
coeffs(23) = Ki;

% Meta
coeffs(24) = single(N);
coeffs(25) = single(FsHz);

end
