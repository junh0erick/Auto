function [A,B,C,D,L,K,Ki,meta] = uartp_parse_ss(coeffs25)
c = single(coeffs25(:));
assert(numel(c)==25, "coeffs25 debe tener 25 floats");

% A fue empaquetada por filas (fila-major)
A = reshape(c(1:9), 3, 3).';
B = c(10:12);
C = c(13:15).';
D = c(16);
L = c(17:19);
K = c(20:22);
Ki = c(23);

meta.N    = c(24);
meta.FsHz = c(25);
end
