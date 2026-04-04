function [b,a,meta] = uartp_parse_tf(coeffs25)
c = single(coeffs25(:));
assert(numel(c)==25, "coeffs25 debe tener 25 floats");

b = c(1:11).';
a = c(12:22).';

meta.order = c(23);
meta.N     = c(24);
meta.FsHz  = c(25);
end
