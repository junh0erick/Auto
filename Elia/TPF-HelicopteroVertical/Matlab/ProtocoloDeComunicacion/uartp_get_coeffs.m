function coeffs = uartp_get_coeffs(sp, timeout_s)
if nargin < 2, timeout_s = 2.0; end

rx = uartp_get_coeffs_raw(sp, timeout_s);  % uint8(64,1)
coeffs = typecast(uint8(rx(:)), "single");
coeffs = coeffs(:);
end
