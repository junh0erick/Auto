function rx = uartp_get_coeffs_raw(sp, timeout_s, nWords)
% uartp_get_coeffs_raw  Lee coeficientes crudos.
%
% host: 't' -> espera 'S' -> recibe 4*nWords bytes -> espera 'K'

if nargin < 2 || isempty(timeout_s), timeout_s = 2.0; end
if nargin < 3 || isempty(nWords),    nWords = 25;  end

uartp_ll_drain_rx(sp);

rsp = uartp_ll_cmd_wait(sp, 't', timeout_s);
assert(rsp == uint8('S'), "t no devolvió S (rsp=%c)", char(rsp));

rx = uartp_ll_readexact(sp, 4*nWords, timeout_s);  % bytes crudos

final = uartp_ll_readexact(sp, 1, timeout_s);
assert(final == uint8('K'), "t no terminó con K (rsp=%c)", char(final));
end
