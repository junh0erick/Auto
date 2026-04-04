function uartp_setmode(sp, mode, timeout_s)
% mode: 0..4
% 0 TF
% 1 SS predictor sin integrador
% 2 SS actual sin integrador
% 3 SS predictor con integrador
% 4 SS actual con integrador
if nargin < 3, timeout_s = 2.0; end
assert(mode>=0 && mode<=5 && floor(mode)==mode, "mode debe ser entero 0..5");

rsp = uartp_ll_cmd_wait(sp, 'm', timeout_s);
assert(rsp == uint8('R'), "m no devolvió R (rsp=%c)", char(rsp));

uartp_ll_send_words4(sp, uint8([mode 0 0 0]), timeout_s);

final = uartp_ll_readexact(sp, 1, timeout_s);
assert(final == uint8('K'), "m no terminó con K (rsp=%c)", char(final));
end
