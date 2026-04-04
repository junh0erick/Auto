function uartp_init(sp, u0, timeout_s)
if nargin < 3, timeout_s = 2.0; end
u0 = single(u0);

rsp = uartp_ll_cmd_wait(sp, 'i', timeout_s);
assert(rsp == uint8('R'), "i no devolvió R (rsp=%c)", char(rsp));

uartp_ll_send_words4(sp, typecast(u0,"uint8"), timeout_s);

final = uartp_ll_readexact(sp, 1, timeout_s);
assert(final == uint8('K'), "i no terminó con K (rsp=%c)", char(final));
end
