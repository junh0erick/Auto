function uartp_stop(sp, wait_back_to_command, timeout_s)
% En CONTROL: ISR responde 'K' y luego vuelve a COMMAND cuando termina stop suave.
% En COMMAND: devuelve 'K' inmediato.
if nargin < 2 || isempty(wait_back_to_command), wait_back_to_command = false; end
if nargin < 3, timeout_s = 10.0; end

write(sp, uint8('s'), "uint8");
b = uartp_ll_readexact(sp, 1, timeout_s);
assert(b == uint8('K'), "stop no devolvió K (rsp=%c)", char(b));

if wait_back_to_command
    ok = uartp_ll_wait_back_to_command(sp, 5.0);
    assert(ok, "No volvió a COMMAND a tiempo");
end
end
