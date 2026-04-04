function uartp_reset(sp, timeout_s)
if nargin < 2, timeout_s = 1.0; end

rsp = uartp_ll_cmd_wait(sp, 'r', timeout_s);
assert(rsp == uint8('K'), "Reset no devolvió K (rsp=%c)", char(rsp));

pause(0.40); % tiempo para reboot
uartp_ll_drain(sp);
end
