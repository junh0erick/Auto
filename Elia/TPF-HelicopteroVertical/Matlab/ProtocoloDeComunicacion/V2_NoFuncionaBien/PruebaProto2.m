clc; clear; close all;

sp = psoc_open("COM9", 115200);

psoc_resync_reset(sp);   % <-- CLAVE

coef = single(randn(12,1));
payload = typecast(coef, "uint8"); % 48 bytes

rsp = psoc_cmd(sp, 'c', 2.0);
fprintf("rsp c = %c\n", char(rsp));
assert(rsp == uint8('R'));

psoc_send_words4(sp, payload, 2.0);

final = psoc_readexact(sp, 1, 2.0); final = final(1);
fprintf("final c = %c\n", char(final));
assert(final == uint8('K'));

rsp = psoc_cmd(sp, 't', 2.0);
fprintf("rsp t = %c\n", char(rsp));
assert(rsp == uint8('S'));

rx = psoc_recv_words4(sp, numel(payload), 2.0);

final = psoc_readexact(sp, 1, 2.0); final = final(1);
assert(final == uint8('K'));

assert(isequal(rx(:), payload(:)));
disp("TODO OK");

clear sp;
