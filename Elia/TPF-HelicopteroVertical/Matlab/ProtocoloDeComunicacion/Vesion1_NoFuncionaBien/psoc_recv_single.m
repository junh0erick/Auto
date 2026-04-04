function [cmd, v] = psoc_recv_single(sp, timeout_s)
[cmd, payload] = psoc_recv(sp, timeout_s);
if mod(numel(payload), 4) ~= 0
error("psoc_recv_single:len", "Payload no múltiplo de 4 (single)");
end
v = typecast(uint8(payload), 'single');
end