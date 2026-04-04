function [cmd, payloadBytes] = psoc_recv(sp, timeout_s)
%PSOC_RECV (fail-fast) Recibe 1 frame: [SOF][CMD][LEN_L][LEN_H][PAYLOAD][CKSUM]
% Si checksum falla: error inmediato (para que el caller reintente).
% payloadBytes siempre sale 1×LEN (aunque LEN=0).

if nargin < 2
    timeout_s = 1.0;
end

SOF = uint8(hex2dec('7E'));
t0  = tic;

% 1) Buscar SOF
while true
    if toc(t0) > timeout_s
        error("psoc_recv:timeout", "Timeout esperando SOF");
    end

    if sp.NumBytesAvailable > 0
        b = read(sp, 1, "uint8");
        if b == SOF
            break;
        end
    else
        pause(0.001);
    end
end

% 2) Leer header
hdr = read_exact(sp, 3, timeout_s, t0);
cmd  = hdr(1);
lenL = hdr(2);
lenH = hdr(3);
len  = uint16(lenL) + bitshift(uint16(lenH), 8);

% 3) Leer payload y checksum
payloadBytes = zeros(1, double(len), "uint8");
if len > 0
    payloadBytes(:) = read_exact(sp, double(len), timeout_s, t0);
end
ck_got = read_exact(sp, 1, timeout_s, t0);
ck_got = ck_got(1);

% 4) Checksum XOR
ck = uint8(0);
ck = bitxor(ck, cmd);
ck = bitxor(ck, uint8(lenL));
ck = bitxor(ck, uint8(lenH));
for k = 1:double(len)
    ck = bitxor(ck, payloadBytes(k));
end

if ck ~= ck_got
    error("psoc_recv:checksum", "Checksum inválido (calc=%u got=%u)", ck, ck_got);
end

end

function data = read_exact(sp, n, timeout_s, t0)
data = zeros(1, n, "uint8");
idx  = 1;

while idx <= n
    if toc(t0) > timeout_s
        error("psoc_recv:timeout_read", "Timeout leyendo %d bytes", n);
    end

    avail = sp.NumBytesAvailable;
    if avail > 0
        k = min(avail, n - idx + 1);
        chunk = read(sp, k, "uint8");
        data(idx:idx+k-1) = chunk;
        idx = idx + k;
    else
        pause(0.001);
    end
end
end
