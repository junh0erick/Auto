clc; clear; close all;

sp = psoc_open("COM9", 115200);
flush(sp);           % limpia buffers del host
pause(0.05);         % deja que terminen de llegar bytes en vuelo
while sp.NumBytesAvailable > 0
    read(sp, sp.NumBytesAvailable, "uint8");  % descartar todo
    pause(0.01);
end


CMD_STATUS = uint8(255);
STATUS_BUSY = uint8(1);

% =========================
% TEST A: Stop-and-wait (mide RTT y throughput real)
% =========================
lengths = [0 4 16 64 128 256 400 512];  % bytes de payload
N = 50;                                  % repeticiones por tamaño

fprintf("=== TEST A: Stop-and-wait ===\n");

for L = lengths
    rtt = zeros(1, N);
    for k = 1:N
        payload = uint8(randi([0 255], 1, L));
        cmd = uint8('A');

        t0 = tic;
        psoc_send(sp, cmd, payload);
        [cmd2, payload2] = psoc_recv(sp, 5.0);
        rtt(k) = toc(t0);

        % Si recibimos status, tratamos como fallo (no debería en stop-and-wait)
        if cmd2 == CMD_STATUS
            error("Recibí STATUS en stop-and-wait: [%d %d]", payload2(1), payload2(2));
        end

        if cmd2 ~= cmd || ~isequal(payload2, payload)
            error("Mismatch en eco (L=%d, k=%d)", L, k);
        end
    end

    rtt_ms = 1000*rtt;
    mean_ms = mean(rtt_ms);
    p95_ms  = prctile(rtt_ms, 95);

    % Bytes por frame en el cable: SOF+CMD+LEN2+PAYLOAD+CKSUM = L+5
    bits_per_frame = (L + 5) * 8;
    eff_bps = bits_per_frame / mean(rtt);   % aprox stop-and-wait

    fprintf("L=%3d bytes | RTT mean=%.2f ms | p95=%.2f ms | eff=%.0f bps\n", ...
        L, mean_ms, p95_ms, eff_bps);
   
end

% =========================
% TEST B: Burst para provocar BUSY
% =========================
fprintf("\n=== TEST B: Burst (provocar BUSY) ===\n");

L = 256;
burstN = 100;

payloads = cell(1, burstN);
cmd = uint8('B');

% Mandar todo sin esperar (esto fuerza que el PSoC no pueda responder todo)
for k = 1:burstN
    payloads{k} = uint8(randi([0 255], 1, L));
    psoc_send(sp, cmd, payloads{k});
end

busy_cnt = 0;
ok_cnt = 0;

% Leer respuestas por un rato
t0 = tic;
while toc(t0) < 3.0
    if sp.NumBytesAvailable > 0
        [c, p] = psoc_recv(sp, 1.0);
        if c == CMD_STATUS
            if numel(p) >= 2 && p(1) == STATUS_BUSY
                busy_cnt = busy_cnt + 1;
            end
        else
            % eco normal
            ok_cnt = ok_cnt + 1;
        end
    else
        pause(0.001);
    end
end

fprintf("Respuestas eco OK: %d\n", ok_cnt);
fprintf("BUSY recibidos:     %d\n", busy_cnt);
fprintf("(En burst, es normal perder ecos si no hay cola; esto prueba el rechazo.)\n");

% =========================
% TEST C: Strings largos como payload
% =========================
msg = repmat('HolaPSoC_', 1, 200);   % <-- char vector 1xN
payload = uint8(msg);               % bytes ASCII

psoc_send(sp, 'S', payload);
[cmd2, payload2] = psoc_recv(sp, 2.0);

assert(cmd2 == uint8('S'));
assert(isequal(payload2, payload));
fprintf("OK string largo (len=%d bytes)\n", numel(payload));
