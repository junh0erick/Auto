%% PruebaUARTP_Random.m
clc; clear; close all;

PORT = "COM9";
BAUD = 115200;

N_ITERS = 50;     % subí a 200/1000 para stress test
MODE_LIST = 0:4;

% ========= abrir =========
sp = serialport(PORT, BAUD);
sp.DataBits = 8;
sp.StopBits = 1;
sp.Parity   = "none";
flush(sp); pause(0.20); drain(sp);

fprintf("Conectado a %s @ %d\n", PORT, BAUD);

% ========= reset =========
fprintf("Enviando reset...\n");
rsp = cmd_wait(sp, 'r', 1.0);
assert(rsp == uint8('K'), "Reset no devolvió K");
pause(0.40);
drain(sp);

% ========= loop modos =========
for mode = MODE_LIST
    fprintf("\n==== MODO %d ====\n", mode);

    % --- set mode 'm' ---
    rsp = cmd_wait(sp, 'm', 1.0);  assert(rsp == uint8('R'));
    send_words4(sp, uint8([mode 0 0 0]), 1.5);
    final = readexact(sp, 1, 1.0); assert(final == uint8('K'));

    for k = 1:N_ITERS
        % --- random coeffs ---
        % Usamos mezcla de rangos + casos "feos" cada tanto
        c = single(randn(16,1));             % N(0,1)
        c = c .* single(10.^(randi([-3 3],16,1))); % escala random 1e-3..1e3

        if mod(k,10)==0
            c(1) = single(0.0);
            c(2) = typecast(uint32(hex2dec('80000000')),'single'); % -0.0
            c(3) = single(inf);
            c(4) = single(-inf);
            c(5) = single(nan);
        end

        payload = typecast(c(:), "uint8");   % 64 bytes

        % --- c: enviar coef ---
        rsp = cmd_wait(sp, 'c', 1.0);
        assert(rsp == uint8('R'), "c no devolvió R (iter %d)", k);
        send_words4(sp, payload, 2.0);
        final = readexact(sp, 1, 1.0);
        assert(final == uint8('K'), "c no terminó con K (iter %d)", k);

        % --- t: recibir y verificar ---
        rsp = cmd_wait(sp, 't', 1.0);
        assert(rsp == uint8('S'), "t no devolvió S (iter %d)", k);

        rx = recv_words4(sp, numel(payload), 2.0);

        final = readexact(sp, 1, 1.0);
        assert(final == uint8('K'), "t no terminó con K (iter %d)", k);

        % 1) bytes idénticos
        assert(isequal(rx(:), payload(:)), "Mismatch BYTES (modo %d, iter %d)", mode, k);

        % 2) floats idénticos (bit-exacto)
        c_rx = typecast(uint8(rx(:)), "single");

        % NaN nunca es igual a NaN con isequal, pero bitwise sí.
        % Como ya chequeamos bytes, esto es redundante, pero lo dejamos “humano”.
        % Para evitar el tema NaN, comparamos por bytes ya arriba.
        % Igual mostramos si hay NaN:
        if any(isnan(c))
            % ok, ya validado por bytes
        else
            assert(isequal(c_rx(:), c(:)), "Mismatch FLOATS (modo %d, iter %d)", mode, k);
        end

        if mod(k,10)==0
            fprintf("  iter %d/%d OK\n", k, N_ITERS);
        end
    end

    fprintf("OK: modo %d pasó %d iteraciones random\n", mode, N_ITERS);

    % --- init -> control -> stop -> back to command (1 vez por modo) ---
    u0 = single(0.25 + 0.01*mode);
    rsp = cmd_wait(sp, 'i', 1.0);  assert(rsp == uint8('R'));
    send_words4(sp, typecast(u0,"uint8"), 1.5);
    final = readexact(sp, 1, 1.0); assert(final == uint8('K'));
    fprintf("OK: i (entra CONTROL)\n");

    % en CONTROL rechaza 'c'
    write(sp, uint8('c'), "uint8");
    b = readexact(sp, 1, 1.0);
    assert(b == uint8('!'), "En CONTROL, 'c' no fue rechazado");
    fprintf("OK: CONTROL rechaza 'c'\n");

    % stop
    write(sp, uint8('s'), "uint8");
    b = readexact(sp, 1, 1.0);
    assert(b == uint8('K'), "Stop no devolvió K");
    fprintf("OK: stop\n");

    ok_back = wait_back_to_command(sp, 3.0);
    assert(ok_back, "No volvió a COMMAND");
    fprintf("OK: volvió a COMMAND\n");

    drain(sp);
end

fprintf("\n✅ RANDOM TEST OK en modos %s con %d iteraciones cada uno\n", mat2str(MODE_LIST), N_ITERS);
clear sp;

%% ========================= funciones locales =========================

function drain(sp)
pause(0.02);
while sp.NumBytesAvailable > 0
    read(sp, sp.NumBytesAvailable, "uint8");
    pause(0.001);
end
end

function b = readexact(sp, n, timeout_s)
t0 = tic;
buf = zeros(1,n,'uint8'); k = 0;
while k < n
    if toc(t0) > timeout_s
        error("Timeout leyendo %d bytes (tengo %d)", n, k);
    end
    avail = sp.NumBytesAvailable;
    if avail > 0
        m = min(avail, n-k);
        tmp = read(sp, m, "uint8");
        buf(k+1:k+m) = tmp(:);
        k = k + m;
    else
        pause(0.001);
    end
end
b = buf;
end

function rsp = cmd_wait(sp, cmd_char, timeout_s)
valid = uint8(['R','S','K','!']);
write(sp, uint8(cmd_char), "uint8");
t0 = tic;
while true
    if toc(t0) > timeout_s
        error("Timeout esperando respuesta a '%s'", cmd_char);
    end
    if sp.NumBytesAvailable > 0
        b = read(sp, 1, "uint8"); b = b(1);
        if any(b == valid)
            rsp = b;
            return;
        end
    else
        pause(0.001);
    end
end
end

function send_words4(sp, payload_u8, timeout_s)
A = uint8('A'); N = uint8('N');
payload_u8 = uint8(payload_u8(:));
n = numel(payload_u8);
idx = 1;

while idx <= n
    w = uint8([0;0;0;0]);
    take = min(4, n-idx+1);
    w(1:take) = payload_u8(idx:idx+take-1);

    tries = 0;
    while true
        tries = tries + 1;
        if tries > 120
            error("Demasiados reintentos en idx=%d", idx);
        end

        write(sp, w, "uint8");

        echo = readexact(sp, 4, timeout_s);
        if isequal(echo(:), w(:)), write(sp, A, "uint8");
        else,                     write(sp, N, "uint8");
        end

        conf = readexact(sp, 1, timeout_s);
        if conf == A && isequal(echo(:), w(:))
            break;
        end
    end

    idx = idx + take;
end
end

function payload = recv_words4(sp, nbytes, timeout_s)
A = uint8('A'); N = uint8('N');
payload = zeros(nbytes,1,'uint8');
idx = 1;

while idx <= nbytes
    w = readexact(sp, 4, timeout_s);
    write(sp, w, "uint8");

    ctl = readexact(sp, 1, timeout_s);
    if ctl ~= A && ctl ~= N
        write(sp, N, "uint8");
        continue;
    end

    write(sp, ctl, "uint8");

    if ctl == A
        take = min(4, nbytes-idx+1);
        payload(idx:idx+take-1) = w(1:take);
        idx = idx + take;
    end
end
end

function ok = wait_back_to_command(sp, timeout_s)
t0 = tic; ok = false;
while toc(t0) < timeout_s
    try
        rsp = cmd_wait(sp, 't', 0.25);
        if rsp == uint8('S')
            % Consumimos payload de 64 y K para no desync
            rx = recv_words4(sp, 64, 1.0); %#ok<NASGU>
            k = readexact(sp, 1, 1.0);
            if k ~= uint8('K'), drain(sp); end
            ok = true;
            return;
        end
    catch
    end
    pause(0.05);
end
end
