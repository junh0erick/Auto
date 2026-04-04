function uart_terminal_echo()
clc; clear; close all;

sp = psoc_open("COM9", 921600);

% Purga real
flush(sp);
pause(0.05);
while sp.NumBytesAvailable > 0
    read(sp, sp.NumBytesAvailable, "uint8");
    pause(0.01);
end

fprintf("\nTerminal UART (eco)\n");
fprintf("Formato: CMD, payload\n");
fprintf("Ejemplos:\n");
fprintf("  A, hola mundo\n");
fprintf("  S, 1.5 -2 3.25   (interpreta como singles si CMD= 'S')\n");
fprintf("  X,              (payload vacío)\n");
fprintf("Comandos especiales:\n");
fprintf("  quit\n\n");

CMD_STATUS  = uint8(255);
STATUS_BUSY = uint8(1);

while true
    line = input("> ", "s");
    if isempty(line)
        continue;
    end
    if strcmpi(strtrim(line), "quit")
        break;
    end

    % Parse: "CMD, payload"
    parts = strsplit(line, ",");
    cmd_str = strtrim(parts{1});
    if isempty(cmd_str)
        fprintf("CMD vacío\n");
        continue;
    end
    cmd_char = cmd_str(1);
    cmd = uint8(cmd_char);

    payload_raw = "";
    if numel(parts) >= 2
        payload_raw = strtrim(join(parts(2:end), ",")); % por si hay comas adentro
    end

    % Construir payload según CMD:
    % - si cmd=='S': intenta parsear números y mandar singles
    % - si no: manda ASCII tal cual
    if cmd_char == 'S'
        nums = str2num(payload_raw); %#ok<ST2NM>
        if isempty(nums) && ~isempty(payload_raw)
            fprintf("CMD='S' pero no pude parsear números. Mandando ASCII.\n");
            payload = uint8(char(payload_raw));
            payload_type = "ASCII";
        else
            v = single(nums);
            payload = typecast(v(:).', 'uint8');
            payload_type = sprintf("single[%d]", numel(v));
        end
    else
        payload = uint8(char(payload_raw));
        payload_type = "ASCII";
    end

    % Metadatos TX
    L = numel(payload);           % bytes de payload
    frame_bytes = L + 5;          % bytes totales en el cable (SOF+CMD+LEN2+PAYLOAD+CKSUM)

    % Medición de RTT
    t0 = tic;
    psoc_send(sp, cmd, payload);
    [cmd2, payload2] = psoc_recv(sp, 2.0);
    rtt = toc(t0);

    % Chequeos y reporte
    fprintf("\n--- TX ---\n");
    fprintf("CMD_TX      : '%s' (0x%02X)\n", char(cmd), cmd);
    fprintf("Payload type: %s\n", payload_type);
    fprintf("L (payload) : %d bytes\n", L);
    fprintf("Frame bytes : %d bytes (L+5)\n", frame_bytes);
    fprintf("Baud config : %d\n", baud);

    fprintf("\n--- RX ---\n");
    fprintf("CMD_RX      : '%s' (0x%02X)\n", char(cmd2), cmd2);
    fprintf("LEN_RX      : %d bytes\n", numel(payload2));

    if cmd2 == CMD_STATUS && numel(payload2) >= 2 && payload2(1) == STATUS_BUSY
        fprintf("STATUS      : BUSY (rechazó cmd=0x%02X '%s')\n", payload2(2), char(payload2(2)));
    else
        same = (cmd2 == cmd) && isequal(payload2, payload);
        fprintf("Echo match  : %s\n", string(same));
    end

    fprintf("\n--- Timing ---\n");
    fprintf("RTT         : %.3f ms\n", 1000*rtt);

    % Métricas útiles (aprox):
    % Asumiendo 8N1 => 10 bits/byte en la línea
    bits_on_wire_roundtrip = 2 * frame_bytes * 10;
    baud_est = bits_on_wire_roundtrip / rtt;     % estimación “bruta” de velocidad efectiva ida/vuelta
    payload_eff_bps = (L * 8) / rtt;             % payload bits/s “por RTT” (stop-and-wait)

    fprintf("baud_est     : %.0f bps (aprox, ida+vuelta)\n", baud_est);
    fprintf("payload_eff  : %.0f bps (payload/RTT)\n", payload_eff_bps);

    % Mostrar payload recibido según tipo
    if cmd_char == 'S' && mod(numel(payload2),4)==0 && ~(cmd2==CMD_STATUS)
        v2 = typecast(uint8(payload2), 'single');
        fprintf("\nPayload RX como single:\n");
        disp(v2);
    else
        if numel(payload2) <= 200
            fprintf("\nPayload RX como ASCII:\n");
            disp(char(payload2));
        else
            fprintf("\nPayload RX: muy largo para imprimir como ASCII (%d bytes)\n", numel(payload2));
            fprintf("Primeros 32 bytes HEX:\n");
            disp(dec2hex(payload2(1:32))');
        end
    end

    fprintf("\n============================\n\n");
end

clear sp;
fprintf("Cerrado.\n");
end
