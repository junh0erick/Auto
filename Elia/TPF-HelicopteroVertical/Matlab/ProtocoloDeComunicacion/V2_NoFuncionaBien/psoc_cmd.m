function rsp = psoc_cmd(sp, cmd, timeout_s)
write(sp, uint8(cmd), "uint8");

valid = uint8(['R' 'S' 'K' '!']); % READY_RX, READY_TX, OK, ERR
t0 = tic;

while true
    if toc(t0) > timeout_s
        error("Timeout esperando respuesta a '%s'", char(cmd));
    end
    if sp.NumBytesAvailable > 0
        b = read(sp, 1, "uint8"); b = b(1);
        if any(b == valid)
            rsp = b; return;
        end
        % basura -> descartar
    else
        pause(0.001);
    end
end
end
