function rsp = uartp_ll_cmd_wait(sp, cmd_char, timeout_s)
if nargin < 3 || isempty(timeout_s), timeout_s = 2.0; end

cmd_char = char(cmd_char);

% drenar antes de mandar comando
uartp_ll_drain_rx(sp);

write(sp, uint8(cmd_char), "uint8");

switch cmd_char
    case 'c'
        expected = uint8('R');
    case 't'
        expected = uint8('S');
    otherwise
        expected = uint8(['R','S','!']);
end

t0 = tic;
while true
    if toc(t0) > timeout_s
        error("Timeout esperando respuesta a '%s'", cmd_char);
    end

    if sp.NumBytesAvailable > 0
        b = read(sp, 1, "uint8"); b = b(1);

        if b == expected
            rsp = b;
            return;
        end

        if b == uint8('!')
            rsp = b; % error del firmware
            return;
        end

        % si no es lo esperado, ignoramos (basura/stream)
    else
        pause(0.001);
    end
end
end
