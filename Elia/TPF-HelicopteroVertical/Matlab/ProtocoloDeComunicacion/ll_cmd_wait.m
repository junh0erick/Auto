function rsp = ll_cmd_wait(cmd_char, timeout_s)
    if nargin<2, timeout_s = STEP_TIMEOUT_S; end
    valid = uint8(['R','S','K','!']);

    write(S.sp, uint8(cmd_char), "uint8");

    % Esperar primer byte válido sin comérselo antes
    t0 = tic;
    while true
        if toc(t0) > timeout_s
            error("Timeout esperando respuesta a '%s'", char(cmd_char));
        end
        if S.sp.NumBytesAvailable > 0
            bb = read(S.sp, 1, "uint8"); bb = bb(1);

            % DEBUG opcional (sin romper)
            % disp("RX1: " + char(bb) + " HEX=" + dec2hex(bb));

            if any(bb == valid)
                rsp = bb;
                return;
            end
        else
            pause(0.001);
        end
    end
end
