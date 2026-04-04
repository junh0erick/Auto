function psoc_send_words4(sp, payload, timeout_s)
payload = uint8(payload(:)');
A = uint8('A'); N = uint8('N');

idx = 1;
while idx <= numel(payload)
    w = uint8([0 0 0 0]);
    take = min(4, numel(payload)-idx+1);
    w(1:take) = payload(idx:idx+take-1);

    while true
        % 1) mando 4
        write(sp, w, "uint8");

        % 2) espero eco 4
        echo = psoc_readexact(sp, 4, timeout_s);
        echo = echo(:)';

        if isequal(echo, w)
            % 3) mando ACK
            write(sp, A, "uint8");
        else
            % 3) mando NAK
            write(sp, N, "uint8");
        end

        % 4) espero confirmación del PSoC
        c = psoc_readexact(sp, 1, timeout_s);
        c = c(1);

        if c == A && isequal(echo, w)
            idx = idx + take;
            break;
        end
        % si c==N o no match -> reintento mismo word
    end
end
end
