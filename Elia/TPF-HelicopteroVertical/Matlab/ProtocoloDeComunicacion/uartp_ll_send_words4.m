function uartp_ll_send_words4(sp, payload_u8, timeout_s)
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
        if tries > 200
            error("Demasiados reintentos enviando word en idx=%d", idx);
        end

        write(sp, w, "uint8");

        echo = zeros(4,1,'uint8');
        for i=1:4, echo(i)=uartp_ll_readexact(sp,1,timeout_s); end

        if isequal(echo(:), w(:))
            write(sp, A, "uint8");
        else
            write(sp, N, "uint8");
        end

        conf = uartp_ll_readexact(sp, 1, timeout_s);
        if conf == A && isequal(echo(:), w(:))
            break;
        end
    end

    idx = idx + take;
end
end
