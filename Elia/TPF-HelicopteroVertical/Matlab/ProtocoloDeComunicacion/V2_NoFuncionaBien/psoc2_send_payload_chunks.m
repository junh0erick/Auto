function psoc2_send_payload_chunks(sp, payload, timeout_s)
payload = uint8(payload(:)');
seq = uint8(0);
idx = 1;

while idx <= numel(payload)
    data4 = uint8([0 0 0 0]);
    for i=1:4
        if idx <= numel(payload)
            data4(i) = payload(idx);
            idx = idx + 1;
        end
    end

    pkt = uint8([hex2dec('5A') seq data4]);
    crc = psoc2_crc8(pkt);
    frame = uint8([pkt crc]);

    retries = 0;
    while true
        write(sp, frame, "uint8");
        ack = psoc2_readexact(sp, 1, timeout_s);
        if ack(1) == hex2dec('06') % ACK
            break;
        end
        retries = retries + 1;
        if retries > 20
            error("Demasiados NAK/timeout en seq=%d", seq);
        end
    end

    seq = uint8(mod(double(seq)+1, 256));
end
end
