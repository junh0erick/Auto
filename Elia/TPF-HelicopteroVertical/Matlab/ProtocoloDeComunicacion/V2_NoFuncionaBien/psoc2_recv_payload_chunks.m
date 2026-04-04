function payload = psoc2_recv_payload_chunks(sp, nbytes, timeout_s)
payload = uint8([]);
seq = uint8(0);

while numel(payload) < nbytes
    % Buscar SOF 0x5A
    b = psoc2_readexact(sp, 1, timeout_s);
    while b(1) ~= hex2dec('5A')
        b = psoc2_readexact(sp, 1, timeout_s);
    end

    rest = psoc2_readexact(sp, 6, timeout_s); % seq + 4 data + crc
    pkt = uint8([b(1) rest(1:5)']);          % SOF..data
    crc_got = rest(6);
    crc_calc = psoc2_crc8(pkt);

    if crc_calc ~= crc_got || rest(1) ~= seq
        % NAK
        write(sp, uint8(hex2dec('15')), "uint8");
        continue;
    end

    % ACK
    write(sp, uint8(hex2dec('06')), "uint8");

    data4 = rest(2:5)';
    need = nbytes - numel(payload);
    payload = [payload; data4(1:min(4,need))]; %#ok<AGROW>

    seq = uint8(mod(double(seq)+1, 256));
end
end
