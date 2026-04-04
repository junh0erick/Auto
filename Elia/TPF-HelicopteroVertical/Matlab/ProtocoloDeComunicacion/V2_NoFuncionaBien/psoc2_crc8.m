function crc = psoc2_crc8(bytes)
% CRC-8 poly 0x07, init 0x00
crc = uint8(0);
for b = uint8(bytes(:))'
    crc = bitxor(crc, b);
    for i=1:8
        if bitand(crc, uint8(128)) ~= 0
            crc = bitxor(bitshift(crc, 1), uint8(7));
        else
            crc = bitshift(crc, 1);
        end
        crc = uint8(bitand(crc, 255));
    end
end
end
