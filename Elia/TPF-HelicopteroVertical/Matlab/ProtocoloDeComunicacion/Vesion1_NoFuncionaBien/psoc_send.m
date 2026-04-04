function psoc_send(sp, cmd, payloadBytes)
SOF = uint8(hex2dec('7E'));


if ischar(cmd) || isstring(cmd)
cmd = uint8(char(cmd));
else
cmd = uint8(cmd);
end


payloadBytes = uint8(payloadBytes(:).');
len = uint16(numel(payloadBytes));
lenL = uint8(bitand(len, 255));
lenH = uint8(bitshift(len, -8));


ck = uint8(0);
ck = bitxor(ck, cmd);
ck = bitxor(ck, lenL);
ck = bitxor(ck, lenH);
for k = 1:numel(payloadBytes)
ck = bitxor(ck, payloadBytes(k));
end


frame = [SOF, cmd, lenL, lenH, payloadBytes, ck];
write(sp, frame, "uint8");
end