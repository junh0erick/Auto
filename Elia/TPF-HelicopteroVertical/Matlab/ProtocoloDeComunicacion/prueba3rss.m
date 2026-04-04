sp = serialport("COM19", 921600);
configureTerminator(sp,"LF");
flush(sp);
pause(0.05);

write(sp, uint8('m'), "uint8");
pause(0.2);

n = sp.NumBytesAvailable;
raw = uint8([]);
if n>0, raw = read(sp,n,"uint8"); end

disp("n=" + n);
disp(dec2hex(raw).');
disp(char(max(32, min(126, raw))).');

clear sp
