clc; clear; close all;

sp = psoc_open("COM9", 115200);
flush(sp); pause(0.05);
while sp.NumBytesAvailable > 0
    read(sp, sp.NumBytesAvailable, "uint8");
end

cmd = uint8('A');
v   = uint8(0:8);

psoc_send(sp, cmd, v);
[cmd2, v2] = psoc_recv(sp, 30.0);

disp(char(cmd2));
disp(v2);

assert(cmd2 == cmd);
assert(isequal(v2, v));

disp("OK echo");


clear sp;
