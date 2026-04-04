clear sp
sp = serialport("COM9", 115200, "Timeout", 0.2);
sp.FlowControl = "software";
flush(sp);
disp("OK abierto");
clear sp
