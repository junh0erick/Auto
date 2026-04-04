function sp = psoc_open(port, baud)
arguments
port (1,1) string
baud (1,1) double = 115200
end
sp = serialport(port, baud);
sp.DataBits = 8;
sp.Parity = "none";
sp.StopBits = 1;
sp.Timeout = 1; % s
flush(sp);
end