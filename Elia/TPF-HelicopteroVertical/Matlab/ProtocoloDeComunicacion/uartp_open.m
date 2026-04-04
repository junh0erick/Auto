function sp = uartp_open(port, baud)
sp = serialport(port, baud);
sp.DataBits = 8;
sp.StopBits = 1;
sp.Parity   = "none";
flush(sp);
pause(0.20);
uartp_ll_drain(sp);
end
