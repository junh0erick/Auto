function uartp_ll_drain_rx(sp)
pause(0.02);
while sp.NumBytesAvailable > 0
    read(sp, sp.NumBytesAvailable, "uint8");
    pause(0.001);
end
end
