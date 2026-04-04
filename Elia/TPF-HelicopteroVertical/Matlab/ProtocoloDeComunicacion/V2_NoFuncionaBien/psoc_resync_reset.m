function psoc_resync_reset(sp)
flush(sp); pause(0.30);
while sp.NumBytesAvailable > 0
    read(sp, sp.NumBytesAvailable, "uint8");
end

for k = 1:10
    try
        rsp = psoc_cmd(sp, 'r', 0.5);
        if rsp == uint8('K')
            return;
        end
    catch
        % nada
    end
    pause(0.2);
end
error("No pude resync/resetear al PSoC");
end
