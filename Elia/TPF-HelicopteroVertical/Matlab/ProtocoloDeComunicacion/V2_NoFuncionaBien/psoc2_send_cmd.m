function rsp = psoc2_send_cmd(sp, cmd, timeout_s)
write(sp, uint8(cmd), "uint8");
rsp = psoc2_readexact(sp, 1, timeout_s);
rsp = rsp(1);
end
