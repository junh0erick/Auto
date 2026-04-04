function ok = uartp_ll_wait_back_to_command(sp, timeout_s)
t0 = tic;
ok = false;

while toc(t0) < timeout_s
    try
        rsp = uartp_ll_cmd_wait(sp, 't', 0.25);
        if rsp == uint8('S')
            % consumimos payload 64 + K para no desync
            rx = uartp_ll_recv_words4(sp, 64, 1.0); %#ok<NASGU>
            k  = uartp_ll_readexact(sp, 1, 1.0);
            if k ~= uint8('K'), uartp_ll_drain(sp); end
            ok = true;
            return;
        end
    catch
    end
    pause(0.05);
end
end
