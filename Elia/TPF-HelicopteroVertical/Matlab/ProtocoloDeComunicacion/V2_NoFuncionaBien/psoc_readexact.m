function data = psoc_readexact(sp, n, timeout_s)
t0 = tic;
data = uint8([]);
while numel(data) < n
    if toc(t0) > timeout_s
        error("Timeout leyendo %d bytes (tengo %d)", n, numel(data));
    end
    if sp.NumBytesAvailable > 0
        k = min(sp.NumBytesAvailable, n - numel(data));
        data = [data; read(sp, k, "uint8")]; %#ok<AGROW>
    else
        pause(0.001);
    end
end
end
