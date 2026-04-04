function buf = uartp_ll_readexact(sp, n, timeout_s)
if nargin < 3 || isempty(timeout_s), timeout_s = 2.0; end

t0 = tic;
buf = zeros(n,1,'uint8');
k = 0;

while k < n
    if toc(t0) > timeout_s
        error("Timeout leyendo %d bytes (tengo %d)", n, k);
    end

    avail = sp.NumBytesAvailable;
    if avail > 0
        m = min(avail, n-k);
        tmp = read(sp, m, "uint8");
        buf(k+1:k+m) = tmp(:);
        k = k + m;
    else
        pause(0.001);
    end
end
end
