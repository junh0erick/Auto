function payload = psoc_recv_words4(sp, nbytes, timeout_s)
payload = uint8([]);
A = uint8('A'); N = uint8('N');

while numel(payload) < nbytes
    % 1) leo 4 del psoc
    w = psoc_readexact(sp, 4, timeout_s);
    w = w(:)';

    % 2) eco 4
    write(sp, w, "uint8");

    % 3) leo decisión del PSoC
    d = psoc_readexact(sp, 1, timeout_s);
    d = d(1);

    % 4) confirmo con el mismo byte
    write(sp, d, "uint8");

    if d == A
        need = nbytes - numel(payload);
        payload = [payload; w(1:min(4,need)).']; %#ok<AGROW>
    elseif d == N
        % psoc va a reenviar el mismo word
    else
        error("Decisión inválida 0x%02X", d);
    end
end
end
