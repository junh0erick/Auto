function payload = uartp_ll_recv_words4(sp, nBytes, timeout_s)
% Recibe nBytes con el mismo esquema de echo/A/N por word de 4 bytes.
if nargin < 3 || isempty(timeout_s), timeout_s = 2.0; end
assert(mod(nBytes,4)==0, "nBytes debe ser múltiplo de 4");

A = uint8('A'); 
N = uint8('N');

payload = zeros(nBytes,1,'uint8');
idx = 1;

while idx <= nBytes
    w = uartp_ll_readexact(sp, 4, timeout_s);   % el device manda 4 bytes

    % devolvemos echo
    write(sp, w, "uint8");

    ack = uartp_ll_readexact(sp, 1, timeout_s); % host manda A/N? (depende del rol)
    % OJO: si tu protocolo es al revés acá (device espera A/N del host),
    % entonces no debe leerse ack, debe enviarse.

    % ---- IMPORTANTE ----
    % Como no tengo tu firmware, te doy la versión segura para el caso
    % "device manda word, host eco, host manda A/N, device confirma A".
    % Si tu firmware hace otra cosa, ajustamos.

    if ack == A
        payload(idx:idx+3) = w;
        idx = idx + 4;
    else
        % si N, reintenta leyendo el mismo word otra vez
        % (en la práctica, conviene implementar retries con timeout)
    end
end
end
