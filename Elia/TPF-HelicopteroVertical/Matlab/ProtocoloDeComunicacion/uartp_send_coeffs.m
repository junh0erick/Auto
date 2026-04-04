function uartp_send_coeffs(sp, coeffs, verify_roundtrip, timeout_s)
% uartp_send_coeffs  Envía coeficientes (N floats) por UARTP.
%
% coeffs:
%   - single(N,1) / double(N,1) -> se envían como 4*N bytes (float32)
%   - uint8(4*N,1)              -> se envían tal cual
%
% Protocolo:
%   host:  'c' -> espera 'R' -> manda payload -> espera 'K'
%   verify:'t' -> espera 'S' -> recibe payload -> espera 'K'

if nargin < 3 || isempty(verify_roundtrip), verify_roundtrip = true; end
if nargin < 4 || isempty(timeout_s), timeout_s = 2.0; end

% Convertir coeffs -> payload bytes (4*nWords)
[payload, nWords] = uartp_ll_to_payload_bytes(coeffs);

% Drenar basura previa (importante si venías con streaming)
uartp_ll_drain_rx(sp);

% Handshake 'c'
rsp = uartp_ll_cmd_wait(sp, 'c', timeout_s);
assert(rsp == uint8('R'), "c no devolvió R (rsp=%c)", char(rsp));

% Enviar payload
uartp_ll_send_words(sp, payload, timeout_s);

% Final
final = uartp_ll_readexact(sp, 1, timeout_s);
assert(final == uint8('K'), "c no terminó con K (rsp=%c)", char(final));

% Verify roundtrip
if verify_roundtrip
    rx = uartp_get_coeffs_raw(sp, timeout_s, nWords);  % uint8(4*nWords,1)
    assert(isequal(rx(:), payload(:)), "Roundtrip 't' devolvió bytes distintos");
end
end
