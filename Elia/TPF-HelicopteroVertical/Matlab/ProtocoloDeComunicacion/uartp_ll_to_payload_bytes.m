function [payload, nWords] = uartp_ll_to_payload_bytes(coeffs)
% Devuelve payload uint8(4*N,1) + N words.

if isa(coeffs,'uint8')
    payload = coeffs(:);
    assert(mod(numel(payload),4)==0, "payload uint8 debe ser múltiplo de 4 bytes");
    nWords = numel(payload)/4;
    return;
end

c = single(coeffs(:));
nWords = numel(c);
payload = typecast(c, 'uint8');
payload = payload(:);
end
