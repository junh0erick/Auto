function payload = uartp_ll_to_payload64(coeffs)
if isa(coeffs, "uint8")
    payload = uint8(coeffs(:));
    assert(numel(payload)==64, "uint8 payload debe ser 64 bytes");
    return;
end

c = single(coeffs(:));
assert(numel(c)==16, "coeffs debe tener 16 floats");
payload = typecast(c, "uint8");
payload = uint8(payload(:));
assert(numel(payload)==64);
end
