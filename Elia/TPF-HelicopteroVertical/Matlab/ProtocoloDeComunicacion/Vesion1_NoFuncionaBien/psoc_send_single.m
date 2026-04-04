function psoc_send_single(sp, cmd, v)
v = single(v(:).');
payload = typecast(v, 'uint8');
psoc_send(sp, cmd, payload);
end