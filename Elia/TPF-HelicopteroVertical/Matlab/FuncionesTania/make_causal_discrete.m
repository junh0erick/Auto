function [sysc, d] = make_causal_discrete(sys)
% make_causal_discrete
% Si sys discreto SISO es impropio (no causal), lo vuelve propio
% multiplicando por z^-d (retardo puro de d muestras).

if sys.Ts <= 0
    error('El sistema debe ser discreto (Ts>0).');
end

% Pasar a TF para inspeccionar grados (SISO)
tfSys = tf(sys);

[num, den] = tfdata(tfSys, 'v');

% Eliminar ceros líderes por tolerancia numérica
tol = 1e-12;
num = trimLeadingZeros(num, tol);
den = trimLeadingZeros(den, tol);

degNum = length(num) - 1;
degDen = length(den) - 1;

d = max(0, degNum - degDen);  % si num > den, necesito delays

if d == 0
    sysc = sys;
    return;
end

% Multiplicar por z^-d  => en variable 'z' es 1/z^d => den *= z^d
% Implementación: tf(1, z^d) = tf(1, [1 0 0 ... 0], Ts, 'Variable','z')
zDelay = tf(1, [1 zeros(1,d)], sys.Ts, 'Variable','z');

sysc = minreal(tfSys * zDelay);
end

function v = trimLeadingZeros(v, tol)
% Elimina ceros iniciales por debajo de tolerancia
while length(v) > 1 && abs(v(1)) < tol
    v = v(2:end);
end
end
