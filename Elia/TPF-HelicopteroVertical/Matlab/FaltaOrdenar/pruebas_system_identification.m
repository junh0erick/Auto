load('psoc_stream_Pruebas_22-01-26.mat');   % trae n,u,y, meta, etc.

Fs = double(meta.Fs_ui);   % o usa tu Fs real guardada
Ts = 1/Fs;

u = u(:);
y = y(:);

% opcional: recortar un tramo útil (si al inicio hay basura)
% idx = 1000:60000; u=u(idx); y=y(idx);

% quitar offset (MUY recomendado si hay DC)
u0 = mean(u(1:min(2000,end)));
y0 = mean(y(1:min(2000,end)));
u1 = u - u0;
y1 = y - y0;

z = iddata(y1, u1, Ts);
ident
