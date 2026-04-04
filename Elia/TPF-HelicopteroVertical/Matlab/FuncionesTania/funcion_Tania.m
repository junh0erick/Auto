% Supongamos que guardaste u1,u2 (esfuerzo) de cada simulación
% y K1,K2 de cada diseño (place)

u1 = u_case1(:);
u2 = u_case2(:);

metrics = @(u) struct( ...
    'umax', max(abs(u)), ...
    'urms', sqrt(mean(u.^2)), ...
    'uavg', mean(u), ...
    'du_rms', sqrt(mean(diff(u).^2)) ); % suavidad del control

M1 = metrics(u1);
M2 = metrics(u2);

fprintf('CASE 1: umax=%.4f | urms=%.4f | du_rms=%.4f\n', M1.umax, M1.urms, M1.du_rms);
fprintf('CASE 2: umax=%.4f | urms=%.4f | du_rms=%.4f\n', M2.umax, M2.urms, M2.du_rms);

fprintf('Reducción umax: %.1f %%\n', 100*(1 - M2.umax/M1.umax));
fprintf('Reducción urms: %.1f %%\n', 100*(1 - M2.urms/M1.urms));
