% motor_finetune.m  — v2
% Fine-tuning del modelo de motor DC contra datos medidos (barrido + steps).
%
% REQUIERE: System Identification Toolbox (tfest, iddata, compare)
%
% DATOS USADOS (en orden de prioridad):
%   1. barrido(U,Y,MODELO).mat       — datos dinámicos del barrido PSoC
%   2. pruebaStepsRapidos_filtrado.mat  — steps rápidos filtrados
%
% SALIDA: Motor_4.m — modelo fine-tuned listo para diseño de control
%
% NOTAS IMPORTANTES:
%   - Ts = 5 ms → solo el polo MECÁNICO (τ_m ≈ 80-300 ms) es identificable.
%     El polo eléctrico (τ_e = La/Ra ≈ 0.63 ms) queda fuera de Nyquist,
%     por eso el modelo 2do orden se construye analíticamente (no con tfest).
%   - La entrada u es [PWM counts]; K absorbe la conversión PWM→Voltaje.
%   - La salida y es ω [rad/s].

clear; close all;

ROOT = fileparts(mfilename('fullpath'));

% =========================================================================
%  1. PARÁMETROS FÍSICOS DE REFERENCIA (Motor_3)
% =========================================================================
Ra = 4.04;       % Ohm
La = 0.00255;    % H
Km = 0.1863;     % N·m/A
Bm = 1.391e-3;   % N·m·s/rad
Jm = 8.315e-4;   % kg·m²

tau_e  = La / Ra;                        % constante eléctrica [s]  ≈ 0.63 ms
denom  = Km^2 + Ra*Bm;
K_m3   = Km  / denom;                    % ganancia mecánica [rad/s/V]
tau_m3 = Ra*Jm / denom;                  % constante mecánica [s]

fprintf('=== Referencia Motor_3 ===\n');
fprintf('  tau_e (eléctrica) = %.2f ms  (polo a %.0f rad/s)\n', ...
        tau_e*1000, 1/tau_e);
fprintf('  tau_m (mecánica)  = %.2f ms\n', tau_m3*1000);
fprintf('  K_m3  [rad/s/V]   = %.5f\n\n', K_m3);

% =========================================================================
%  2. CARGAR DATOS DE IDENTIFICACIÓN
% =========================================================================

% Candidatos en orden de preferencia
CANDIDATOS = {
    fullfile(ROOT, 'barrido(U,Y,MODELO).mat'),  'barrido_uymod';
    fullfile(ROOT, 'barrido0-10.mat'),           'barrido_raw';
    fullfile(ROOT, 'Matlab_PenduloControl', 'pruebaStepsRapidos_filtrado.mat'), 'steps_v2';
    fullfile(ROOT, 'Matlab_PenduloControl', 'pruebaStepsRapidos.mat'),          'steps_v2_raw';
};

[u_raw, y_raw, Fs, src_label] = cargar_datos(CANDIDATOS);

Ts = 1 / Fs;
N  = numel(y_raw);
t_ax = (0:N-1).' * Ts;

fprintf('=== Datos cargados: "%s" ===\n', src_label);
fprintf('  Muestras: %d  |  Fs = %.4g Hz  |  Ts = %.2f ms\n', N, Fs, Ts*1000);
fprintf('  u: [%.1f, %.1f]   y: [%.3f, %.3f] rad/s\n\n', ...
        min(u_raw), max(u_raw), min(y_raw), max(y_raw));

% =========================================================================
%  3. INSPECCIÓN VISUAL DE LOS DATOS
% =========================================================================
figure('Name','Datos crudos','NumberTitle','off');
subplot(2,1,1);
stairs(t_ax, u_raw, 'b'); ylabel('u [PWM]'); xlabel('t [s]');
title(sprintf('Entrada — %s', src_label)); grid on;
subplot(2,1,2);
plot(t_ax, y_raw, 'r'); ylabel('\omega [rad/s]'); xlabel('t [s]');
title('Salida (velocidad angular)'); grid on;

% =========================================================================
%  4. PREPARAR DATOS PARA IDENTIFICACIÓN (iddata)
% =========================================================================
% Eliminar offset DC; el modelo captura la dinámica alrededor del OP.
u = detrend(u_raw, 'constant');
y = detrend(y_raw, 'constant');

% Separar 70% identificación / 30% validación
N_id  = round(0.7 * N);
data_id  = iddata(y(1:N_id),      u(1:N_id),      Ts, ...
                  'InputName', 'u [PWM]', 'OutputName', '\omega [rad/s]');
data_val = iddata(y(N_id+1:end),  u(N_id+1:end),  Ts, ...
                  'InputName', 'u [PWM]', 'OutputName', '\omega [rad/s]');

fprintf('  Train: %d muestras  |  Val: %d muestras\n\n', N_id, N-N_id);

% =========================================================================
%  5. IDENTIFICACIÓN 1er ORDEN  G(s) = K / (tau·s + 1)
% =========================================================================
% Estimación inicial de K [rad/s/PWM]:
%   Asumiendo PWM_max ≈ 1264 (limit en Motor_3.m) y V_max ≈ 12 V:
K_init_pwm   = K_m3 * 12 / 1264;    % ≈ 0.0438 rad/s/PWM
tau_init_pwm = tau_m3;               % la constante mecánica no cambia

fprintf('--- Identificación 1er orden (tfest) ---\n');
fprintf('  Punto de partida: K = %.5f  tau = %.1f ms\n', ...
        K_init_pwm, tau_init_pwm*1000);

sys1_init  = idtf(K_init_pwm, [tau_init_pwm, 1]);
opt        = tfestOptions('EnforceStability', true, 'Display', 'off');

sys1_tuned = tfest(data_id, sys1_init, opt);
sys1_free  = tfest(data_id, 1, opt);       % libre (sin estructura)

[n1t, d1t] = tfdata(sys1_tuned, 'v');
d1t_n      = d1t / d1t(end);
K_tuned    = n1t(end) / d1t(end);
tau_tuned  = d1t_n(1);

fprintf('  Motor_3  :  K = %.5f   tau = %.1f ms\n', K_init_pwm, tau_init_pwm*1000);
fprintf('  Tuned    :  K = %.5f   tau = %.1f ms   fit(val) = %.1f%%\n', ...
        K_tuned, tau_tuned*1000, fit_pct(sys1_tuned, data_val));
fprintf('  Libre    :  fit(val) = %.1f%%\n\n', fit_pct(sys1_free, data_val));

if tau_tuned < 0 || K_tuned < 0
    warning(['tfest convergió a valores negativos (K=%.4f, tau=%.1fms). ' ...
             'Usando el modelo libre en su lugar.'], K_tuned, tau_tuned*1000);
    [n1t, d1t] = tfdata(sys1_free, 'v');
    d1t_n   = d1t / d1t(end);
    K_tuned    = n1t(end) / d1t(end);
    tau_tuned  = d1t_n(1);
    sys1_tuned = sys1_free;
end

% =========================================================================
%  6. MODELO 2do ORDEN  — construcción analítica
%
%  Por qué NO se usa tfest aquí:
%    El polo eléctrico tau_e = La/Ra ≈ 0.63 ms es mucho más rápido que
%    Ts = 5 ms (Nyquist = 100 Hz = 628 rad/s, pero el polo está en 1584 rad/s).
%    tfest no puede resolverlo y converge a soluciones sin sentido físico
%    (ej: zeta > 100, wn >> 1000 rad/s). La solución correcta es añadir el
%    polo eléctrico conocido analíticamente sobre el modelo mecánico
%    identificado.
%
%  G2(s) = K_tuned / [(tau_tuned·s + 1)(tau_e·s + 1)]
%         = K_tuned / [tau_tuned·tau_e·s² + (tau_tuned+tau_e)·s + 1]
% =========================================================================
fprintf('--- Modelo 2do orden (analítico, polo eléctrico fijo) ---\n');

num2 = K_tuned;
den2 = [tau_tuned * tau_e,  tau_tuned + tau_e,  1];

wn_2   = sqrt(den2(3) / den2(1));
zeta_2 = den2(2) / (2 * sqrt(den2(1) * den2(3)));

fprintf('  G2(s) = %.5f / (%.4e s² + %.4f s + 1)\n', num2, den2(1), den2(2));
fprintf('  wn = %.1f rad/s  |  zeta = %.4f\n', wn_2, zeta_2);
fprintf('  polo lento: -%.1f rad/s (tau_m = %.1f ms)\n', 1/tau_tuned, tau_tuned*1000);
fprintf('  polo rápido: -%.0f rad/s (tau_e = %.2f ms)\n\n', 1/tau_e, tau_e*1000);

G1_ct = tf(K_tuned, [tau_tuned, 1]);
G2_ct = tf(num2, den2);

% =========================================================================
%  7. GRÁFICAS
% =========================================================================
% -- 7a. Ajuste en validación -----------------------------------------------
figure('Name','Comparación — Validación','NumberTitle','off');
compare(data_val, sys1_tuned, sys1_free);
legend({'Medido', '1er orden (tuned)', '1er orden (libre)'}, 'Location','best');
title(sprintf('Validación 1er orden  [%s]', src_label));
grid on;

% -- 7b. Respuesta en frecuencia -------------------------------------------
G_m3_ct = tf(K_m3, [tau_m3, 1]);   % Motor_3 en [rad/s/V] — solo para forma

figure('Name','Bode comparativo','NumberTitle','off');
bode(G1_ct, G2_ct);
legend({'G1 (1er orden, tuned)', 'G2 (2do orden, analítico)'}, 'Location','best');
title('Bode: Motor\_4 fine-tuned');
grid on;

% -- 7c. Fit tiempo-dominio ------------------------------------------------
y_sim1 = lsim(G1_ct, u, t_ax);
y_sim2 = lsim(G2_ct, u, t_ax);

figure('Name','Fit tiempo-dominio — datos completos','NumberTitle','off');
subplot(2,1,1);
plot(t_ax, y_raw, 'k', 'LineWidth', 1.0, 'DisplayName', 'Medido'); hold on;
plot(t_ax, y_sim1 + mean(y_raw), 'b--', 'LineWidth', 1.2, 'DisplayName', 'G1 tuned');
plot(t_ax, y_sim2 + mean(y_raw), 'r-.',  'LineWidth', 1.2, 'DisplayName', 'G2 analítico');
ylabel('\omega [rad/s]'); xlabel('t [s]');
title('Comparación tiempo-dominio'); legend('Location','best'); grid on; hold off;
subplot(2,1,2);
stairs(t_ax, u_raw, 'g', 'LineWidth', 1.2);
ylabel('u [PWM]'); xlabel('t [s]'); title('Entrada aplicada'); grid on;

% =========================================================================
%  8. DISCRETIZACIÓN
% =========================================================================
T = Ts;   % misma Ts que los datos

G1_d = c2d(G1_ct, T, 'zoh');
G2_d = c2d(G2_ct, T, 'zoh');

[num1d, den1d] = tfdata(G1_d, 'v');
[num2d, den2d] = tfdata(G2_d, 'v');

fprintf('=== Coeficientes discretos (ZOH, Ts=%.1f ms) ===\n', T*1000);
fprintf('  G1: b1 = %.6f   a1 = %.6f\n', num1d(2), den1d(2));
fprintf('  G2: b1 = %.6f  b2 = %.6f   a1 = %.6f  a2 = %.6f\n\n', ...
        num2d(2), num2d(3), den2d(2), den2d(3));

% =========================================================================
%  9. GENERAR Motor_4.m
% =========================================================================
out_file = fullfile(ROOT, 'Motor_4.m');
fid = fopen(out_file, 'w');
if fid < 0
    error('No se pudo escribir %s', out_file);
end

% Cabecera
fprintf(fid, '%% Motor_4.m — Modelo fine-tuned contra datos reales\n');
fprintf(fid, '%% Generado por motor_finetune.m v2\n');
fprintf(fid, '%% Fecha     : %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, '%% Fuente    : %s\n', src_label);
fprintf(fid, '%% Ts        : %.3f ms\n', T*1000);
fprintf(fid, '%%\n');
fprintf(fid, '%% UNIDADES:\n');
fprintf(fid, '%%   Entrada u : PWM counts (motor PSoC)\n');
fprintf(fid, '%%   Salida  y : omega [rad/s]\n');
fprintf(fid, '%%   K absorbe la conversión PWM → Voltaje\n');
fprintf(fid, '%%\n');
fprintf(fid, '%% NOTA 2do ORDEN:\n');
fprintf(fid, '%%   El polo eléctrico (tau_e=%.2fms) no es resoluble a Ts=%.1fms.\n', ...
        tau_e*1000, T*1000);
fprintf(fid, '%%   G2 se construye como G1 mas el polo electrico conocido (La/Ra).\n');
fprintf(fid, '\n');
fprintf(fid, 'clear all\nclose all\n\n');

% 1er orden
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, '%% MODELO 1er ORDEN   G1(s) = K / (tau*s + 1)\n');
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, 'K_1   = %.8f;   %% [rad/s / PWM]\n', K_tuned);
fprintf(fid, 'tau_1 = %.8f;   %% [s]  (%.2f ms)\n', tau_tuned, tau_tuned*1000);
fprintf(fid, '\nG1 = tf(K_1, [tau_1, 1]);\n\n');

% 2do orden
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, '%% MODELO 2do ORDEN   G2(s) = K / [(tau_m*s+1)(tau_e*s+1)]\n');
fprintf(fid, '%% tau_m = tau_1 identificado;  tau_e = La/Ra fijo\n');
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, 'tau_e = %.8f;   %% La/Ra = %.6f/%.2f  [s]  (%.3f ms)\n', ...
        tau_e, La, Ra, tau_e*1000);
fprintf(fid, 'K_2   = K_1;\n');
fprintf(fid, 'tau_m = tau_1;\n');
fprintf(fid, '\nG2 = tf(K_2, [tau_m*tau_e, tau_m+tau_e, 1]);\n');
fprintf(fid, '\n%% Forma expandida:\n');
fprintf(fid, 'b0_2 = %.8f;\n', num2(1));
fprintf(fid, 'a2_2 = %.10f;\n', den2(1));
fprintf(fid, 'a1_2 = %.8f;\n', den2(2));
fprintf(fid, 'a0_2 = 1;\n');
fprintf(fid, '\nwn_2   = %.4f;   %% rad/s\n', wn_2);
fprintf(fid, 'zeta_2 = %.4f;\n\n', zeta_2);

% Referencia Motor_3
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, '%% PARÁMETROS FÍSICOS (Motor_3) — referencia\n');
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, 'Ra = %.5f;  La = %.6f;  Km = %.5f;\n', Ra, La, Km);
fprintf(fid, 'Bm = %.4e;  Jm = %.4e;\n', Bm, Jm);
fprintf(fid, 'K_m3_volt  = %.8f;   %% [rad/s/V]  (referencia fisica)\n', K_m3);
fprintf(fid, 'tau_m3     = %.8f;   %% [s]\n\n', tau_m3);

% Discretización
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, '%% DISCRETIZACIÓN (ZOH, Ts = %.3f ms)\n', T*1000);
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, 'T = %.6f;\n\n', T);
fprintf(fid, 'G1_d = c2d(G1, T, ''zoh'');\n');
fprintf(fid, 'G2_d = c2d(G2, T, ''zoh'');\n\n');
fprintf(fid, '[num1d, den1d] = tfdata(G1_d, ''v'');\n');
fprintf(fid, '[num2d, den2d] = tfdata(G2_d, ''v'');\n\n');
fprintf(fid, '%% Coeficientes ecuación en diferencias:\n');
fprintf(fid, 'b1_1   = num1d(2);  a1_1  = den1d(2);           %% 1er orden\n');
fprintf(fid, 'b1_2   = num2d(2);  b2_2  = num2d(3);           %% 2do orden\n');
fprintf(fid, 'a1_2d  = den2d(2);  a2_2d = den2d(3);\n\n');
fprintf(fid, '%% Valores numéricos (solo referencia rápida):\n');
fprintf(fid, 'fprintf(''G1_d: b1=%%.6f  a1=%%.6f\\n'', num1d(2), den1d(2));\n');
fprintf(fid, 'fprintf(''G2_d: b1=%%.6f b2=%%.6f  a1=%%.6f a2=%%.6f\\n'', ...\n');
fprintf(fid, '        num2d(2), num2d(3), den2d(2), den2d(3));\n\n');

% Controlador PI (rediseño con G1_d)
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, '%% DISEÑO CONTROLADOR PI — REVISAR con los nuevos parámetros\n');
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, 'Kp = 0.5748;   %% REVISAR: diseniado con Motor_3\n');
fprintf(fid, 'Ti = 0.0695;\n');
fprintf(fid, 'C_s  = tf(Kp * [Ti 1], [Ti 0]);\n');
fprintf(fid, 'C_z1 = c2d(C_s, T, ''tustin'');\n\n');
fprintf(fid, 'cloop1 = feedback(C_z1 * G1_d, 1);\n');
fprintf(fid, 'cloop2 = feedback(C_z1 * G2_d, 1);\n\n');

% Gráficas en Motor_4.m
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, '%% GRÁFICAS\n');
fprintf(fid, '%% ===========================================================\n');
fprintf(fid, 'figure;\n');
fprintf(fid, 'step(cloop1, cloop2);\n');
fprintf(fid, 'legend(''Lazo cerrado 1er orden'', ''Lazo cerrado 2do orden'');\n');
fprintf(fid, 'title(''Respuesta al escalon - Motor\\_4 fine-tuned'');\n');
fprintf(fid, 'grid on;\n\n');
fprintf(fid, 'figure;\n');
fprintf(fid, 'bode(G1_d, G2_d);\n');
fprintf(fid, 'legend(''G1 disc'', ''G2 disc'');\n');
fprintf(fid, 'title(''Bode planta discreta - Motor\\_4'');\n');
fprintf(fid, 'grid on;\n\n');
fprintf(fid, 'figure;\n');
fprintf(fid, 'rlocus(C_z1 * G1_d);\n');
fprintf(fid, 'zgrid;\n');
fprintf(fid, 'title(''Lugar de raices - Motor\\_4 (1er orden)'');\n');

fclose(fid);
fprintf('\n=== Motor_4.m generado: %s ===\n', out_file);

% =========================================================================
%  10. RESUMEN FINAL
% =========================================================================
fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║           RESUMEN FINE-TUNING  (motor_finetune v2)      ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Fuente datos : %-38s ║\n', src_label);
fprintf('║  Muestras     : %d (Ts=%.1fms)%s║\n', N, T*1000, ...
        repmat(' ', 1, max(0, 31 - length(sprintf('%d (Ts=%.1fms)', N, T*1000)))));
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  1er ORDEN        Motor_3 [PWM]    Fine-tuned           ║\n');
fprintf('║  K   [rad/s/PWM]  %12.5f    %12.5f           ║\n', K_init_pwm, K_tuned);
fprintf('║  tau [ms]         %12.2f    %12.2f           ║\n', tau_init_pwm*1000, tau_tuned*1000);
fprintf('║  fit validación   %12s    %11.1f%%           ║\n', '---', fit_pct(sys1_tuned, data_val));
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  2do ORDEN (analítico: G1 + polo eléctrico tau_e)       ║\n');
fprintf('║  wn   [rad/s]     %12.1f    — sin sentido a Ts=5ms ║\n', wn_2);
fprintf('║  zeta             %12.4f                             ║\n', zeta_2);
fprintf('║  Polo lento  -%.0f rad/s (tau_m tuned = %.1f ms)%s║\n', ...
        1/tau_tuned, tau_tuned*1000, ...
        repmat(' ', 1, max(0, 11 - length(sprintf('%.0f', 1/tau_tuned)))));
fprintf('║  Polo rápido -%.0f rad/s (tau_e = La/Ra = %.2f ms)  ║\n', 1/tau_e, tau_e*1000);
fprintf('╚══════════════════════════════════════════════════════════╝\n');

% =========================================================================
%  FUNCIONES LOCALES
% =========================================================================

function [u_out, y_out, Fs_out, label_out] = cargar_datos(candidatos)
% Intenta cargar datos de identificación de varios archivos candidatos.
% Retorna el primero que tenga campos U e Y válidos.

    for k = 1:size(candidatos, 1)
        fpath = candidatos{k, 1};
        tag   = candidatos{k, 2};

        if ~isfile(fpath), continue; end

        d = load(fpath);
        fprintf('[cargar_datos] Probando: %s\n', fpath);

        % --- Detectar campos según convención de nomenclatura ---
        [u_c, y_c, Fs_c, ok] = detectar_campos(d, tag);

        if ok
            u_out     = u_c(:);
            y_out     = y_c(:);
            Fs_out    = Fs_c;
            label_out = tag;
            return;
        end
    end

    error(['No se encontró ningún archivo de datos válido.\n' ...
           'Ejecuta primero la GUI (pendulo_gui2) y guarda una sesión,\n' ...
           'o genera pruebaStepsRapidos_filtrado.mat con fir_filtrar_mat.']);
end

function [u, y, Fs, ok] = detectar_campos(d, tag)
% Detecta los campos de entrada/salida y frecuencia de muestreo según el
% formato del archivo.
    u = []; y = []; Fs = 0; ok = false;

    % ── Formato barrido(U,Y,MODELO) ──────────────────────────────────────
    if contains(tag, 'barrido_uymod')
        % Posibles nombres de campo (case-insensitive search)
        fnames = fieldnames(d);
        u_name = buscar_campo(fnames, {'U','u','u1','pwm','input'});
        y_name = buscar_campo(fnames, {'Y','y','y1','omega','output','speed'});
        Fs_name = buscar_campo(fnames, {'Fs','fs','Fs_inner','Fs1','fs_inner'});
        Ts_name = buscar_campo(fnames, {'Ts','ts','Ts_inner','ts_inner','dt'});

        if isempty(u_name) || isempty(y_name), return; end

        u = double(d.(u_name));
        y = double(d.(y_name));

        if ~isempty(Fs_name)
            Fs = double(d.(Fs_name));
        elseif ~isempty(Ts_name)
            Fs = 1 / double(d.(Ts_name));
        else
            % Estimar Fs a partir del tamaño asumiendo experimento típico
            warning('cargar_datos: Fs no encontrado en barrido_uymod; asumiendo 200 Hz');
            Fs = 200;
        end

        ok = numel(u) > 10 && numel(u) == numel(y) && Fs > 0;
        return;
    end

    % ── Formato barrido_raw ───────────────────────────────────────────────
    if contains(tag, 'barrido_raw')
        fnames = fieldnames(d);
        u_name = buscar_campo(fnames, {'u','U','u1','pwm'});
        y_name = buscar_campo(fnames, {'y','Y','y1','omega','rpm'});
        Fs_name = buscar_campo(fnames, {'Fs','fs','Fs_inner','Fs1'});
        Ts_name = buscar_campo(fnames, {'Ts','ts','Ts_inner','dt'});

        if isempty(u_name) || isempty(y_name), return; end

        u = double(d.(u_name));
        y = double(d.(y_name));

        if ~isempty(Fs_name)
            Fs = double(d.(Fs_name));
        elseif ~isempty(Ts_name)
            Fs = 1 / double(d.(Ts_name));
        else
            warning('cargar_datos: Fs no encontrado en barrido_raw; asumiendo 200 Hz');
            Fs = 200;
        end

        % Si y está en RPM, convertir a rad/s
        if max(abs(y)) > 500
            fprintf('  [aviso] y parece estar en RPM (max=%.1f); convirtiendo a rad/s.\n', max(abs(y)));
            y = y * (2*pi/60);
        end

        ok = numel(u) > 10 && numel(u) == numel(y) && Fs > 0;
        return;
    end

    % ── Formato GUI v2 (pendulo_gui2): y1, u1, Fs_inner ─────────────────
    if contains(tag, 'steps_v2')
        if ~isfield(d, 'y1') || ~isfield(d, 'u1'), return; end

        y = double(d.y1);
        u = double(d.u1);

        if isfield(d, 'Fs_inner')
            Fs = double(d.Fs_inner);
        elseif isfield(d, 'Ts_inner') && d.Ts_inner > 0
            Fs = 1 / double(d.Ts_inner);
        else
            Fs = 200;
            warning('cargar_datos: Fs no encontrado en steps_v2; asumiendo 200 Hz');
        end

        ok = numel(y) > 10 && numel(y) == numel(u) && Fs > 0;
        return;
    end
end

function nombre = buscar_campo(fnames, opciones)
% Busca el primer campo disponible de una lista de opciones (exacto luego case-insensitive).
    nombre = '';
    fnames_low = lower(fnames);
    for k = 1:numel(opciones)
        % Búsqueda exacta primero
        idx = find(strcmp(fnames, opciones{k}), 1);
        if ~isempty(idx), nombre = fnames{idx}; return; end
        % Búsqueda case-insensitive
        idx = find(strcmp(fnames_low, lower(opciones{k})), 1);
        if ~isempty(idx), nombre = fnames{idx}; return; end
    end
end

function p = fit_pct(sys, data)
% Extrae el porcentaje de ajuste de un modelo identificado contra data.
    try
        [~, fit_vec] = compare(data, sys);
        p = fit_vec;
    catch
        p = nan;
    end
end
