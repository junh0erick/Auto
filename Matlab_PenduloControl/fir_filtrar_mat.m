function fir_filtrar_mat(mat_path, Fc, ventana, orden)
% FIR_FILTRAR_MAT  Filtra la(s) señal(es) medida(s) de un .mat exportado
%                  por psoc_pendulo_gui.m o pendulo_gui2.m con un FIR de
%                  fase lineal, compensando el retardo de grupo.
%
% USO:
%   fir_filtrar_mat(mat_path, Fc, ventana)
%   fir_filtrar_mat(mat_path, Fc, ventana, orden)
%
% ARGUMENTOS:
%   mat_path : (string) ruta al .mat generado por la GUI
%   Fc       : (double) frecuencia de corte del FIR en Hz
%   ventana  : (string) tipo de ventana FIR:
%                'hamming'     (default si se omite)
%                'hanning'/'hann'
%                'blackman'
%                'rectangular'/'rectwin'
%                'bartlett'
%                'flattopwin'
%                'kaiser'      (usa beta = 6 por defecto)
%   orden    : (entero par, opcional, default=64) orden del filtro FIR.
%              El retardo de grupo compensado es orden/2 muestras.
%
% SALIDA:
%   Guarda un nuevo .mat con el sufijo _filtrado en el mismo directorio.
%   Las señales medidas filtradas reemplazan a las originales.
%   Se descartan las primeras orden/2 muestras de TODOS los vectores
%   del mismo eje temporal para mantener la coherencia.
%
% EJEMPLO:
%   fir_filtrar_mat('datos.mat', 5, 'hamming', 64)
%   fir_filtrar_mat('sesiones/prueba.mat', 10, 'blackman')

    % ── Argumentos por defecto ──────────────────────────────────────────
    if nargin < 4 || isempty(orden)
        orden = 64;
    end
    if nargin < 3 || isempty(ventana)
        ventana = 'hamming';
    end

    % orden debe ser par para FIR de fase lineal tipo I
    if mod(orden, 2) ~= 0
        orden = orden + 1;
        warning('FIR_FILTRAR_MAT: orden impar ajustado a %d (debe ser par).', orden);
    end

    delay = orden / 2;   % retardo de grupo en muestras = (N_taps - 1) / 2

    % ── Cargar .mat ─────────────────────────────────────────────────────
    if ~isfile(mat_path)
        error('FIR_FILTRAR_MAT: no se encontró el archivo "%s".', mat_path);
    end
    data = load(mat_path);

    % ── Detectar formato de GUI ──────────────────────────────────────────
    %   GUI v1 (psoc_pendulo_gui)  → campos: theta_real, t, Ts_inner
    %   GUI v2 (pendulo_gui2)      → campos: y1, y2, Fs_inner, Fs_outer
    if isfield(data, 'theta_real')
        formato = 'v1';
    elseif isfield(data, 'y1')
        formato = 'v2';
    else
        error(['FIR_FILTRAR_MAT: formato de .mat no reconocido. ' ...
               'Se esperaban los campos "theta_real" (GUI v1) o "y1" (GUI v2).']);
    end

    fprintf('Formato detectado: GUI %s\n', formato);
    fprintf('Ventana: %s | Orden: %d | Fc: %.4g Hz | Retardo: %d muestras\n', ...
            ventana, orden, Fc, delay);

    % ── Filtrar según formato ────────────────────────────────────────────
    switch formato

        % ────────────────────────────────────────────────────────────────
        case 'v1'   % psoc_pendulo_gui: theta_real, omega_real, t, u, ...
        % ────────────────────────────────────────────────────────────────
            Fs = obtener_Fs_v1(data);
            fprintf('Fs = %.4g Hz  (Ts_inner = %.4g ms)\n', Fs, 1000/Fs);

            h = disenar_fir(orden, Fc, Fs, ventana);

            % Señales a filtrar (misma frecuencia de muestreo)
            [data.theta_real, info] = aplicar_fir(data.theta_real, h, delay, 'theta_real');

            if isfield(data, 'omega_real') && ~isempty(data.omega_real)
                [data.omega_real, ~] = aplicar_fir(data.omega_real, h, delay, 'omega_real');
            end

            % Recortar vectores del mismo eje temporal (trim primero delay muestras)
            campos_t = {'t', 'u', 'u_unsat', 'theta_sim', 'omega_sim', ...
                        'omega_ref', 'x_hat'};
            data = recortar_campos(data, campos_t, delay);

            fprintf('%s\n', info);

        % ────────────────────────────────────────────────────────────────
        case 'v2'   % pendulo_gui2: y1 (Fs_inner), y2 (Fs_outer), n, u1...
        % ────────────────────────────────────────────────────────────────
            Fs_inner = obtener_campo_num(data, 'Fs_inner', 'Fs del lazo interno');
            Fs_outer = obtener_campo_num(data, 'Fs_outer', 'Fs del lazo externo');

            fprintf('Fs_inner = %.4g Hz | Fs_outer = %.4g Hz\n', Fs_inner, Fs_outer);

            % Lazo interno (y1, misma longitud que n, u1, u1_unsat, ysim1, x1i, x2i)
            h_inner = disenar_fir(orden, Fc, Fs_inner, ventana);
            [data.y1, info1] = aplicar_fir(data.y1, h_inner, delay, 'y1');
            fprintf('%s\n', info1);

            campos_inner = {'n', 'u1', 'u1_unsat', 'ysim1', 'x1i', 'x2i'};
            data = recortar_campos(data, campos_inner, delay);

            % Lazo externo (y2, misma longitud que u2, ysim2, x1o, x2o)
            h_outer = disenar_fir(orden, Fc, Fs_outer, ventana);
            [data.y2, info2] = aplicar_fir(data.y2, h_outer, delay, 'y2');
            fprintf('%s\n', info2);

            campos_outer = {'u2', 'ysim2', 'x1o', 'x2o'};
            data = recortar_campos(data, campos_outer, delay);
    end

    % ── Anotar metadatos del filtrado ────────────────────────────────────
    data.filtrado_fir = struct( ...
        'Fc',       Fc, ...
        'ventana',  ventana, ...
        'orden',    orden, ...
        'delay_compensado_muestras', delay, ...
        'fecha',    datestr(now, 'yyyy-mm-dd HH:MM:SS') ...
    );

    % ── Guardar con sufijo _filtrado ─────────────────────────────────────
    [dir_out, nombre, ext] = fileparts(mat_path);
    if isempty(dir_out), dir_out = '.'; end
    out_path = fullfile(dir_out, [nombre '_filtrado' ext]);

    save(out_path, '-struct', 'data');
    fprintf('Guardado: %s\n', out_path);
end

% =========================================================================
%  FUNCIONES AUXILIARES
% =========================================================================

function h = disenar_fir(orden, Fc, Fs, ventana)
% Diseña un FIR paso-bajo de fase lineal.
    Wn = Fc / (Fs / 2);   % frecuencia normalizada (0-1)
    if Wn <= 0 || Wn >= 1
        error(['FIR_FILTRAR_MAT: Fc=%.4g Hz no válida para Fs=%.4g Hz. ' ...
               'Debe estar en (0, Fs/2).'], Fc, Fs);
    end

    win_vec = obtener_ventana(ventana, orden + 1);
    h = fir1(orden, Wn, win_vec);
end

function win_vec = obtener_ventana(nombre, N)
% Devuelve el vector de ventana de longitud N.
    switch lower(strtrim(nombre))
        case 'hamming'
            win_vec = hamming(N);
        case {'hanning', 'hann'}
            win_vec = hanning(N);
        case 'blackman'
            win_vec = blackman(N);
        case {'rectangular', 'rectwin', 'rect'}
            win_vec = rectwin(N);
        case 'bartlett'
            win_vec = bartlett(N);
        case 'flattopwin'
            win_vec = flattopwin(N);
        case 'kaiser'
            win_vec = kaiser(N, 6);    % beta=6: compromiso selectividad/transición
        otherwise
            % Intentar llamar directamente como función de MATLAB
            try
                win_vec = feval(nombre, N);
            catch
                error('FIR_FILTRAR_MAT: ventana "%s" no reconocida.', nombre);
            end
    end
end

function [y_filt, info] = aplicar_fir(y, h, delay, nombre)
% Filtra y, descarta las primeras `delay` muestras para compensar retardo.
    if isempty(y)
        y_filt = y;
        info = sprintf('  [%s] vacío, omitido.', nombre);
        return;
    end

    es_col = iscolumn(y);
    y = y(:);                      % forzar columna para filter()

    y_filt_raw = filter(h, 1, y);  % respuesta causal (incluye transitorio)

    % Adelantar: descartar las primeras `delay` muestras
    y_filt = y_filt_raw(delay + 1 : end);

    if es_col
        y_filt = y_filt(:);
    else
        y_filt = y_filt(:).';
    end

    info = sprintf('  [%s] %d muestras → %d muestras (-%d por compensación FIR).', ...
                   nombre, numel(y), numel(y_filt), delay);
end

function data = recortar_campos(data, campos, delay)
% Recorta las primeras `delay` muestras de los campos indicados (si existen
% y tienen la longitud esperada). Solo actúa sobre vectores 1D.
    for k = 1:numel(campos)
        c = campos{k};
        if ~isfield(data, c), continue; end
        v = data.(c);
        if isempty(v), continue; end
        if isvector(v) && numel(v) > delay
            if iscolumn(v)
                data.(c) = v(delay + 1 : end);
            else
                data.(c) = v(delay + 1 : end);
            end
        end
        % Matrices (ej: x_hat): recortar filas o columnas según orientación
        if ~isvector(v) && ismatrix(v)
            [nr, nc] = size(v);
            if nr > nc && nr > delay        % filas = tiempo
                data.(c) = v(delay + 1 : end, :);
            elseif nc > nr && nc > delay    % columnas = tiempo
                data.(c) = v(:, delay + 1 : end);
            end
        end
    end
end

function Fs = obtener_Fs_v1(data)
% Extrae Fs del .mat de GUI v1 (psoc_pendulo_gui).
    if isfield(data, 'Ts_inner') && data.Ts_inner > 0
        Fs = 1 / data.Ts_inner;
    elseif isfield(data, 't') && numel(data.t) > 1
        dt_med = median(diff(data.t));
        if dt_med > 0
            Fs = 1 / dt_med;
            warning('FIR_FILTRAR_MAT: Ts_inner no encontrado; Fs estimada de t = %.4g Hz.', Fs);
        else
            error('FIR_FILTRAR_MAT: no se pudo determinar Fs del .mat (GUI v1).');
        end
    else
        error('FIR_FILTRAR_MAT: no se pudo determinar Fs del .mat (GUI v1).');
    end
end

function val = obtener_campo_num(data, campo, descripcion)
% Lee un campo numérico escalar del struct; lanza error si falta.
    if ~isfield(data, campo) || isempty(data.(campo))
        error('FIR_FILTRAR_MAT: campo "%s" (%s) no encontrado en el .mat.', ...
              campo, descripcion);
    end
    val = double(data.(campo));
end
