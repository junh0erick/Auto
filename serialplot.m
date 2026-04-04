function serialplot()

    %% --- Puerto serie ---
    COM  = 'COM4';
    BAUD = 115200;
    TS   = 0.005;      % tiempo de muestreo [s]

    s = serialport(COM, BAUD);
    configureTerminator(s, "CR/LF");
    flush(s);

    %% --- Figura (figure clásico, compatible con callbacks) ---
    fig = figure('Name', 'PSoC Motor PI', 'Position', [100 100 820 500], ...
                 'CloseRequestFcn', @onClose);

    ax = axes(fig, 'Position', [0.30 0.10 0.67 0.83]);
    xlabel(ax, 'Tiempo [s]');
    ylabel(ax, 'Esfuerzo u [V]');
    title(ax, 'Salida PI — Motor');
    grid(ax, 'on');
    hLine = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.2);

    %% --- Controles ---
    uicontrol(fig, 'Style', 'text', 'String', 'Setpoint [rad/s]:', ...
              'Position', [10 450 170 20], 'HorizontalAlignment', 'left');
    editSP = uicontrol(fig, 'Style', 'edit', 'String', '0', ...
                       'Position', [10 425 170 25]);

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'Enviar SP + Graficar', ...
              'Position', [10 378 170 36], ...
              'BackgroundColor', [0.2 0.6 0.2], 'ForegroundColor', 'w', ...
              'Callback', @enviarYGraficar);

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'PARAR MOTOR', ...
              'Position', [10 330 170 36], ...
              'BackgroundColor', [0.85 0.15 0.15], 'ForegroundColor', 'w', ...
              'FontWeight', 'bold', 'Callback', @pararMotor);

    uicontrol(fig, 'Style', 'text', 'String', 'Muestras a capturar:', ...
              'Position', [10 295 170 20], 'HorizontalAlignment', 'left');
    editN = uicontrol(fig, 'Style', 'edit', 'String', '500', ...
                      'Position', [10 270 170 25]);

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'Detener captura', ...
              'Position', [10 225 170 30], 'Callback', @detenerCaptura);

    lblEstado = uicontrol(fig, 'Style', 'text', 'String', 'Listo', ...
                          'Position', [10 190 170 22], ...
                          'ForegroundColor', [0 0.6 0], ...
                          'HorizontalAlignment', 'left');

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'Guardar datos', ...
              'Position', [10 145 170 30], 'Callback', @guardarDatos);

    %% --- Estado interno ---
    datos      = [];
    idx        = 0;
    capturando = false;
    nMuestras  = 500;

    %% --- Callbacks ---

    function enviarYGraficar(~, ~)
        sp = str2double(get(editSP, 'String'));
        if isnan(sp), sp = 0; end
        write(s, uint8(sprintf('%d\r', round(sp * 10))), 'uint8');
        iniciarCaptura();
        set(lblEstado, 'String', sprintf('SP=%.1f rad/s — capturando...', sp), ...
            'ForegroundColor', [0.8 0.5 0]);
    end

    function pararMotor(~, ~)
        stopCaptura();
        write(s, uint8(sprintf('0\r')), 'uint8');
        set(lblEstado, 'String', 'Motor detenido', ...
            'ForegroundColor', [0.8 0.15 0.15]);
    end

    function iniciarCaptura(~, ~)
        if capturando, return; end
        n = round(str2double(get(editN, 'String')));
        if isnan(n) || n < 10, n = 500; end
        nMuestras  = n;
        datos      = nan(1, nMuestras);
        idx        = 0;
        capturando = true;
        flush(s);
        configureCallback(s, "terminator", @leerLinea);
    end

    function leerLinea(~, ~)
        if ~capturando || ~isvalid(fig)
            stopCaptura();
            return;
        end
        try
            linea = readline(s);
            val   = str2double(strtrim(linea)) / 100;
            if ~isnan(val) && idx < nMuestras
                idx = idx + 1;
                datos(idx) = val;
                set(hLine, 'XData', (0:idx-1)*TS, 'YData', datos(1:idx));
                drawnow limitrate;
            end
        catch e
            disp(['serialplot callback error: ' e.message]);
        end
        if idx >= nMuestras
            stopCaptura();
            set(lblEstado, 'String', sprintf('Listo — %d muestras', idx), ...
                'ForegroundColor', [0 0.6 0]);
        end
    end

    function detenerCaptura(~, ~)
        stopCaptura();
        set(lblEstado, 'String', sprintf('Detenido en %d muestras', idx), ...
            'ForegroundColor', [0 0.6 0]);
    end

    function stopCaptura()
        capturando = false;
        try, configureCallback(s, "off"); catch, end
    end

    function guardarDatos(~, ~)
        if idx == 0
            msgbox('No hay datos para guardar.', 'Sin datos');
            return;
        end
        u_out = datos(1:idx);
        t_out = (0:idx-1) * TS;
        [archivo, ruta] = uiputfile('*.mat', 'Guardar datos');
        if archivo ~= 0
            save(fullfile(ruta, archivo), 'u_out', 't_out');
            set(lblEstado, 'String', 'Datos guardados.');
        end
    end

    function onClose(~, ~)
        stopCaptura();
        delete(s);
        delete(fig);
    end

end
