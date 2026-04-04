%% Demo_UARTP_HighLevel.m
clc; clear; close all;

PORT = "COM9";
BAUD = 115200;

sp = uartp_open(PORT, BAUD);
fprintf("Conectado a %s @ %d\n", PORT, BAUD);

%% =========================================================
%  1) DEMO TF (modo 0)
% =========================================================
fprintf("\n=== DEMO TF (modo 0) ===\n");

uartp_reset(sp);
uartp_setmode(sp, 0);  % TF
fprintf("OK: setmode(TF)\n");

% num/den de ejemplo (6 coef cada uno)
num6 = [ 0.10  -0.20  0.30  -0.40  0.50  -0.60 ];
den6 = [ 1.00   0.05 -0.03  0.02  -0.01  0.005 ];
N = 20;
period = 1500;
coeffs_tf = uartp_make_tf(num6, den6,N,period);

uartp_send_coeffs(sp, coeffs_tf, true);
fprintf("OK: TF coef enviados y verificados con 't'\n");

rx_tf = uartp_get_coeffs(sp);

% Parse TF: reconstruye num6/den6 desde lo recibido
[num6_rx, den6_rx, res_tf] = uartp_parse_tf(rx_tf);

fprintf("TF rx num6 = "); disp(num6_rx);
fprintf("TF rx den6 = "); disp(den6_rx);
fprintf("TF reserved(13..16) = "); disp(res_tf);

% Verificación fuerte
assert(isequal(rx_tf(:), single(coeffs_tf(:))), "TF: rx coeffs != tx coeffs");
assert(isequal(num6_rx, single(num6)), "TF: num6 parseado no coincide");
assert(isequal(den6_rx, single(den6)), "TF: den6 parseado no coincide");
fprintf("OK: TF rx == tx y parse TF OK\n");

u0_tf = 0.25;
uartp_init(sp, u0_tf);
fprintf("OK: init(TF) u0=%.3f (entra CONTROL)\n", u0_tf);

uartp_stop(sp, true);
fprintf("OK: stop + back to COMMAND (TF)\n");


%% =========================================================
%  2) DEMO SS (modo 4: SS actual con integrador)
% =========================================================
fprintf("\n=== DEMO SS (modo 4) ===\n");

uartp_reset(sp);
uartp_setmode(sp, 4);
fprintf("OK: setmode(SS act + integrador)\n");

A  = [1 2; 3 4];
B  = [5; 6];
C  = [7 8];
D  = 9;
L  = [0.1; 0.2];
K  = [1.1; 1.2];
Ki = 0.5;

coeffs_ss = uartp_make_ss(A,B,C,D,L,K,Ki, N, period);

uartp_send_coeffs(sp, coeffs_ss, true);
fprintf("OK: SS coef enviados y verificados con 't'\n");

rx_ss = uartp_get_coeffs(sp);

% Parse SS: reconstruye A,B,C,D,L,K,Ki desde lo recibido
[A_rx,B_rx,C_rx,D_rx,L_rx,K_rx,Ki_rx,res_ss] = uartp_parse_ss(rx_ss);

fprintf("SS rx A =\n"); disp(A_rx);
fprintf("SS rx B =\n"); disp(B_rx);
fprintf("SS rx C =\n"); disp(C_rx);
fprintf("SS rx D =\n"); disp(D_rx);
fprintf("SS rx L =\n"); disp(L_rx);
fprintf("SS rx K =\n"); disp(K_rx);
fprintf("SS rx Ki =\n"); disp(Ki_rx);
fprintf("SS reserved(15..16) = "); disp(res_ss);

% Verificación fuerte
assert(isequal(rx_ss(:), single(coeffs_ss(:))), "SS: rx coeffs != tx coeffs");
assert(isequal(A_rx, single(A)), "SS: A parseada no coincide");
assert(isequal(B_rx, single(B)), "SS: B parseada no coincide");
assert(isequal(C_rx, single(C)), "SS: C parseada no coincide");
assert(isequal(D_rx, single(D)), "SS: D parseada no coincide");
assert(isequal(L_rx, single(L)), "SS: L parseada no coincide");
assert(isequal(K_rx, single(K)), "SS: K parseada no coincide");
assert(isequal(Ki_rx, single(Ki)), "SS: Ki parseada no coincide");
fprintf("OK: SS rx == tx y parse SS OK\n");

u0_ss = 0.33;
uartp_init(sp, u0_ss);
fprintf("OK: init(SS) u0=%.3f (entra CONTROL)\n", u0_ss);

uartp_stop(sp, true);
fprintf("OK: stop + back to COMMAND (SS)\n");


%% =========================================================
%  3) EXTRA: RAW BYTES (cualquier cosa)
% =========================================================
fprintf("\n=== DEMO RAW BYTES ===\n");

uartp_reset(sp);
uartp_setmode(sp, 0);

raw_tx = uint8(randi([0 255], 64, 1));
uartp_send_coeffs(sp, raw_tx, true);
fprintf("OK: RAW 64 bytes enviados y verificados con 't'\n");

raw_rx = uartp_get_coeffs_raw(sp);
assert(isequal(raw_rx(:), raw_tx(:)), "RAW: bytes no coinciden");
fprintf("OK: RAW rx == tx\n");

fprintf("\n✅ DEMO COMPLETA OK\n");
clear sp;



%% ========================= UARTP High-Level API (MATLAB) =========================
% Este conjunto de helpers envuelve el protocolo UART robusto del PSoC en funciones
% "de alto nivel". La idea es que vos puedas operar el sistema con llamadas simples:
%   - resetear
%   - elegir modo de controlador (TF vs SS + variantes)
%   - cargar coeficientes (siempre 64 bytes = 16 floats)
%   - verificar lo cargado (con comando 't')
%   - inicializar el control (i) y luego pararlo (s)
%
% ------------------------------------------------------------------------------
% CONCEPTO GENERAL DEL PROTOCOLO (resumen)
% ------------------------------------------------------------------------------
% - Los comandos son 1 byte:
%     'r' reset (CySoftwareReset)
%     'm' set mode (payload 4B: [mode 0 0 0])
%     'c' cargar coeficientes (payload fijo 64B)
%     't' transmitir coeficientes almacenados (devuelve 64B)
%     'i' init control (payload 4B = float u0)
%     's' stop (en modo CONTROL lo atiende la ISR, mínimo overhead)
%
% - Respuestas 1 byte:
%     'R' ready para recibir payload (RX)
%     'S' ready para transmitir payload (TX)
%     'K' ok / operación completada
%     '!' error / comando inválido
%
% - Transferencia de payload (robusta):
%     Se manda en "words" de 4 bytes.
%     Cada word hace eco y se confirma con ACK/NAK.
%     Esto permite mandar cualquier byte (0x00..0xFF) sin depender de delimitadores
%     y sin desync típico de paquetes largos en UART.
%
% ------------------------------------------------------------------------------
% TAMAÑOS Y FORMATO DE COEFICIENTES
% ------------------------------------------------------------------------------
% - Siempre se usan 16 floats (16*4 = 64 bytes) como bloque de coeficientes.
% - TF (modo 0):
%     coeffs(1..6)  = numerador (6)
%     coeffs(7..12) = denominador (6)
%     coeffs(13..16) reservados (podés usar luego para flags, etc.)
%
% - SS (modos 1..4) (2 estados):
%     coeffs(1..4)  = A11 A12 A21 A22
%     coeffs(5..6)  = B1  B2
%     coeffs(7..8)  = C1  C2
%     coeffs(9)     = D
%     coeffs(10..11)= L1  L2 (ganancias observador)
%     coeffs(12..13)= K1  K2 (realimentación de estado)
%     coeffs(14)    = Ki (integrador, si aplica; si 0 se considera "sin integrador")
%     coeffs(15..16) reservados
%
% ------------------------------------------------------------------------------
% MODOS (uartp_setmode)
% ------------------------------------------------------------------------------
% mode = 0..4:
%   0 = TF (compensador en cascada)
%   1 = SS + observador predictor SIN integrador
%   2 = SS + observador actual    SIN integrador
%   3 = SS + observador predictor CON integrador
%   4 = SS + observador actual    CON integrador
%
% Nota: el firmware NO permite cambiar coeficientes en "CONTROL mode".
%       En CONTROL solo se acepta 's' (stop) por ISR.
% ------------------------------------------------------------------------------
% FUNCIONES DE ALTO NIVEL (lo que vos usás normalmente)
% ------------------------------------------------------------------------------
%
% 1) sp = uartp_open(port, baud)
%    Qué hace:
%      - Abre el puerto serial con configuración 8N1 (8 bits, no parity, 1 stop).
%      - Hace flush inicial y drena cualquier basura en RX.
%
%    Parámetros:
%      - port (string):
%          Nombre del puerto.
%          Ejemplos: "COM9", "COM3" (Windows) / "/dev/ttyUSB0" (Linux).
%      - baud (double/int):
%          Baudrate del UART (ej: 115200).
%
%    Retorna:
%      - sp (serialport):
%          Objeto de MATLAB serialport ya listo para usar.
%
%
% 2) uartp_reset(sp, timeout_s=1.0)
%    Qué hace:
%      - Envía el comando 'r' (reset).
%      - Espera respuesta 'K' (OK).
%      - Espera un rato para que el PSoC reinicie y drena RX.
%
%    Parámetros:
%      - sp (serialport):
%          Puerto serial abierto.
%      - timeout_s (double, opcional):
%          Timeout máximo para esperar la respuesta del PSoC (en segundos).
%          Si estás en una PC lenta o el PSoC tarda, subilo (ej: 2.0).
%
%    Retorna:
%      - Nada (lanza error si falla).
%
%
% 3) uartp_setmode(sp, mode, timeout_s=2.0)
%    Qué hace:
%      - Envía el comando 'm' (set mode).
%      - El PSoC responde 'R' (READY para recibir payload).
%      - Se manda un payload fijo de 4 bytes: [mode, 0, 0, 0].
%      - El PSoC responde 'K' (OK).
%      - Define cómo el firmware interpretará los 16 floats del comando 'c'.
%
%    Parámetros:
%      - sp (serialport):
%          Puerto serial abierto.
%      - mode (int 0..4):
%          Selección del "tipo de controlador" (lo interpreta el firmware):
%            0 = TF (transfer function, compensador por numerador/denominador)
%            1 = SS predictor sin integrador
%            2 = SS actual    sin integrador
%            3 = SS predictor con integrador
%            4 = SS actual    con integrador
%      - timeout_s (double, opcional):
%          Timeout para este comando (en segundos).
%
%    Retorna:
%      - Nada (lanza error si falla).
%
%
% 4) coeffs = uartp_make_tf(num6, den6)
%    Qué hace:
%      - Construye un vector single(16x1) con el layout TF esperado por el firmware.
%
%    Parámetros:
%      - num6 (vector de 6 números):
%          Coeficientes del numerador del compensador (orden fijo: 6 valores).
%          Ej: [b0 b1 b2 b3 b4 b5]
%      - den6 (vector de 6 números):
%          Coeficientes del denominador del compensador (6 valores).
%          Ej: [a0 a1 a2 a3 a4 a5]
%
%    Retorna:
%      - coeffs (single(16,1)):
%          Vector listo para uartp_send_coeffs()
%          Mapeo:
%            coeffs(1..6)  = num6
%            coeffs(7..12) = den6
%            coeffs(13..16)= reservados (0)
%
%
% 5) coeffs = uartp_make_ss(A,B,C,D,L,K,Ki)
%    Qué hace:
%      - Construye single(16x1) con el layout SS (2 estados) esperado por el firmware.
%
%    Parámetros:
%      - A (2x2):
%          Matriz de estado.
%      - B (2x1):
%          Vector de entrada.
%      - C (1x2 o 2x1):
%          Vector de salida.
%      - D (escalar):
%          Ganancia directa.
%      - L (2x1):
%          Ganancias del observador (L1, L2).
%      - K (2x1):
%          Ganancias de realimentación de estado (K1, K2).
%      - Ki (escalar):
%          Ganancia del integrador (si no se usa integrador, poné Ki = 0).
%
%    Retorna:
%      - coeffs (single(16,1)):
%          Layout:
%            (1..4)   A11,A12,A21,A22
%            (5..6)   B1,B2
%            (7..8)   C1,C2
%            (9)      D
%            (10..11) L1,L2
%            (12..13) K1,K2
%            (14)     Ki
%            (15..16) reservados
%
%
% 6) uartp_send_coeffs(sp, coeffs_or_u8, verify_roundtrip=true, timeout_s=2.0)
%    Qué hace:
%      - Envía el comando 'c' (cargar coeficientes).
%      - El PSoC responde 'R' (READY RX).
%      - Se envían exactamente 64 bytes con handshake robusto por words de 4 bytes.
%      - El PSoC responde 'K' (OK) cuando terminó de recibir y guardar.
%      - Opcionalmente verifica con 't' que el PSoC guardó exactamente lo mismo.
%
%    Parámetros:
%      - sp (serialport):
%          Puerto serial abierto.
%      - coeffs_or_u8 (single/double(16) o uint8(64)):
%          Dos formas de usarla:
%            a) single/double con 16 floats:
%               - Se convierte a single y se envía como 64 bytes.
%               - Ideal para TF/SS (coeficientes).
%            b) uint8(64):
%               - Se envía tal cual (RAW), útil para tests o debug.
%      - verify_roundtrip (bool, opcional):
%          true  -> después de enviar hace un 't' y compara bytes (muy recomendable).
%          false -> no verifica, más rápido.
%      - timeout_s (double, opcional):
%          Timeout para cada etapa del handshake (segundos).
%
%    Retorna:
%      - Nada (lanza error si falla o si verify_roundtrip detecta mismatch).
%
%
% 7) coeffs = uartp_get_coeffs(sp, timeout_s=2.0)
%    Qué hace:
%      - Envía 't' para pedir los coeficientes almacenados en el PSoC.
%      - El PSoC responde 'S' (READY TX) y manda 64 bytes con handshake.
%      - Convierte esos 64 bytes a single(16x1).
%
%    Parámetros:
%      - sp (serialport):
%          Puerto serial abierto.
%      - timeout_s (double, opcional):
%          Timeout de recepción.
%
%    Retorna:
%      - coeffs (single(16,1)):
%          Los 16 floats exactamente como están guardados en el PSoC.
%
%
% 8) raw = uartp_get_coeffs_raw(sp, timeout_s=2.0)
%    Qué hace:
%      - Igual que uartp_get_coeffs, pero devuelve uint8(64) sin convertir.
%
%    Parámetros:
%      - sp (serialport)
%      - timeout_s (double, opcional)
%
%    Retorna:
%      - raw (uint8(64,1)):
%          Bytes exactos almacenados en el PSoC.
%
%
% 9) uartp_init(sp, u0, timeout_s=2.0)
%    Qué hace:
%      - Envía el comando 'i' (iniciar controlador).
%      - El PSoC responde 'R' (READY RX).
%      - Se envía un payload de 4 bytes que representa un float: u0.
%      - El PSoC responde 'K' y pasa a modo CONTROL.
%
%    Parámetros:
%      - sp (serialport):
%          Puerto serial abierto.
%      - u0 (float / double / single):
%          Valor inicial de entrada/condición inicial para el controlador.
%          En tu caso: "la entrada inicial del controlador" (un solo float).
%          Se envía como IEEE754 single (4 bytes).
%      - timeout_s (double, opcional):
%          Timeout de handshake.
%
%    Retorna:
%      - Nada.
%
%    Nota:
%      - Una vez en CONTROL, el PSoC debe ignorar/rechazar comandos como 'c','m','t','i'.
%      - En CONTROL sólo se permite 's' (stop) por ISR para minimizar carga.
%
%
% 10) uartp_stop(sp, wait_back_to_command=false, timeout_s=2.0)
%     Qué hace:
%       - Envía el comando 's' (stop).
%       - El PSoC responde 'K' (stop recibido / ejecutado).
%       - Opcional: espera a que el firmware vuelva a COMMAND mode.
%
%     Parámetros:
%       - sp (serialport):
%           Puerto serial abierto.
%       - wait_back_to_command (bool, opcional):
%           false -> solo confirma que el stop fue aceptado (rápido).
%           true  -> además espera que termine el stop suave y vuelva a COMMAND.
%                   Implementación MATLAB: “poll” de 't' hasta que responda 'S'.
%       - timeout_s (double, opcional):
%           Timeout para recibir la 'K' inicial.
%           Ojo: si usás wait_back_to_command=true, internamente espera más (≈ 5s).
%
%     Retorna:
%       - Nada.
%
% ------------------------------------------------------------------------------
% EJEMPLO DE FLUJO RECOMENDADO (sesión típica)
% ------------------------------------------------------------------------------
% sp = uartp_open("COM9",115200);
% uartp_reset(sp);
% uartp_setmode(sp, 0); % TF
% coeffs = uartp_make_tf(num6, den6);
% uartp_send_coeffs(sp, coeffs, true); % verifica con 't'
% uartp_init(sp, 0.25); % u0 = entrada inicial del controlador
% ... control corriendo ...
% uartp_stop(sp, true); % stop + espera volver a COMMAND
% clear sp
% === Helpers de "parse" (camino inverso) ===
% - uartp_parse_tf(coeffs16):
%     Toma coeffs (16 floats) y te devuelve:
%       num6 = coeffs(1..6)
%       den6 = coeffs(7..12)
%       reserved = coeffs(13..16)
%
% - uartp_parse_ss(coeffs16):
%     Reconstruye matrices/vectores:
%       A = [c1 c2; c3 c4]
%       B = [c5; c6]
%       C = [c7 c8]
%       D = c9
%       L = [c10; c11]
%       K = [c12; c13]
%       Ki = c14
%       reserved = c15..c16
%
% Esto sirve para:
%   (1) leer lo que el PSoC guardó con uartp_get_coeffs()
%   (2) convertirlo a objetos “humanos” (num/den o A,B,C,D,...) para debug o logging
% ------------------------------------------------------------------------------
% NOTAS IMPORTANTES (para no romperlo sin querer)
% ------------------------------------------------------------------------------
% - No intentes 'c','m','i','t' mientras el PSoC está en CONTROL (salvo 's').
% - Si el control real deshabilita interrupciones o hace secciones críticas largas,
%   podés provocar timeouts. El protocolo es robusto, pero necesita que el UART/ISR
%   tenga chances de correr.
% - verify_roundtrip=true te salva de bugs sutiles (endianness, offsets, corrupción).
%   En producción podés poner false para ganar velocidad.
% ==============================================================================


