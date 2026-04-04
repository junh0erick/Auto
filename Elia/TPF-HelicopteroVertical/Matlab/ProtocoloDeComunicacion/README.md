# UARTP High‑Level API (MATLAB)

Helpers MATLAB para manejar un **protocolo UART robusto** implementado en un PSoC. La idea es operar el sistema con llamadas simples:

* reset
* selección de modo de controlador (**TF** vs **State‑Space** + variantes)
* carga de coeficientes (**siempre 64 bytes = 16 floats**)
* verificación de lo cargado (round‑trip con `t`)
* iniciar control (`i`) y detenerlo (`s`)

> Este README describe **la API de alto nivel** (lado MATLAB) y **cómo el firmware espera los datos**.

---

## Resumen del protocolo

### Comandos (1 byte)

* `r` → reset (CySoftwareReset)
* `m` → set mode (payload 4B: `[mode 0 0 0]`)
* `c` → cargar coeficientes (payload fijo 64B)
* `t` → transmitir coeficientes almacenados (devuelve 64B)
* `i` → init control (payload 4B = float `u0`)
* `s` → stop (**en modo CONTROL lo atiende la ISR**, mínimo overhead)

### Respuestas (1 byte)

* `R` → READY para recibir payload (RX)
* `S` → READY para transmitir payload (TX)
* `K` → OK / operación completada
* `!` → ERROR / comando inválido

### Transferencia de payload (robusta)

El payload se envía en **words de 4 bytes**. Cada word:

1. se envía
2. hace **eco** (echo)
3. se confirma con **ACK/NAK**

Esto permite enviar cualquier byte `0x00..0xFF` sin delimitadores y evita desync típico de paquetes largos por UART.

---

## Tamaños y formato de coeficientes

Siempre se usan **16 floats** → `16 * 4 = 64 bytes`.

### TF (modo 0)

* `coeffs(1..6)`  = numerador (6)
* `coeffs(7..12)` = denominador (6)
* `coeffs(13..16)` = reservados (flags / uso futuro)

### State‑Space (modos 1..4) — 2 estados

* `coeffs(1..4)`   = `A11 A12 A21 A22`
* `coeffs(5..6)`   = `B1  B2`
* `coeffs(7..8)`   = `C1  C2`
* `coeffs(9)`      = `D`
* `coeffs(10..11)` = `L1 L2` (observador)
* `coeffs(12..13)` = `K1 K2` (realimentación de estado)
* `coeffs(14)`     = `Ki` (integrador; si `0` ⇒ “sin integrador”)
* `coeffs(15..16)` = reservados

---

## Modos disponibles (`uartp_setmode`)

`mode = 0..4`:

* `0` = TF (compensador en cascada)
* `1` = SS + observador predictor **SIN** integrador
* `2` = SS + observador actual    **SIN** integrador
* `3` = SS + observador predictor **CON** integrador
* `4` = SS + observador actual    **CON** integrador

> **Restricción del firmware:** no permite cambiar coeficientes en **CONTROL mode**.
> En CONTROL solo se acepta `s` (stop) por ISR.

---

## Flujo recomendado (sesión típica)

```matlab
sp = uartp_open("COM9",115200);

uartp_reset(sp);

uartp_setmode(sp, 0); % TF

coeffs = uartp_make_tf(num6, den6);

uartp_send_coeffs(sp, coeffs, true); % verifica con 't'

uartp_init(sp, 0.25); % u0 = entrada inicial

% ... control corriendo ...

uartp_stop(sp, true); % stop + espera volver a COMMAND

clear sp
```

---

## API de alto nivel (MATLAB)

### 1) `sp = uartp_open(port, baud)`

**Qué hace**

* Abre el puerto serial en 8N1.
* Hace flush inicial y drena basura en RX.

**Parámetros**

* `port` (string): ej. `"COM9"` (Windows) o `"/dev/ttyUSB0"` (Linux)
* `baud` (int): ej. `115200`

**Retorna**

* `sp` (`serialport`): objeto serial listo para usar.

---

### 2) `uartp_reset(sp, timeout_s=1.0)`

**Qué hace**

* Envía `r`.
* Espera `K`.
* Espera un momento para el reinicio y drena RX.

**Notas**

* Si la PC es lenta o el PSoC tarda, subir `timeout_s` (ej: `2.0`).

---

### 3) `uartp_setmode(sp, mode, timeout_s=2.0)`

**Qué hace**

* Envía `m`.
* Espera `R`.
* Envía payload 4B: `[mode, 0, 0, 0]`.
* Espera `K`.

**Parámetros**

* `mode` (int 0..4): selección del tipo de controlador (interpretado por firmware).

---

### 4) `coeffs = uartp_make_tf(num6, den6)`

**Qué hace**

* Construye `single(16x1)` con el layout TF.

**Parámetros**

* `num6` (1x6): `[b0 b1 b2 b3 b4 b5]`
* `den6` (1x6): `[a0 a1 a2 a3 a4 a5]`

**Retorna**

* `coeffs` (`single(16,1)`):

  * `coeffs(1..6)=num6`, `coeffs(7..12)=den6`, `coeffs(13..16)=0`

---

### 5) `coeffs = uartp_make_ss(A,B,C,D,L,K,Ki)`

**Qué hace**

* Construye `single(16x1)` con el layout SS (2 estados).

**Parámetros**

* `A` (2x2)
* `B` (2x1)
* `C` (1x2 o 2x1)
* `D` (escalar)
* `L` (2x1): observador
* `K` (2x1): realimentación
* `Ki` (escalar): integrador (`0` si no se usa)

**Retorna**

* `coeffs` (`single(16,1)`): layout descrito en “Tamaños y formato de coeficientes”.

---

### 6) `uartp_send_coeffs(sp, coeffs_or_u8, verify_roundtrip=true, timeout_s=2.0)`

**Qué hace**

* Envía `c`.
* Espera `R`.
* Envía exactamente **64 bytes** usando handshake por words de 4 bytes.
* Espera `K`.
* Opcional: verifica con `t` (round‑trip) que el PSoC guardó lo mismo.

**Entrada**

* `single/double(16)` → se convierte a `single` y se envía como 64B.
* `uint8(64)` → se envía RAW.

**Recomendación**

* `verify_roundtrip=true` durante desarrollo/debug (salva de endianness/offsets/corrupción).
* En producción podés poner `false` para más velocidad.

---

### 7) `coeffs = uartp_get_coeffs(sp, timeout_s=2.0)`

**Qué hace**

* Envía `t`.
* Espera `S`.
* Recibe 64B con handshake.
* Convierte a `single(16x1)`.

---

### 8) `raw = uartp_get_coeffs_raw(sp, timeout_s=2.0)`

**Qué hace**

* Igual que `uartp_get_coeffs`, pero devuelve `uint8(64)` sin convertir.

---

### 9) `uartp_init(sp, u0, timeout_s=2.0)`

**Qué hace**

* Envía `i`.
* Espera `R`.
* Envía payload 4B IEEE754 `single` con `u0`.
* Espera `K` y pasa a **CONTROL mode**.

**Nota importante**

* En CONTROL el firmware debe ignorar/rechazar `c/m/t/i`.
* En CONTROL solo se acepta `s` por ISR.

---

### 10) `uartp_stop(sp, wait_back_to_command=false, timeout_s=2.0)`

**Qué hace**

* Envía `s`.
* Espera `K`.

**Opcional**

* `wait_back_to_command=true` → además espera volver a COMMAND mode (poll con `t` hasta que responda `S`).

---

## Helpers de parse (camino inverso)

Útiles para:

* leer lo guardado con `uartp_get_coeffs()`
* convertirlo a formato “humano” para debug/log

### `uartp_parse_tf(coeffs16)`

Devuelve:

* `num6 = coeffs(1..6)`
* `den6 = coeffs(7..12)`
* `reserved = coeffs(13..16)`

### `uartp_parse_ss(coeffs16)`

Reconstruye:

* `A = [c1 c2; c3 c4]`
* `B = [c5; c6]`
* `C = [c7 c8]`
* `D = c9`
* `L = [c10; c11]`
* `K = [c12; c13]`
* `Ki = c14`
* `reserved = c15..c16`

---

## Notas importantes (para no romperlo sin querer)

* **No** intentes `c`, `m`, `i` o `t` mientras el PSoC está en **CONTROL mode** (salvo `s`).
* Si el control real deshabilita interrupciones o hace secciones críticas largas, podés provocar timeouts: el protocolo es robusto, pero necesita que el UART/ISR tenga chances de correr.
* `verify_roundtrip=true` te salva de bugs sutiles (endianness, offsets, corrupción). En producción podés usar `false` para mayor velocidad.
