# Documentación — Librería `tfmini_psoc` (TFMini Plus en PSoC 5LP)

## 1. Objetivo

La librería `tfmini_psoc` implementa la recepción y decodificación de tramas (“frames”) del sensor **TFMini Plus** usando un **UART hardware** del PSoC. Provee una API simple para:

* drenar el buffer de recepción UART continuamente (`tfmini_poll`)
* detectar y validar tramas del TFMini (header + checksum)
* entregar el último dato válido de distancia/strength/temperatura (`tfmini_get`)
* filtrar mediciones fuera de rango (anti-outliers) (`tfmini_set_range`)

La salida de debug (impresión por terminal) **no está dentro de la librería**: se hace desde `main` con `uart_dbg`.

---

## 2. Arquitectura / Integración en PSoC Creator

### 2.1 Componentes requeridos

* **UART `uart`**: conectado al TFMini Plus (mínimo RX).
* (Opcional) **UART `uart_dbg`**: para enviar logs a PuTTY (debug).

### 2.2 Conexiones eléctricas mínimas (lectura unidireccional)

* **TFMini VCC → 5V**
* **TFMini GND → GND común**
* **TFMini TX → PSoC Rx_1** (pin RX del UART `uart`)

Notas prácticas:

* Configurar el pin RX en **LVTTL** para leer correctamente niveles de 3.3V.
* Si hay ruido/cable largo, es recomendable resistencia serie 330Ω–1kΩ en la línea TX→RX y desacople (100nF + 47–100µF) cerca del sensor.

---

## 3. Protocolo del TFMini Plus (modo estándar)

El TFMini entrega tramas de **9 bytes** en stream continuo:

| Byte | Contenido  |
| ---- | ---------- |
| 0    | 0x59       |
| 1    | 0x59       |
| 2    | Dist L     |
| 3    | Dist H     |
| 4    | Strength L |
| 5    | Strength H |
| 6    | Temp L     |
| 7    | Temp H     |
| 8    | Checksum   |

**Checksum**: suma de bytes 0..7 (8 bytes) y se compara con el byte 8 (8-bit wrap):
[
\text{chk} = \left(\sum_{i=0}^{7} b_i\right)\ &\ 0xFF
]

La librería se sincroniza buscando el header `0x59 0x59` y recién ahí arma el frame.

---

## 4. Diseño interno de la librería

### 4.1 Parser de stream (máquina simple)

El parser funciona como una máquina de estados mínima con índice `s_idx`:

* **Estado 0**: espera `0x59` (primer header)
* **Estado 1**: espera `0x59` (segundo header)
* **Estado 2..8**: acumula bytes hasta completar 9 bytes
* Luego valida checksum:

  * si falla: incrementa `frames_bad`
  * si pasa: decodifica y actualiza estructura “último válido”

Este enfoque permite manejar streams continuos sin depender de delimitadores externos.

### 4.2 Drenaje del UART (modelo polling)

La librería no usa interrupciones por defecto: `tfmini_poll()` lee todo lo disponible con:

* `uart_GetRxBufferSize()`
* `uart_ReadRxData()`

y alimenta byte a byte al parser.

**Requisito operativo**: `tfmini_poll()` debe llamarse frecuentemente (idealmente en el loop principal con latencia baja) para evitar overflow del RX buffer. En entornos con mucho debug printing o alta tasa de frames, se recomienda aumentar el RX buffer del componente UART.

### 4.3 Filtro de plausibilidad (anti-outliers)

Además del checksum, la librería aplica un filtro simple sobre `dist`:

* se define un rango válido `[min_dist, max_dist]`
* si el valor cae fuera, se descarta la medición (evita “valores imposibles” por frames espurios)

Esto se configura con:

```c
tfmini_set_range(min, max);
```

### 4.4 Conversión de temperatura

El sensor entrega `temp_raw`. La librería computa `temp_c` con la fórmula típica:

[
T(\degree C)=\frac{temp_raw}{8}-256
]

En proyectos con `printf` sin floats (común en newlib-nano), se recomienda imprimir temperatura en décimas (sin float) desde `main`.

---

## 5. API pública

### 5.1 `void tfmini_init(void)`

Inicializa el estado interno del parser y contadores.
No arranca el UART (eso se hace afuera con `uart_Start()`).

**Uso:**

```c
uart_Start();
tfmini_init();
```

### 5.2 `void tfmini_set_range(uint16_t min_dist, uint16_t max_dist)`

Define el rango de distancia aceptado por el filtro de plausibilidad.

**Ejemplo:**

```c
tfmini_set_range(1, 1200);
```

### 5.3 `uint8_t tfmini_poll(void)`

Drena el RX del UART y procesa bytes.
Retorna `1` si durante ese llamado se capturó al menos un frame válido.

**Uso típico:**

```c
if (tfmini_poll()) { ... }
```

### 5.4 `uint8_t tfmini_get(tfmini_data_t *out)`

Copia el último dato válido (dist/strength/temp + contadores) en `out`.

* Retorna `1` si ya existe al menos una medición válida.
* Retorna `0` si todavía no se capturó ningún frame válido.

**Patrón correcto (evita basura por structs no inicializados):**

```c
tfmini_data_t d = (tfmini_data_t){0};
uint8_t valid = tfmini_get(&d);
if (valid) { /* usar d */ }
```

---

## 6. Buenas prácticas de robustez (lecciones reales)

### 6.1 Aumentar RX Buffer Size

Si `frames_bad` crece muy rápido y `frames_ok` casi no sube, suele ser overflow o pérdida de bytes.

En el componente UART (`uart`) se recomienda:

* RX Buffer Size: **256 o 512**
* mantener el loop principal sin bloqueos largos.

### 6.2 No imprimir por cada byte / por cada frame

Imprimir por UART debug es lento y puede bloquear el CPU, lo que aumenta overflow.
Recomendación:

* imprimir cada **200–500 ms**
* llamar `tfmini_poll()` lo más seguido posible.

### 6.3 Integridad de señal

Cables largos, fuentes ruidosas o proximidad a ESC/motores pueden corromper el stream.
Mitigación:

* resistencia serie 330Ω–1kΩ en TX→RX
* desacople cerca del sensor (100nF + 47–100µF)

---

## 7. Ejemplo de uso (resumen)

Secuencia recomendada:

1. Hardware y pins configurados (RX con LVTTL)
2. En `main`:

```c
uart_Start();
uart_dbg_Start();     /* opcional */
tfmini_init();
tfmini_set_range(1, 1200);

for(;;) {
    tfmini_poll();    /* lo más seguido posible */
    /* cada 200ms: tfmini_get + print */
}
```

---

## 8. Limitaciones y extensiones futuras

* La versión actual es “polling”. Para cargas altas o tareas críticas, se puede migrar a:

  * ISR de RX (leer bytes en interrupción y parsear en background)
  * DMA (si aplica)
* No implementa comandos hacia el sensor (TX hacia TFMini). Para configurar baud o tasa de salida habría que:

  * habilitar TX del UART
  * implementar funciones `tfmini_send_cmd(...)`

---
