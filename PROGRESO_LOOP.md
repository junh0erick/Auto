# Progreso Loop — Péndulo Invertido v6.2
**Fecha inicio:** 2026-04-05  
**Objetivo:** Control en PSoC, botón único de envío de parámetros, timer restaurado, telemetría 32B con xhat, baud 921600, CMSIS-DSP, isr_2, tipo numérico seleccionable.

---

## Arquitectura v6.2

| Componente | Rol |
|---|---|
| PSoC | Control completo (ctrl_pend.c con CMSIS-DSP), streaming telemetría 32B/tick |
| MATLAB | Configuración, display, envío de parámetros, simulación con precisión PSoC |

**Flujo:**
1. MATLAB: configura parámetros en popups → guarda en S.cfg
2. MATLAB: Start → serializa todo → envía `'p'` (216B payload, incl. num_type) → PSoC aplica → envía `'i'`
3. PSoC: timer ISR cada Ts → `ctrl_step()` (CMSIS-DSP f32) → telemetría 32B → UART @ 921600
4. PSoC: `isr_2` (UART_1 rx_interrupt) → `UARTP_Rx_ISR()` actúa inmediatamente en 's'/'u'
5. MATLAB: loop no-bloqueante → acumula bytes → procesa tramas 32B → display + simulación

---

## Protocolo v6.2

### COMMAND mode:
- `'r'` → Reset → `'K'`
- `'f'` → Set Fs (compatibilidad) → `'K'`
- `'p'` → **Set all params** → `'K'` / `'E'`
- `'i'` → Start STREAM mode → `'K'`
- `'s'` → Stop → `'K'`

### 'p' payload (216 bytes + 1 checksum XOR):
```
[0]       mode_inner   uint8  (CTRL_MODE_TF/SS_PRED_NOI/SS_ACT_NOI/SS_PRED_I/SS_ACT_I/OL/OFF)
[1]       mode_outer   uint8
[2]       num_type     uint8  (CTRL_NUM_F32=0, F64=1, F16=2, Q31=3, Q15=4, Q7=5)
[3]       padding (cero)
[4-103]   coeffs_inner[25] float32 LE  (100B)
[104-203] coeffs_outer[25] float32 LE  (100B)
[204-207] ref_inner    float32 LE
[208-211] sat_min      float32 LE
[212-215] sat_max      float32 LE
```

### STREAM mode (telemetría PSoC→MATLAB):
- **32 bytes por tick** (8 × float32 LE):

| Offset | Campo | Descripción |
|--------|-------|-------------|
| 0-3   | y1   | omega [rad/s] |
| 4-7   | y2   | theta [rad] |
| 8-11  | u1   | PWM float (sin saturar) |
| 12-15 | u2   | inner ref (=salida outer) |
| 16-19 | x1i  | inner xhat[0] |
| 20-23 | x2i  | inner xhat[1] |
| 24-27 | x1o  | outer xhat[0] |
| 28-31 | x2o  | outer xhat[1] |

- En STREAM mode PSoC acepta (ahora via `isr_2` ISR):
  - `'s'` → stop inmediato
  - `'u'` + 4B float → actualizar ref_inner live sin parar

### Coeficientes layout (ctrl_pend.h):
```
TF:  c[0..5]=b,  c[6..11]=a,  c[14]=N,  c[15]=Fs_hz
SS:  c[0..3]=A(2x2 row-major), c[4..5]=B, c[6..7]=C,
     c[8]=D, c[9..10]=L, c[11..12]=K, c[13]=Kx(Nbar/Ki),
     c[14]=N, c[15]=Fs_hz
```

---

## CMSIS-DSP Integration (v6.2 rev3 — CMSIS 4.4.5)

**Archivos en `4 - Pendulo.cydsn/` (CMSIS 4.4.5 / V1.4.5b, compatible PSoC Creator 4.4):**
- **F32**: `arm_dot_prod_f32.c`, `arm_scale_f32.c`, `arm_add_f32.c`
- **Q31**: `arm_dot_prod_q31.c`, `arm_scale_q31.c`, `arm_add_q31.c`, `arm_float_to_q31.c`, `arm_q31_to_float.c`
- **Q15**: `arm_dot_prod_q15.c`, `arm_scale_q15.c`, `arm_add_q15.c`, `arm_float_to_q15.c`, `arm_q15_to_float.c`
- **Q7**: `arm_dot_prod_q7.c`, `arm_scale_q7.c`, `arm_add_q7.c`, `arm_float_to_q7.c`, `arm_q7_to_float.c`
- `arm_mat_init_f32.c`, `arm_mat_mult_f32.c`
- `arm_math.h` — self-contained (sin dsp/ ni arm_math_types.h)
- `arm_common_tables.h`, `arm_const_structs.h`
- `core_cm3.h`, `core_cmFunc.h`, `core_cmInstr.h`, `core_cmSimd.h`
- `cmsis_gcc.h`, `cmsis_armcc.h`

**Define requerido en PSoC Creator:** `ARM_MATH_CM3` en Preprocessor Defines.

**Uso en ctrl_pend.c:**
```c
#define ARM_MATH_CM3
#include "arm_math.h"

// SS observer: A·x row by row
arm_dot_prod_f32(A_row0, x, 2, &z[0]);
arm_dot_prod_f32(A_row1, x, 2, &z[1]);

// L·innov vectorial
arm_scale_f32(L, innov, Linv, 2);
arm_add_f32(z, Linv, xhat, 2);
```

---

## isr_2 Integration (v6.2)

**Cambio de diseño:** En lugar de polling `UARTP_ControlRxPoll()` en el main loop,
`isr_2` dispara `UARTP_Rx_ISR()` inmediatamente al llegar cada byte.

**Conexión:** isr_2 → UART_1 rx_interrupt (en PSoC Creator)

**Ventaja:** El comando 's' (stop) se ejecuta al instante sin esperar al ciclo del main loop.
Crítico para la seguridad mecánica del péndulo.

**Inicialización:**
```c
void UARTP_Init(void) {
    UART_1_Start();
    isr_2_StartEx(UARTP_Rx_ISR);  // conectar ISR
    ...
}
```

**main.c:** `UARTP_ControlRxPoll()` ya no se llama (reemplazado por comentario).

---

## Tipo Numérico Seleccionable (v6.2)

**MATLAB:** Dropdown `ddNumType` en panel Control → `{'f32','f64','f16','q31','q15','q7'}`
- `payload(3)` en `uartp_send_params()` = índice 0-based del tipo seleccionado
- La simulación en MATLAB usa `num_cast()` para aplicar la misma precisión

**PSoC:** `ctrl_set_num_type(s_p_buf[2])` en `handle_cmd_p()`
- Los defines `CTRL_NUM_F32`..`CTRL_NUM_Q7` en `ctrl_pend.h`
- Actualmente PSoC siempre opera en f32 (CMSIS-DSP); los tipos Q son para extensión futura

**Simulación MATLAB `num_cast(x, type_str)`:**
- `f32` → `double(single(x))` — round-trip float32
- `f64` → `double(x)` — sin pérdida
- `f16` → emulación por truncado de mantisa (10 bits efectivos)
- `q31` → cuantización a 2^31 niveles (normalizado a ±1)
- `q15` → cuantización a 2^15 niveles
- `q7`  → cuantización a 2^7 niveles

---

## Baud Rate y Timing

| Baud | Frame | UART | +USB latency | Margen a Ts=5ms |
|------|-------|------|--------------|-----------------|
| 921600 | 32B | 0.347ms | +1ms = 1.347ms | **3.65ms** ✓ |
| 115200 | 32B | 2.78ms | +1ms = 3.78ms | 1.22ms (ajustado) |

**⚠️ Requerido: PSoC Creator → UART_1 → Baud Rate = 921600 → rebuild + flash**

---

## Estado actual — COMPLETO v6.3 (2026-04-06)

### PSoC (todos los archivos listos para rebuild en PSoC Creator):
- [x] `ctrl_pend.h` — v6.3: CTRL_NUM_* (Q31/Q15/Q7 implementados), nota c[16]=q_scale
- [x] `ctrl_pend.c` — **v6.3**: paths Q31/Q15/Q7 completos para observer SS
  - Inline helpers: flt_to_q31n/q31_to_flt, flt_to_q15n/q15_to_flt, flt_to_q7n/q7_to_flt
  - Sat helpers: qsat31, qsat15, qsat7
  - Q31 helpers: ss_yhat_q31, ss_predict_q31, ss_obs_pred_q31, ss_obs_act_correct_q31, ss_obs_act_predict_q31, ss_u_cmd_q31
  - Q15 helpers: ídem con q15_t, arm_dot_prod/scale/add_q15
  - Q7 helpers: ídem con q7_t, arm_dot_prod/scale/add_q7
  - Despacho: obs_pred(), obs_act_correct(), obs_act_predict(), u_cmd() — switch g_num_type
  - run_step: usa despacho, F32 para TF (Q no aplica a DF2T de 6 coefs)
  - ctrl_apply_coeffs: parsea c[16]=q_scale, pre-convierte A/B/C/D/K/L/Kx a Q31/Q15/Q7 con arm_float_to_q31/q15/q7
- [x] `pendulo.c`   — ISR restaurado + Timer_1_Start en pendulo_init
- [x] `uartp_pend.h` — UARTP_PendingRef, CY_ISR_PROTO(UARTP_Rx_ISR), protocolo v6.2
- [x] `uartp_pend.c` — handle_cmd_p parsea byte[2] num_type, UARTP_Rx_ISR implementado, UARTP_Init llama isr_2_StartEx
- [x] `main.c`       — TELEM_FRAME_SZ=32, 8 floats, UARTP_ControlRxPoll removido (→ isr_2)
- [x] **CMSIS Q31/Q15/Q7**: 15 archivos .c copiados a `4 - Pendulo.cydsn/`:
  - arm_dot_prod_q31/q15/q7.c, arm_scale_q31/q15/q7.c, arm_add_q31/q15/q7.c
  - arm_float_to_q31/q15/q7.c, arm_q31/q15/q7_to_float.c

### MATLAB (pendulo_gui2.m — listo para ejecutar):
- [x] `TELEM_SZ = 32` — 8×float32
- [x] Baud default = 921600
- [x] `uartp_reset()` — envía `'s'` antes de `'r'`
- [x] `onStart()` — send params + start stream
- [x] `runStreamLoop()` — loop no-bloqueante, tramas 32B
- [x] `processTelemFrame()` — decode 32B: y1 y2 u1 u2 x1i x2i x1o x2o
- [x] `build_coeffs_for_psoc()` — serializa S.cfg → float[25]; **c(17)=q_scale** (v6.3)
- [x] `uartp_send_params()` — 'p' + 216B + XOR; payload[3]=num_type del dropdown
- [x] `uartp_update_ref()` — 'u' + float32 (live ref sin parar)
- [x] `ddNumType` dropdown — f32/f64/f16/q31/q15/q7
- [x] **`num_cast(x, type_str, q_scale)`** — v6.3: parámetro q_scale para normalizar señales físicas en Q31/Q15/Q7 igual que PSoC
- [x] **`processTelemFrame`** — v6.3: sim_cast1/sim_cast2 con q_scale por planta; c1.q_scale y c2.q_scale
- ~~`btnSend`~~ — eliminado; todo el envío de params se hace vía **Start** (simplificado)
- [x] `openSimPopup()` — carga modelo TF o SS (continuo/discreto), Ts configurable
- [x] Anti-windup back-calculation en `ss_pred_step`/`ss_act_step`; `loop_step` ya NO sobreescribe `u_prev`
- [x] `num_type` en `.mat` exportado y en sesión guardada/cargada
- [x] **`openCtrlPopup`** — v6.3: fila q_scale en panel SS; `et_qs` numeric field; leído en pDoApply
- [x] **`cfg_default()`** — añadido `q_scale=1.0`
- [x] **`onLoadSession`** — retrocompat: si sess sin q_scale → default 1.0

---

## Pendientes / Acciones requeridas

### Usuario debe hacer:
- [x] **`.cyprj` editado** (2026-04-06): los 15 .c Q-format ya están en el archivo de proyecto. PSoC Creator los detectará al reabrir el proyecto.
- [x] **`ARM_MATH_CM3` auto-detección** (2026-04-06): arm_math.h modificado para detectar `__ARM_ARCH_7M__` (GCC -mcpu=cortex-m3) cuando no se define explícitamente. Build falla sin esto. NO requiere cambio en PSoC Creator.
- [ ] **PSoC Creator**: UART_1 Baud Rate = **921600** → **Clean → Build → Flash** *(fuente ctrl_pend.c modificado a 13:26, object compilado a 12:44 → rebuild necesario)*
- [ ] **PSoC Creator**: Verificar que `isr_2` esté conectado a `UART_1` rx_interrupt en el esquemático
- [ ] **Device Manager**: COMx → Port Settings → Advanced → Latency Timer = **1ms**
- [ ] **Test hardware**: end-to-end con péndulo físico

### Mejoras futuras (visiones):
- [x] **Q31/Q15/Q7 en PSoC**: **IMPLEMENTADO** v6.3 — observer SS completo en Q31/Q15/Q7. TF usa siempre F32 (óptimo para 6 coeficientes escalares). Control law final en float (integrador vint evita overflow Q). Coeficientes pre-convertidos en ctrl_apply_coeffs. Despacho dinámico en run_step vía g_num_type.
- [ ] **F16 en PSoC**: `__fp16` requiere GCC ARM con soporte específico. Posible si se actualiza toolchain. Actualmente fallback a F32.
- [ ] **Comando 'p' en STREAM mode** (live param update sin parar): requiere buffer doble o mutex en ctrl_pend.c
- [ ] **Frame extendido**: theta_dot, vel_motor, etc. si se sube baud a 2M
- [ ] **Q-format con ganancias > 1**: actualmente los coeficientes K, L > 1 saturan al convertir a Q. Solución futura: `coeff_scale` separado en c[18]; MATLAB pre-divide K/L por coeff_scale antes de enviar, PSoC multiplica resultado por coeff_scale. Alternativamente: diseñar controlador en forma normalizada para usar Q-format con mayor fidelidad.
- [x] **Anti-windup en integrador SS** — **IMPLEMENTADO** (ver bugs corregidos)
- [x] **num_type en .mat exportado** — **IMPLEMENTADO**: `data.num_type` en onExport
- [x] **Guardado/carga de sesión con num_type** — **IMPLEMENTADO**: sess.num_type en save/load
- [x] **openCtrlPopup rediseñado** — **IMPLEMENTADO** (2026-04-05): popup ctrl ahora usa campos de texto tipo expresión MATLAB (como openSimPopup), con `evalin('base',expr)` para variables workspace, eval button inline + feedback verde/rojo por campo, auto-discretización ZOH para modelos continuos, soporte tf/zpk/ss objetos o cell {A,B,C,D}. Ver detalle abajo.

---

## Bugs conocidos / Limitaciones

- En modo TF/OL, x1i/x2i/x1o/x2o = 0 en PSoC (xhat no se actualiza en esos modos) — correcto
- Primera muestra puede tener u2=0 si outer loop no ejecutó en el primer tick (N>1)
- Q31/Q15/Q7: ganancias K, L con |valor| > 1 saturan al convertir con arm_float_to_q31. Funciona pero introduce error de representación. Usuario debe diseñar controlador con |coef| < 1 ó esperar coeff_scale (visión futura).
- Q7: solo 7 bits de señal efectivos. Para plantas reales con q_scale correcta, hay ~3-4 bits efectivos de señal → ruido de cuantización alto, esperado para modo demostrativo.

---

## Bugs corregidos en sesiones anteriores

| Bug | Fix |
|-----|-----|
| `Timeout cmd 'r'` (Start FAIL) | `uartp_reset()` envía `'s'` + flush antes de `'r'` |
| `Invalid array indexing` L1451 | Split typecast: `ref_bytes = typecast(...); write(..., [uint8('u'), ref_bytes(:)'])` |
| x1i/x2i/x1o/x2o siempre NaN | PSoC envía 32B con xhat; MATLAB decodifica bytes 17-32 |
| Botón Apply eliminado | Re-agregado como `btnSend` ("📤 Enviar params") |
| `UARTP_ControlRxPoll` doble-procesa bytes | Removido del main loop; `isr_2` es el único receptor en STREAM |
| **Observer diverge durante saturación** | `run_step()` guarda u saturado en `lp->u_out` y pasa `u_sat` al predictor y actual. Patrón de Elia `g_u_out`. |
| **Integrador sin anti-windup** | Back-calculation en PSoC: `vint += (u_sat - u) / Kx` tras saturación. Mismo en MATLAB `ss_pred_step`/`ss_act_step`. MATLAB: `loop_step` no sobreescribe `u_prev` (fijo bug que anulaba el u_sat). |
| **Race isr_2 vs ctrl_step()** | Guard `if (!g_running) { Motor_Free(); return; }` al inicio de `ctrl_step()`. Reduce ventana de carrera entre stop ISR y paso de control activo. |
| **Integrador con factor Ts incorrecto** | `ctrl_pend.c` usaba `vint += (ref-y) * g_Ts` en vez de `vint += (ref-y)`. Elia usa sin Ts: Ki absorbe el Ts en el diseño. **Corregido** a `vint += (ref-y)` en PRED_I y ACT_I. **Ki en MATLAB debe diseñarse sin Ts** (convención discreta: `u_i = Ki * Σe[k]`). |

---

## Notas de diseño

### ¿Por qué CMSIS-DSP y no operaciones escalares?
El Cortex-M3 en el PSoC 5LP no tiene FPU, pero CMSIS-DSP tiene versiones optimizadas en ensamblador para CM3. Las funciones `arm_dot_prod_f32`, `arm_scale_f32`, `arm_add_f32` mejoran la eficiencia del loop de control SS.

### ¿Por qué isr_2 en lugar de polling?
Polling en el main loop tiene latencia = Ts + tiempo de ctrl_step(). Con isr_2, el stop del motor ocurre dentro de microsegundos de recibir 's', independientemente de dónde esté el programa. Crítico para la seguridad del hardware.

### ¿Por qué 32B en lugar de 16B?
Necesario para incluir estados del observador (xhat), útiles para debug y validación.
A 921600 baud: 32B = 0.347ms UART + 1ms USB = 1.347ms de los 5ms de Ts. ✓

### openCtrlPopup (v2, 2026-04-05) — diseño de la UI
- **TF panel**: campo `Modelo TF` acepta `tf([b…],[a…],Ts)`, variable workspace (`mySys`), o `zpk`. Auto-discretiza si `Ts==0` vía ZOH. Convierte de z-descending (MATLAB) a z^-k ascending (PSoC). Valida orden ≤ 5.
- **SS panel**: campo `Modelo SS` acepta `ss(A,B,C,D,Ts)` o `{A,B,C,D}`. Valida 2 estados (PSoC hardcoded). Campos separados para `K (1×2)`, `L (2×1)`, `Ki` o `Nbar` (Ki visible si Integrador=on, Nbar si off).
- **Eval inline**: cada fila tiene botón `✔ Eval` → texto verde si OK (con info de resultado), rojo si error.
- **workspace lookup**: `evalin('base',expr)` con fallback a `eval(expr)`.
- **Apply**: valida todo al aplicar y muestra `uialert` si algún campo falla.

### Comparación con Elia control_app.c (referencia de implementación)

Revisión exhaustiva de `ctrl_pend.c` vs `FinalV1.8/control_app.c` (2026-04-06):

| Aspecto | Elia | ctrl_pend.c | Estado |
|---------|------|-------------|--------|
| TF step (DF2T orden 5) | idéntico | idéntico | ✅ igual |
| SS observer predictor | `innov=y−ŷ(xhat,u_prev); z=A·x+B·u; xhat=z+L·innov` | igual | ✅ igual |
| SS observer actual | correct+predict separados | igual | ✅ igual |
| CMSIS-DSP (dot_prod, scale, add) | `arm_dot_prod_f32` por fila | igual | ✅ igual |
| u_out tracking (saturado) | `g_u_out` post-sat | `lp->u_out` post-sat | ✅ igual |
| u_prev en observer | `g_u_out` (saturado) | `lp->u_out` (saturado) | ✅ igual |
| Ley de control: `u = Kr·r + Ki·vint − K·xhat` | igual | igual | ✅ igual |
| Integrador | `vint += e` (sin Ts) | **CORREGIDO** a `vint += e` | ✅ fix aplicado |
| Anti-windup | no tiene | `vint += (u_sat−u)/Ki` | ➕ mejora añadida |
| Bumpless start (vint0=u0/Ki) | sí | no (zero-start) | ⚠️ diferencia menor |

**Convención Ki (importante para diseño en MATLAB):**
- `vint[k] = vint[k-1] + e[k]` → acumulación de muestras (Euler sin Ts)
- `u_i = Ki · vint` → Ki es ganancia sobre la **suma discreta de errores**
- En tiempo continuo: `Ki_discreto = Ki_continuo · Ts`
- Si diseñaste `Ki_cont = 10` con `Ts = 0.005s` → `Ki_psoc = 0.05`

### ¿Por qué no usar echo-protocol para 'p'?
Echo-protocol: 54 round trips × ~2.87ms = 155ms. Demasiado lento.
Simple checksum: 219B burst + 1ms resp ≈ 20ms. Aceptable.

### Control del timing
PSoC es el master de timing (timer ISR). MATLAB ya no busy-waits.
USB latency no afecta el control (PSoC actúa sin esperar MATLAB).

---

## 💬 Sección de Comentarios del Usuario

*(Rellena aquí si notas algo raro durante el proceso)*
---

## ✅ Problemas Finalizados

| Comentario resuelto | Fix aplicado |
|---|---|
| Q31/Q15/Q7 en PSoC "para implementación futura" — paths Q del observer SS completos | `ctrl_pend.c` v6.3: inline conversion helpers (flt_to_q31n/q31_to_flt, etc.), qsat31/15/7, SS observer completo en Q31/Q15/Q7 usando arm_dot_prod/scale/add, despacho en run_step por g_num_type. MATLAB: num_cast(x, type, q_scale), per-plant sim_cast con q_scale, build_coeffs c(17)=q_scale, openCtrlPopup row q_scale, cfg_default + session retrocompat. 15 CMSIS .c copiados. |
| "Agregar 15 .c Q-format al proyecto manualmente" | Editado `4 - Pendulo.cyprj` (2026-04-06): añadidos los 15 arm_*_q31/q15/q7.c con `build_action=SOURCE_C` — PSoC Creator los compila automáticamente al abrir/rebuildar. Verificado: DSP\Include arm_math.h v1.10.0 es compatible (const-mismatch aceptado por GCC ARM 5.4.1, igual que los F32 existentes). |
| Popup controlador: expresiones MATLAB, workspace lookup, feedback verde/rojo, tf/ss objetos, K/L/Ki/Nbar separados | `openCtrlPopup` rediseñado con campos texto + `✔ Eval` inline, `evalin('base',expr)`, ZOH auto, validación |
| `isr_1.h`/`isr_2.h` no hace falta importar, vienen en `project.h` | Eliminado `#include "isr_2.h"` de `uartp_pend.c` |
| CMSIS incompatible con PSoC Creator 4.4 / GCC ARM 5.4 | Reemplazados todos los headers y .c con CMSIS_4-4.5.0 (V1.4.5b). Eliminados arm_math_types/memory/compiler_specific.h y dsp/. Copiados core_cm3.h + auxiliares. Nota de compilación en ctrl_pend.c actualizada. |
| Eliminar botón "Enviar params" — todo por Start | `btnSend` y `onSendParams()` eliminados |
| openSimPopup: aplicar mismo estilo que ctrl popup | `openSimPopup` rediseñado: acepta `tf(...)`, `zpk(...)`, `ss(...)` o variable workspace; `evalin`+fallback; `✔ Eval` inline con feedback; auto-discretización ZOH; una expresión por panel |
| ŷ₁/ŷ₂ checkboxes visibles aunque sim esté desactivado | `cbSigSim1`/`cbSigSim2` inician como `Visible=false`; `onSimToggle` sincroniza visibilidad con estado de `cbSimEn` |
| Ventana escaladores: botón superpuesto a x̂₂ₒ×; altura incorrecta | `openScalerPopup`: SH calculado dinámicamente (`NS*ROW_STEP + margen`) para garantizar que el último campo nunca solape con los botones |
| Curvas/leyendas colgadas cuando ambas plantas Off | `updateAxesLayout` limpia XData/YData de todas las líneas en el caso `else` (ambas Off) |
| Un planta Off: axes no llenan bien la pantalla | `AH2` calculado desde `AX_TOP` real en lugar de `2*AX_H+AX_GAP` fijo; axes llenan todo el espacio disponible |
| Start: si hubo cambios → enviar params → verificar eco → arrancar | `uartp_send_params(force)`: compara payload con `S.last_sent_payload`; si igual → skip. Si distinto → envía `'p'` + `'K'` + `'v'` + lee eco 217B + verifica byte a byte |
| PSoC debe ecoar lo recibido antes de arrancar | Nuevo comando `'v'` en PSoC: responde con 216B de `s_p_buf` + xsum XOR acumulado |
| No reenviar si no hubo cambios | `S.last_sent_payload` guardado tras envío verificado; comparación en cada Start de inicio (no en Send ref) |
| Botón Start → "📤 Send ref" (azul) cuando control corre | `btnStart` cambia texto/color al iniciar stream; se restaura a verde al detenerse o desconectar |
| `onLoadSession` no sincronizaba ŷ checkboxes al cargar sesión con sim activo | Añadido `for pp2=1:2, onSimToggle(pp2); end` al final del bloque de carga, después de `onModeChange` |
| Integrador PSoC usaba `vint += e * Ts` — convención distinta a Elia | Corregido a `vint += e` (sin Ts) en `PRED_I` y `ACT_I`. Ki en MATLAB debe ser `Ki_cont * Ts`. Ver nota de diseño "Comparación con Elia". |
| MATLAB `ss_pred_step`/`ss_act_step` también usaban `* Ts` | Corregido a `vint += (ref-y)` (sin Ts). Añadido `%#ok<INUSL>` pues son funciones no-llamadas (v6: PSoC controla). |
| Comentarios stale en pendulo_gui2.m ("16B/tick", "16 bytes") | Actualizados a "32B/tick" y "32 bytes (8×float32)". Header actualizado a v6.2. |
| Entrada stale `btnSend` en Estado actual del .md | Actualizado a tachado con nota "eliminado; todo vía Start" |
