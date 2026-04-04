#include "tfmini_psoc.h"
#include <string.h>

/* Tu modelo exportado (si lo usás) */
#include "tfmini_calib_coeffs.h"   /* tfmini_correct_distance_cm(...) */

volatile uint8_t tfmini_sample_pending = 0u;

/* ===== Comandos TFMini Plus ===== */
#define TF_CMD_HDR   (0x5Au)
#define TF_DATA_HDR  (0x59u)

#define CMD_SET_RATE (0x03u) /* payload: rate (uint16 LE) */
#define CMD_OUTPUT   (0x07u) /* payload: 0 disable, 1 enable */
#define CMD_SAVE     (0x11u) /* no payload */

static tfmini_data_t       s_last;
static volatile uint8_t    s_frame[TFMINI_FRAME_SIZE];
static volatile uint8_t    s_state = 0u; /* 0:wait H1, 1:wait H2, 2:collect */
static volatile uint8_t    s_idx = 0u;

static uint16_t s_min_cm = TFMINI_MIN_DIST_CM;
static uint16_t s_max_cm = TFMINI_MAX_DIST_CM;
static uint16_t s_fps_hz = 100u;

static void parser_reset(void)
{
    s_state = 0u;
    s_idx   = 0u;
}

static uint8_t sum8_first8(const uint8_t* p9)
{
    uint16_t s = 0u;
    uint8_t i;
    for (i = 0u; i < 8u; i++) s += p9[i];
    return (uint8_t)s;
}

static void rx_flush_nolock(void)
{
    while (uart_TFminiPlus_GetRxBufferSize() > 0u) {
        (void)uart_TFminiPlus_ReadRxData();
    }
    (void)uart_TFminiPlus_ReadRxStatus(); /* limpia flags sticky */
}

static void rx_flush_and_reset_locked(void)
{
    uint8 intr = CyEnterCriticalSection(); /* corta TODAS las IRQ -> ISR no corre */
    rx_flush_nolock();
    parser_reset();
    tfmini_sample_pending = 0u;
    isr_rx_TFminiPlus_ClearPending();
    CyExitCriticalSection(intr);
}

/* 0x5A Len ID payload... checksum(sum low8) */
static bool send_cmd(uint8_t id, const uint8_t* payload, uint8_t plen)
{
    uint8_t  len;
    uint8_t  buf[8];
    uint16_t s;
    uint8_t  i;

    len = (uint8_t)(3u + plen + 1u);
    if (len > 8u) return false;

    buf[0] = TF_CMD_HDR;
    buf[1] = len;
    buf[2] = id;

    for (i = 0u; i < plen; i++) buf[3u + i] = payload[i];

    s = 0u;
    for (i = 0u; i < (uint8_t)(len - 1u); i++) s += buf[i];
    buf[len - 1u] = (uint8_t)s;

    /* flush + reset atómico (ISR no corre) */
    rx_flush_and_reset_locked();

    for (i = 0u; i < len; i++) uart_TFminiPlus_PutChar(buf[i]);

    /* opcional: limpiar basura/respuestas */
    CyDelay(2u);
    rx_flush_and_reset_locked();

    return true;
}

static void on_frame(const uint8_t* f)
{
    uint16_t dist;
    uint16_t str;
    int16_t  temp_raw;
    uint8_t  sum;

    sum = sum8_first8(f);
    if (sum != f[8]) {
        return; /* frame inválido */
    }

    dist     = (uint16_t)f[2] | ((uint16_t)f[3] << 8);
    str      = (uint16_t)f[4] | ((uint16_t)f[5] << 8);
    temp_raw = (int16_t)((uint16_t)f[6] | ((uint16_t)f[7] << 8));

    /* calibración (tu función) */
    dist = tfmini_calibrate_cm(dist, str, s_fps_hz, temp_raw);

    /* clamp final */
    if (dist < s_min_cm) dist = s_min_cm;
    if (dist > s_max_cm) dist = s_max_cm;

    s_last.dist_cm   = dist;
    s_last.strength  = str;
    s_last.temp_raw  = temp_raw;
    s_last.valid     = 1u;

    tfmini_sample_pending = 1u;
}

static void parser_push(uint8_t b)
{
    if (s_state == 0u) {
        if (b == TF_DATA_HDR) {
            s_frame[0] = b;
            s_state = 1u;
        }
        return;
    }

    if (s_state == 1u) {
        if (b == TF_DATA_HDR) {
            s_frame[1] = b;
            s_idx = 2u;
            s_state = 2u;
        } else {
            s_state = 0u;
        }
        return;
    }

    /* s_state == 2u: collect */
    s_frame[s_idx] = b;
    s_idx++;

    if (s_idx >= TFMINI_FRAME_SIZE) {
        uint8_t fcopy[TFMINI_FRAME_SIZE];
        uint8_t i;
        for (i = 0u; i < TFMINI_FRAME_SIZE; i++) fcopy[i] = (uint8_t)s_frame[i];
        on_frame(fcopy);
        parser_reset();
    }
}

/* ISR por RX On Byte Received */
CY_ISR(isr_rx_tfmini_handler)
{
    isr_rx_TFminiPlus_ClearPending();

#if defined(uart_TFminiPlus_RX_STS_FIFO_NOTEMPTY)
    for (;;) {
        uint8_t st = uart_TFminiPlus_ReadRxStatus();

    #if defined(uart_TFminiPlus_RX_STS_OVERRUN)
        if (st & uart_TFminiPlus_RX_STS_OVERRUN) {
            rx_flush_and_reset_locked();
            break;
        }
    #endif

        if (st & uart_TFminiPlus_RX_STS_FIFO_NOTEMPTY) {
            uint8_t b = (uint8_t)uart_TFminiPlus_ReadRxData();
            parser_push(b);
            continue;
        }
        break;
    }
#else
    while (uart_TFminiPlus_GetRxBufferSize() > 0u) {
        uint8_t b = (uint8_t)uart_TFminiPlus_ReadRxData();
        parser_push(b);
    }
    (void)uart_TFminiPlus_ReadRxStatus();
#endif
}

void tfmini_init(void)
{
    memset(&s_last, 0, sizeof(s_last));
    parser_reset();
    tfmini_sample_pending = 0u;

    uart_TFminiPlus_Start();
    rx_flush_and_reset_locked();

    isr_rx_TFminiPlus_StartEx(isr_rx_tfmini_handler);
}

void tfmini_clear_flags(void)
{
    rx_flush_and_reset_locked();
}

bool tfmini_enable(void)
{
    uint8_t p = 1u;
    return send_cmd(CMD_OUTPUT, &p, 1u);
}

bool tfmini_disable(void)
{
    uint8_t p = 0u;
    return send_cmd(CMD_OUTPUT, &p, 1u);
}

bool tfmini_set_fps(uint16_t fps_hz)
{
    uint8_t p[2];

    if (fps_hz < 1u)    fps_hz = 1u;
    if (fps_hz > 1000u) fps_hz = 1000u;

    p[0] = (uint8_t)(fps_hz & 0xFFu);
    p[1] = (uint8_t)((fps_hz >> 8) & 0xFFu);

    if (!send_cmd(CMD_SET_RATE, p, 2u)) return false;
    s_fps_hz = fps_hz;
    return true;
}

void tfmini_set_range_cm(uint16_t min_cm, uint16_t max_cm)
{
    if (min_cm <= max_cm) {
        s_min_cm = min_cm;
        s_max_cm = max_cm;
    } else {
        s_min_cm = max_cm;
        s_max_cm = min_cm;
    }
}

uint8_t tfmini_get(tfmini_data_t* out)
{
    uint8_t intr;

    if (!out) return 0u;

    intr = CyEnterCriticalSection();
    *out = s_last;
    CyExitCriticalSection(intr);

    return out->valid;
}

/* =========================================================
   Calibración (la única parte “editable”)
   ========================================================= */
uint16_t tfmini_calibrate_cm(uint16_t dist_cm, uint16_t strength, uint16_t fps_hz, int16_t temp_raw)
{
    /* raw -> décimas de °C -> float °C */
    int16_t tc10 = tfmini_temp_c10_from_raw(temp_raw);
    float32_t temp_C = ((float32_t)tc10) * 0.1f;

    /* OJO: tu modelo “best” usaba (Dist_cm, Freq_prom, Temp_C, Lux/intensidad)
       Acá pasamos strength como “Lux” (intensidad) */
    float32_t y = tfmini_correct_distance_cm((float32_t)dist_cm,
                                             (float32_t)fps_hz,
                                             temp_C,
                                             (uint16_t)strength);

    if (y < 0.0f) y = 0.0f;
    if (y > 65535.0f) y = 65535.0f;

    return (uint16_t)(y + 0.5f);
}
