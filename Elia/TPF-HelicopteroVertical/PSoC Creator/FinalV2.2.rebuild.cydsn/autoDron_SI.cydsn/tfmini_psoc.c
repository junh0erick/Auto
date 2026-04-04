#include "tfmini_psoc.h"
#include "tfm_calib_simple.h"

#include <string.h>

volatile uint8_t tfmini_sample_pending = 0u;

/* Parser state */
static volatile uint8_t  s_frame[TFMINI_FRAME_SIZE];
static volatile uint8_t  s_state = 0u; /* 0:wait H1, 1:wait H2, 2:collect */
static volatile uint8_t  s_idx   = 0u;

/* Última distancia calibrada (cm) */
static volatile uint16_t s_last_cm = 0u;

/* ===== TFMini Plus protocol ===== */
#define TF_CMD_HDR   (0x5Au)
#define TF_DATA_HDR  (0x59u)

#define CMD_SET_RATE (0x03u) /* payload: rate (uint16 LE) */
#define CMD_OUTPUT   (0x07u) /* payload: 0 disable, 1 enable */

/* =========================
   Helpers
   ========================= */
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
    (void)uart_TFminiPlus_ReadRxStatus();
}

static void rx_flush_and_reset_locked(void)
{
    uint8 intr = CyEnterCriticalSection();
    rx_flush_nolock();
    parser_reset();
    tfmini_sample_pending = 0u;
    isr_rx_TFminiPlus_ClearPending();
    CyExitCriticalSection(intr);
}

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

    rx_flush_and_reset_locked();
    for (i = 0u; i < len; i++) uart_TFminiPlus_PutChar(buf[i]);

    CyDelay(2u);
    rx_flush_and_reset_locked();

    return true;
}

/* =========================
   Calibración: SOLO dist
   ========================= */
uint16_t tfmini_calibrate_cm(uint16_t dist_cm)
{
    float y = tfmini_correct_distance_cm_simple((float)dist_cm);

    /* guardas mínimas */
    if (!(y == y)) y = 0.0f;          /* NaN */
    if (y < 0.0f) y = 0.0f;
    if (y > 65535.0f) y = 65535.0f;

    return (uint16_t)(y + 0.5f);
}

/* =========================
   Frame handler
   ========================= */
static void on_frame(const uint8_t* f)
{
    if (sum8_first8(f) != f[8]) return;

    /* dist = bytes[2..3] little-endian */
    uint16_t dist = (uint16_t)f[2] | ((uint16_t)f[3] << 8);

    dist = tfmini_calibrate_cm(dist);

    /* saturación a rango útil (cm) */
    if (dist < TFMINI_MIN_CM) dist = TFMINI_MIN_CM;
    else if (dist > TFMINI_MAX_CM) dist = TFMINI_MAX_CM;

    s_last_cm = dist;
    tfmini_sample_pending = 1u;
}

/* =========================
   Byte parser
   ========================= */
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

    /* collect */
    s_frame[s_idx] = b;
    s_idx++;

    if (s_idx >= TFMINI_FRAME_SIZE) {
        uint8_t fcopy[TFMINI_FRAME_SIZE];
        uint8_t i;
        for (i = 0u; i < TFMINI_FRAME_SIZE; i++) fcopy[i] = s_frame[i];
        on_frame(fcopy);
        parser_reset();
    }
}

/* =========================
   ISR RX
   ========================= */
CY_ISR(isr_rx_tfmini_handler)
{
    isr_rx_TFminiPlus_ClearPending();

#if defined(uart_TFminiPlus_RX_STS_FIFO_NOTEMPTY)
    for (;;) {
        uint8_t st = uart_TFminiPlus_ReadRxStatus();

    #if defined(uart_TFminiPlus_RX_STS_OVERRUN)
        if (st & uart_TFminiPlus_RX_STS_OVERRUN) {
            rx_flush_nolock();        /* ya estás en ISR */
            parser_reset();
            tfmini_sample_pending = 0u;
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

/* =========================
   API pública
   ========================= */
void tfmini_init(void)
{
    parser_reset();
    tfmini_sample_pending = 0u;
    s_last_cm = 0u;

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

    return send_cmd(CMD_SET_RATE, p, 2u);
}

bool tfmini_pop_cm(uint16_t *y_cm)
{
    if (!y_cm) return false;

    uint8 intr = CyEnterCriticalSection();
    if (tfmini_sample_pending == 0u) {
        CyExitCriticalSection(intr);
        return false;
    }

    *y_cm = (uint16_t)s_last_cm;
    tfmini_sample_pending = 0u;
    CyExitCriticalSection(intr);

    return true;
}

bool tfmini_pop_m(float *y_m)
{
    if (!y_m) return false;

    uint8 intr = CyEnterCriticalSection();
    if (tfmini_sample_pending == 0u) {
        CyExitCriticalSection(intr);
        return false;
    }

    uint16_t cm = (uint16_t)s_last_cm;
    tfmini_sample_pending = 0u;
    CyExitCriticalSection(intr);

    *y_m = ((float)cm) * TFMINI_CM_TO_M;
    return true;
}
