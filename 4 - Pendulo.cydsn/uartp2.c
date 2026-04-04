/* ============================================================
   uartp2.c — Two-plant UARTP protocol implementation
   ============================================================ */
#include "uartp2.h"
#include "ctrl_pend.h"
#include <string.h>

volatile uartp2_mode_t UARTP2_Mode = UARTP2_CMD;

/* ============================================================
   Protocol constants
   ============================================================ */
#define STEP_TMO_MS   500u   /* per-byte timeout */
#define MAX_RETRIES    50u   /* per word */
#define ABORT_TMO       3u   /* consecutive timeouts before abort */

/* ============================================================
   Low-level byte I/O
   ============================================================ */
static void ll_put(uint8 b)
{
    UART_1_WriteTxData(b);
}

static void ll_flush(void)
{
    while (UART_1_GetRxBufferSize() > 0u) (void)UART_1_ReadRxData();
}

/* Wait for one RX byte; returns 0xFF on timeout */
static uint8 ll_wait(uint32 tmo_ms)
{
    uint32 t = 0u;
    while (UART_1_GetRxBufferSize() == 0u) {
        CyDelay(1u);
        if (++t >= tmo_ms) return 0xFFu;
    }
    return UART_1_ReadRxData();
}

/* ============================================================
   rx_word — receive one 4-byte word with echo-confirm.
   Returns 1 if ACK'd, 0 on timeout/error.
   ============================================================ */
static uint8 rx_word(uint8 dst[4])
{
    uint8 tries = 0u;
    for (;;) {
        uint8 w[4]; uint8 i; uint8 ctl;
        if (++tries > MAX_RETRIES) return 0u;

        for (i = 0u; i < 4u; i++) {
            w[i] = ll_wait(STEP_TMO_MS);
            if (w[i] == 0xFFu) return 0u;   /* byte timeout */
        }
        for (i = 0u; i < 4u; i++) ll_put(w[i]);  /* echo */

        ctl = ll_wait(STEP_TMO_MS);
        if (ctl == 0xFFu) return 0u;

        if (ctl == (uint8)'A') {
            dst[0]=w[0]; dst[1]=w[1]; dst[2]=w[2]; dst[3]=w[3];
            ll_put((uint8)'A');
            return 1u;
        }
        ll_put((uint8)'N');   /* NAK → host will retry */
    }
}

/* ============================================================
   recv_payload — receive nbytes via word-by-word handshake.
   Returns 1 on success, 0 on abort.
   ============================================================ */
static uint8 recv_payload(uint8 *dst, uint16 nbytes)
{
    uint16 off = 0u;
    uint8  tmo_streak = 0u;

    while (off < nbytes) {
        uint8  tmp[4] = {0u, 0u, 0u, 0u};
        uint16 store  = (uint16)(nbytes - off);
        if (store > 4u) store = 4u;

        if (!rx_word(tmp)) {
            if (++tmo_streak >= ABORT_TMO) { ll_flush(); return 0u; }
            continue;
        }
        tmo_streak = 0u;

        uint16 i;
        for (i = 0u; i < store; i++) dst[off + i] = tmp[i];
        off = (uint16)(off + store);
    }
    return 1u;
}

/* ============================================================
   Command handlers
   ============================================================ */
static void handle_setmode(uint8 plant_id)
{
    ll_put((uint8)'R');
    uint8 raw[4];
    if (!recv_payload(raw, 4u)) { ll_put((uint8)'!'); return; }
    ctrl_set_mode(plant_id, raw[0]);
    ll_put((uint8)'K');
}

static void handle_coeffs(uint8 plant_id)
{
    ll_put((uint8)'R');
    uint8 raw[100];
    if (!recv_payload(raw, 100u)) { ll_put((uint8)'!'); return; }
    float c[25];
    memcpy(c, raw, 100u);
    ctrl_apply_coeffs(plant_id, c, 25u);
    ll_put((uint8)'K');
}

static void handle_start(void)
{
    ll_put((uint8)'R');
    uint8 raw[4];
    if (!recv_payload(raw, 4u)) { ll_put((uint8)'!'); return; }
    float ref0;
    memcpy(&ref0, raw, 4u);
    ctrl_start(ref0);
    ll_put((uint8)'K');
    UARTP2_Mode = UARTP2_CTRL;
}

/* ============================================================
   UARTP2_Init()
   ============================================================ */
void UARTP2_Init(void)
{
    UART_1_Start();
    ll_flush();
    UARTP2_Mode = UARTP2_CMD;
}

/* ============================================================
   UARTP2_ProcessOnce()  — call from COMMAND loop
   ============================================================ */
void UARTP2_ProcessOnce(void)
{
    if (UART_1_GetRxBufferSize() == 0u) return;
    uint8 cmd = UART_1_ReadRxData();

    switch (cmd)
    {
        case (uint8)'r':                          /* reset / stop */
            ctrl_stop();
            ll_flush();
            UARTP2_Mode = UARTP2_CMD;
            ll_put((uint8)'K');
            break;

        case (uint8)'s':                          /* stop */
            ctrl_stop();
            UARTP2_Mode = UARTP2_CMD;
            ll_put((uint8)'K');
            break;

        case (uint8)'1':                          /* set mode plant 0 (inner) */
            handle_setmode(PLANT_INNER);
            break;

        case (uint8)'2':                          /* set mode plant 1 (outer) */
            handle_setmode(PLANT_OUTER);
            break;

        case (uint8)'a':                          /* coefficients plant 0 */
            handle_coeffs(PLANT_INNER);
            break;

        case (uint8)'b':                          /* coefficients plant 1 */
            handle_coeffs(PLANT_OUTER);
            break;

        case (uint8)'i':                          /* start control */
            handle_start();
            break;

        default:
            ll_put((uint8)'!');
            break;
    }
}
