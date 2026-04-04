#include "uartp_sw.h"

/* ================= globals ================= */
volatile uartp_sysmode_t    UARTP_SysMode   = UARTP_SYS_COMMAND;
volatile uartp_ctrl_state_t UARTP_CtrlState = UARTP_CTRL_STOPPED;

volatile uartp_impl_t UARTP_Impl        = UARTP_IMPL_TF;
volatile uint8        UARTP_ModeValid   = 0u;
volatile uint8        UARTP_CoeffsValid = 0u;

volatile uint8 UARTP_StopRequested = 0u;
volatile uint8 UARTP_LastCmd = 0u;

float UARTP_Coeffs[UARTP_COEF_COUNT];
float UARTP_InitValue = 0.0f;

volatile uint16 UARTP_StreamN     = 0u;
volatile uint16 UARTP_TimerPeriod = 0u;

static uartp_cfg_t g_cfg;

/* NUEVO: update 'i' en CONTROL */
static volatile uint8  g_init_update_pending = 0u;
static uint8           g_init_update_raw[UARTP_INIT_BYTES];

/* ================= helpers ================= */
static void flush_rx(void)
{
    while (UARTP_UART_GetRxBufferSize() > 0u) (void)UARTP_UART_ReadRxData();
}

/* ---- helpers para “leer bits” de float ---- */
typedef union { float f; uint32 u32; } f32u_t;

static uint16 u16_from_float(float x)
{
    if (x <= 0.0f) return 0u;
    if (x >= 65535.0f) return 65535u;
    return (uint16)(x + 0.5f); // round
}

/* ---- timer oneshot timeout (sin CyDelay) ----
   Supone timer_uart clock 1kHz => period=ms
*/
static void uart_timer_start_ms(uint16 ms)
{
    if (ms == 0u) ms = 1u;

    UARTP_TIMER_UART_Stop();
    (void)UARTP_TIMER_UART_ReadStatusRegister(); /* limpia flags latched */
    UARTP_TIMER_UART_WritePeriod(ms);
    UARTP_TIMER_UART_Start();
}

static bool uart_timer_expired(void)
{
    uint8 st = (uint8)UARTP_TIMER_UART_ReadStatusRegister();
    return ((st & UARTP_TIMER_UART_STATUS_TC) != 0u);
}

/* timeout sin delay bloqueante: poll RX + timer */
static bool read_byte_timeout(uint8* out, uint16 timeout_ms)
{
    uart_timer_start_ms(timeout_ms);

    while (!uart_timer_expired())
    {
        if (UARTP_UART_GetRxBufferSize() > 0u) {
            *out = UARTP_UART_ReadRxData();
            UARTP_TIMER_UART_Stop();
            (void)UARTP_TIMER_UART_ReadStatusRegister();
            return true;
        }
        /* sin CyDelay: el loop corre rápido.
           Si querés bajar carga CPU, podemos meter un NOP o WFI,
           pero primero dejémoslo así.
        */
    }

    UARTP_TIMER_UART_Stop();
    (void)UARTP_TIMER_UART_ReadStatusRegister();
    return false;
}

static bool read_n_timeout(uint8* out, uint16 n, uint16 timeout_ms_each)
{
    for (uint16 i=0u; i<n; i++) {
        if (!read_byte_timeout(&out[i], timeout_ms_each)) return false;
    }
    return true;
}

static void write_byte(uint8 b){ UARTP_UART_PutChar(b); }
static void write_n(const uint8* p, uint16 n){ UARTP_UART_PutArray(p,n); }

/* ===== eco-confirmado RX word ===== */
static bool rx_word_try(uint8* dst4_or_partial, uint16 bytes_to_store, bool* did_timeout)
{
    *did_timeout = false;
    uint8 w[4];
    uint8 ctl;

    if (!read_n_timeout(w, 4u, UARTP_STEP_TIMEOUT_MS)) {
        *did_timeout = true;
        return false;
    }

    /* eco */
    write_n(w, 4u);

    if (!read_byte_timeout(&ctl, UARTP_STEP_TIMEOUT_MS)) {
        *did_timeout = true;
        return false;
    }

    if (ctl == UARTP_CTL_ACK) {
        for (uint16 i=0u; i<bytes_to_store; i++) dst4_or_partial[i] = w[i];
        write_byte(UARTP_CTL_ACK);
        return true;
    }

    write_byte(UARTP_CTL_NAK);
    return false;
}

static bool recv_payload(uint8* dst, uint16 nbytes)
{
    uint16 off = 0u;
    uint8 timeouts_in_row = 0u;

    while (off < nbytes)
    {
        uint16 rem   = (uint16)(nbytes - off);
        uint16 store = (rem >= 4u) ? 4u : rem;

        uint16 tries = 0u;
        for (;;)
        {
            bool did_timeout = false;
            bool ok = rx_word_try(&dst[off], store, &did_timeout);

            if (ok) { off = (uint16)(off + store); timeouts_in_row = 0u; break; }

            if (did_timeout) {
                if (++timeouts_in_row >= UARTP_ABORT_TIMEOUTS) { flush_rx(); return false; }
            }
            if (++tries >= UARTP_MAX_WORD_RETRIES) { flush_rx(); return false; }
        }
    }
    return true;
}

/* ===== eco-confirmado TX word ===== */
static bool tx_word_once(const uint8 w[4], bool* accepted, bool* did_timeout)
{
    *did_timeout = false;
    uint8 echo[4];
    uint8 host_confirm;

    write_n(w, 4u);

    if (!read_n_timeout(echo, 4u, UARTP_STEP_TIMEOUT_MS)) {
        *did_timeout = true;
        return false;
    }

    bool match = (memcmp(echo, w, 4u) == 0);
    write_byte(match ? UARTP_CTL_ACK : UARTP_CTL_NAK);

    if (!read_byte_timeout(&host_confirm, UARTP_STEP_TIMEOUT_MS)) {
        *did_timeout = true;
        return false;
    }

    if (host_confirm != (match ? UARTP_CTL_ACK : UARTP_CTL_NAK)) {
        *accepted = false;
        return false;
    }

    *accepted = match;
    return true;
}

static bool send_payload(const uint8* src, uint16 nbytes)
{
    uint16 off = 0u;
    uint8 timeouts_in_row = 0u;

    while (off < nbytes)
    {
        uint8 w[4] = {0,0,0,0};
        uint16 rem  = (uint16)(nbytes - off);
        uint16 take = (rem >= 4u) ? 4u : rem;

        for (uint16 i=0u; i<take; i++) w[i] = src[off+i];

        uint16 tries = 0u;
        for (;;)
        {
            bool accepted = false;
            bool did_timeout = false;

            bool ok = tx_word_once(w, &accepted, &did_timeout);
            if (ok && accepted) { off = (uint16)(off + take); timeouts_in_row = 0u; break; }

            if (did_timeout) {
                if (++timeouts_in_row >= UARTP_ABORT_TIMEOUTS) { flush_rx(); return false; }
            }
            if (++tries >= UARTP_MAX_WORD_RETRIES) { flush_rx(); return false; }
        }
    }
    return true;
}

/* ================= RX ISR: STOP y INIT en CONTROL =================
   - 's' : como antes
   - 'i' : responde 'R', deshabilita ISR y deja pendiente recibir 4B en foreground
*/
CY_ISR(UARTP_RxIsr)
{
    while (UARTP_UART_GetRxBufferSize() > 0u)
    {
        uint8 b = UARTP_UART_ReadRxData();

        if (b == UARTP_CMD_STOP) {
            UARTP_StopRequested = 1u;
            if (UARTP_CtrlState == UARTP_CTRL_RUNNING) UARTP_CtrlState = UARTP_CTRL_STOPPING;
            write_byte(UARTP_RSP_OK);
            UARTP_ISR_RX_Disable(); /* deja de interceptar bytes */
        }
        else if (b == UARTP_CMD_INIT) {
            /* Permitimos update de u0/ref en CONTROL */
            if (UARTP_CtrlState == UARTP_CTRL_RUNNING) {
                g_init_update_pending = 1u;
                write_byte(UARTP_RSP_READY_RX);  /* host empieza a mandar payload (4B) */
                UARTP_ISR_RX_Disable();          /* para no comer bytes del payload */
            } else {
                write_byte(UARTP_RSP_ERR);
            }
        }
        else {
            /* en CONTROL rechazamos todo lo demás */
            write_byte(UARTP_RSP_ERR);
        }
    }
    (void)UARTP_UART_ReadRxStatus();
}

/* ================= transitions ================= */
void UARTP_EnterCommandMode(void)
{
    UARTP_ISR_RX_Disable();
    UARTP_SysMode = UARTP_SYS_COMMAND;
}

void UARTP_EnterControlMode(void)
{
    UARTP_StopRequested = 0u;
    UARTP_SysMode = UARTP_SYS_CONTROL;
    UARTP_CtrlState = UARTP_CTRL_RUNNING;
    UARTP_ISR_RX_Enable();
}

/* ================= public ================= */
void UARTP_Init(const uartp_cfg_t* cfg)
{
    UARTP_UART_Start();
    UARTP_UART_ClearRxBuffer();
    UARTP_UART_ClearTxBuffer();
    flush_rx();

    /* Arrancamos timer_uart detenido */
    UARTP_TIMER_UART_Stop();
    (void)UARTP_TIMER_UART_ReadStatusRegister();

    memset(&g_cfg, 0, sizeof(g_cfg));
    if (cfg) g_cfg = *cfg;

    UARTP_SysMode = UARTP_SYS_COMMAND;
    UARTP_CtrlState = UARTP_CTRL_STOPPED;

    UARTP_Impl = UARTP_IMPL_TF;
    UARTP_ModeValid = 0u;
    UARTP_CoeffsValid = 0u;

    UARTP_StopRequested = 0u;
    UARTP_LastCmd = 0u;

    memset((void*)UARTP_Coeffs, 0, sizeof(UARTP_Coeffs));
    UARTP_InitValue = 0.0f;

    UARTP_StreamN = 0u;
    UARTP_TimerPeriod = 0u;

    g_init_update_pending = 0u;
    memset((void*)g_init_update_raw, 0, sizeof(g_init_update_raw));

    UARTP_ISR_RX_StartEx(UARTP_RxIsr);
    UARTP_ISR_RX_Disable();
}

bool UARTP_ProcessOnce(void)
{
    if (UARTP_SysMode != UARTP_SYS_COMMAND) return false;
    if (UARTP_UART_GetRxBufferSize() == 0u) return false;

    uint8 cmd = UARTP_UART_ReadRxData();
    UARTP_LastCmd = cmd;

    /* si no está STOPPED, en COMMAND solo aceptamos stop (idempotente) */
    if (UARTP_CtrlState != UARTP_CTRL_STOPPED) {
        if (cmd == UARTP_CMD_STOP) { write_byte(UARTP_RSP_OK); return true; }
        write_byte(UARTP_RSP_ERR);
        return true;
    }

    switch (cmd)
    {
        case UARTP_CMD_RESET:
            write_byte(UARTP_RSP_OK);
            /* NO CyDelay acá: igual el reset mata todo enseguida */
            CySoftwareReset();
            return true;

        case UARTP_CMD_STOP:
            write_byte(UARTP_RSP_OK);
            return true;

        case UARTP_CMD_SETMODE:
        {
            write_byte(UARTP_RSP_READY_RX);

            uint8 raw[UARTP_MODE_BYTES];
            if (!recv_payload(raw, (uint16)UARTP_MODE_BYTES)) { write_byte(UARTP_RSP_ERR); return true; }

            uint8 mode = raw[0];
            if (mode > 5u) { write_byte(UARTP_RSP_ERR); return true; }

            UARTP_Impl = (uartp_impl_t)mode;
            UARTP_ModeValid = 1u;

            write_byte(UARTP_RSP_OK);
            return true;
        }

        case UARTP_CMD_COEFFS:
        {
            if (UARTP_ModeValid == 0u) { write_byte(UARTP_RSP_ERR); return true; }

            write_byte(UARTP_RSP_READY_RX);

            uint8 raw[UARTP_COEF_BYTES];
            if (!recv_payload(raw, (uint16)UARTP_COEF_BYTES)) { write_byte(UARTP_RSP_ERR); return true; }

            memcpy((void*)UARTP_Coeffs, raw, UARTP_COEF_BYTES);
            UARTP_CoeffsValid = 1u;

            /* c15..c16 => UARTP_Coeffs[14], [15] */
            UARTP_StreamN     = u16_from_float(UARTP_Coeffs[14]);
            UARTP_TimerPeriod = u16_from_float(UARTP_Coeffs[15]);

            if (UARTP_TimerPeriod != 0u) {
                UARTP_TIMER_SAMPLING_WritePeriod(UARTP_TimerPeriod);
            }

            if (UARTP_Impl == UARTP_IMPL_TF) {
                if (g_cfg.apply_tf) g_cfg.apply_tf((const float*)UARTP_Coeffs, UARTP_COEF_COUNT);
            } else if (UARTP_Impl == UARTP_IMPL_OPENLOOP) {
                /* nada */
            } else {
                if (g_cfg.apply_ss) g_cfg.apply_ss((const float*)UARTP_Coeffs, UARTP_COEF_COUNT);
            }

            write_byte(UARTP_RSP_OK);
            return true;
        }

        case UARTP_CMD_TXCOEF:
        {
            if (UARTP_CoeffsValid == 0u) { write_byte(UARTP_RSP_ERR); return true; }

            write_byte(UARTP_RSP_READY_TX);

            if (!send_payload((const uint8*)UARTP_Coeffs, (uint16)UARTP_COEF_BYTES)) { write_byte(UARTP_RSP_ERR); return true; }

            write_byte(UARTP_RSP_OK);
            return true;
        }

        case UARTP_CMD_INIT:
        {
            if (UARTP_ModeValid == 0u)   { write_byte(UARTP_RSP_ERR); return true; }
            if (UARTP_CoeffsValid == 0u) { write_byte(UARTP_RSP_ERR); return true; }

            write_byte(UARTP_RSP_READY_RX);

            uint8 raw[UARTP_INIT_BYTES];
            if (!recv_payload(raw, (uint16)UARTP_INIT_BYTES)) { write_byte(UARTP_RSP_ERR); return true; }

            memcpy((void*)&UARTP_InitValue, raw, 4u);

            if (g_cfg.start) g_cfg.start(UARTP_InitValue);

            write_byte(UARTP_RSP_OK);

            UARTP_EnterControlMode();
            return true;
        }

        default:
            flush_rx();
            write_byte(UARTP_RSP_ERR);
            return true;
    }
}

void UARTP_ControlTick(void)
{
    if (UARTP_SysMode != UARTP_SYS_CONTROL) return;

    /* 1) Si hay un init update pendiente (vino 'i' por ISR) */
    if (g_init_update_pending)
    {
        /* recibimos 4 bytes con el MISMO protocolo robusto */
        bool ok = recv_payload((uint8*)g_init_update_raw, (uint16)UARTP_INIT_BYTES);
        if (ok)
        {
            memcpy((void*)&UARTP_InitValue, (const void*)g_init_update_raw, 4u);

            /* Re-usamos el callback start como “update” */
            if (g_cfg.start) g_cfg.start(UARTP_InitValue);

            write_byte(UARTP_RSP_OK);
        }
        else {
            write_byte(UARTP_RSP_ERR);
            flush_rx();
        }

        g_init_update_pending = 0u;
        UARTP_ISR_RX_Enable(); /* volvemos a aceptar s/i por ISR */
    }

    /* 2) stop suave */
    if (UARTP_CtrlState == UARTP_CTRL_STOPPING)
    {
        bool done = true;
        if (g_cfg.stop_step) done = g_cfg.stop_step();

        if (done) {
            UARTP_CtrlState = UARTP_CTRL_STOPPED;
            UARTP_SysMode = UARTP_SYS_COMMAND;
            UARTP_ISR_RX_Disable();
            flush_rx();
        }
    }
}
