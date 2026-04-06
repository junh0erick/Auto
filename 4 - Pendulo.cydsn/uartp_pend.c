/* ============================================================
   uartp_pend.c — Protocolo UART PSoC ↔ MATLAB v6.2
   Proyecto: Péndulo Invertido — PSoC 5LP (control en PSoC)

   v6.2: isr_2 conectado a UART_1 rx_interrupt.
         UARTP_Rx_ISR() maneja 's'/'u' en STREAM mode sin polling.
   ============================================================ */
#include "uartp_pend.h"
/* isr_1 e isr_2 incluidos via project.h — no se importan explícitamente */
#include "pendulo.h"
#include "motor.h"
#include "ctrl_pend.h"
#include <string.h>

/* ============================================================
   Estado global del protocolo
   ============================================================ */
volatile uartp_sys_mode_t UARTP_SysMode  = UARTP_SYS_COMMAND;
volatile float            UARTP_PendingRef = 0.0f;

/* ============================================================
   RX state machine en modo STREAM
   RX_IDLE    → espera 's' o 'u'
   RX_GOT_U   → recibiendo 4 bytes del nuevo ref_inner
   ============================================================ */
#define RX_IDLE  0u
#define RX_GOT_U 1u
static uint8 s_rx_state   = RX_IDLE;
static uint8 s_rx_ubuf[4];
static uint8 s_rx_ucnt    = 0u;

/* ============================================================
   Tamaño del payload del comando 'p'
   ============================================================ */
#define P_PAYLOAD_SZ  216u
static uint8 s_p_buf[P_PAYLOAD_SZ];

/* ============================================================
   LL helpers
   ============================================================ */
#define WORD_TIMEOUT_MS  500u
#define MAX_RETRIES      50u

static void ll_putchar(uint8 c)
{
    UART_1_WriteTxData(c);
}

/* Espera un byte en RX con timeout (devuelve 0xFE si timeout). */
static uint8 ll_wait_byte(uint32 timeout_ms)
{
    uint32 cnt = 0u;
    while (UART_1_GetRxBufferSize() == 0u)
    {
        CyDelay(1u);
        if (++cnt >= timeout_ms) return 0xFEu;   /* 0xFE = timeout sentinel */
    }
    return UART_1_ReadRxData();
}

/* Recibe una word de 4 bytes con eco-confirmación (para comando 'f'). */
static uint8 ll_recv_word4(uint8 *out4)
{
    uint8 i, w[4], ctl;
    uint8 tries = 0u;

    for (;;)
    {
        if (++tries > MAX_RETRIES) return 0u;
        for (i = 0u; i < 4u; i++) {
            w[i] = ll_wait_byte(WORD_TIMEOUT_MS);
            if (w[i] == 0xFEu) return 0u;
        }
        for (i = 0u; i < 4u; i++) ll_putchar(w[i]);
        ctl = ll_wait_byte(WORD_TIMEOUT_MS);
        if (ctl == 0xFEu) return 0u;
        if (ctl == (uint8)'A') {
            ll_putchar((uint8)'A');
            out4[0]=w[0]; out4[1]=w[1]; out4[2]=w[2]; out4[3]=w[3];
            return 1u;
        } else {
            ll_putchar((uint8)'N');
        }
    }
}

/* ============================================================
   handle_cmd_p()
   Recibe y procesa el payload del comando 'p'.
   Formato: [len_lo][len_hi][216 bytes payload][1 byte xsum XOR]
   ============================================================ */
static void handle_cmd_p(void)
{
    uint16 i;
    uint8  b_lo, b_hi;
    uint16 len;
    uint8  xsum_calc = 0u;
    uint8  xsum_recv;
    float  tmp_f;
    uint8  *pf = (uint8*)&tmp_f;
    float  coeffs[CTRL_COEF_COUNT];
    float  ref_inner, sat_min, sat_max;
    uint8  mode_inner, mode_outer;

    /* --- Leer longitud (2B LE) --- */
    b_lo = ll_wait_byte(WORD_TIMEOUT_MS);
    b_hi = ll_wait_byte(WORD_TIMEOUT_MS);
    if (b_lo == 0xFEu || b_hi == 0xFEu) { ll_putchar((uint8)'E'); return; }

    len = (uint16)b_lo | ((uint16)b_hi << 8u);
    if (len != P_PAYLOAD_SZ) {
        /* Drainear y rechazar */
        for (i = 0u; i < len + 1u; i++) ll_wait_byte(50u);
        ll_putchar((uint8)'E');
        return;
    }

    /* --- Leer payload con checksum XOR acumulado --- */
    for (i = 0u; i < P_PAYLOAD_SZ; i++) {
        s_p_buf[i] = ll_wait_byte(WORD_TIMEOUT_MS);
        xsum_calc ^= s_p_buf[i];
    }

    /* --- Verificar checksum --- */
    xsum_recv = ll_wait_byte(WORD_TIMEOUT_MS);
    if (xsum_recv != xsum_calc) {
        ll_putchar((uint8)'E');
        return;
    }

    /* --- Parsear modos y tipo numérico --- */
    mode_inner = s_p_buf[0];
    mode_outer = s_p_buf[1];
    ctrl_set_num_type(s_p_buf[2]);   /* byte[2]: CTRL_NUM_F32/F64/Q31/Q15/Q7 */

    /* --- Aplicar inner (bytes 4..103 = 25 float32) --- */
    ctrl_set_mode(PLANT_INNER, mode_inner);
    for (i = 0u; i < CTRL_COEF_COUNT; i++) {
        pf[0] = s_p_buf[4u  + i*4u];
        pf[1] = s_p_buf[5u  + i*4u];
        pf[2] = s_p_buf[6u  + i*4u];
        pf[3] = s_p_buf[7u  + i*4u];
        coeffs[i] = tmp_f;
    }
    ctrl_apply_coeffs(PLANT_INNER, coeffs, CTRL_COEF_COUNT);

    /* --- Aplicar outer (bytes 104..203 = 25 float32) --- */
    ctrl_set_mode(PLANT_OUTER, mode_outer);
    for (i = 0u; i < CTRL_COEF_COUNT; i++) {
        pf[0] = s_p_buf[104u + i*4u];
        pf[1] = s_p_buf[105u + i*4u];
        pf[2] = s_p_buf[106u + i*4u];
        pf[3] = s_p_buf[107u + i*4u];
        coeffs[i] = tmp_f;
    }
    ctrl_apply_coeffs(PLANT_OUTER, coeffs, CTRL_COEF_COUNT);

    /* --- ref_inner (bytes 204..207) --- */
    pf[0]=s_p_buf[204]; pf[1]=s_p_buf[205]; pf[2]=s_p_buf[206]; pf[3]=s_p_buf[207];
    ref_inner = tmp_f;

    /* --- sat_min (bytes 208..211) --- */
    pf[0]=s_p_buf[208]; pf[1]=s_p_buf[209]; pf[2]=s_p_buf[210]; pf[3]=s_p_buf[211];
    sat_min = tmp_f;

    /* --- sat_max (bytes 212..215) --- */
    pf[0]=s_p_buf[212]; pf[1]=s_p_buf[213]; pf[2]=s_p_buf[214]; pf[3]=s_p_buf[215];
    sat_max = tmp_f;

    /* --- Aplicar saturación y guardar ref para ctrl_start() --- */
    ctrl_set_sat(sat_min, sat_max);
    UARTP_PendingRef = ref_inner;

    ll_putchar((uint8)'K');
}

/* ============================================================
   UARTP_Init()
   ============================================================ */
void UARTP_Init(void)
{
    UART_1_Start();
    isr_2_StartEx(UARTP_Rx_ISR);    /* conectar ISR al rx_interrupt de UART_1 */
    UARTP_SysMode  = UARTP_SYS_COMMAND;
    s_rx_state     = RX_IDLE;
    s_rx_ucnt      = 0u;
}

/* ============================================================
   UARTP_ProcessCommand()
   Procesa UN comando en modo COMMAND.
   Llamar repetidamente desde el loop principal.
   ============================================================ */
void UARTP_ProcessCommand(void)
{
    uint8 cmd;
    uint8 payload[4];
    float fs_hz;
    uint8 *pf;

    if (UART_1_GetRxBufferSize() == 0u) return;

    cmd = UART_1_ReadRxData();

    switch (cmd)
    {
        /* ---- Reset ---- */
        case (uint8)'r':
            Motor_Free();
            ctrl_stop();
            g_flag_control   = 0u;
            UARTP_SysMode    = UARTP_SYS_COMMAND;
            UARTP_PendingRef = 0.0f;
            s_rx_state       = RX_IDLE;
            ll_putchar((uint8)'K');
            break;

        /* ---- Set Fs_inner (compatibilidad) ---- */
        case (uint8)'f':
            ll_putchar((uint8)'R');
            if (ll_recv_word4(payload))
            {
                pf    = (uint8*)&fs_hz;
                pf[0] = payload[0]; pf[1] = payload[1];
                pf[2] = payload[2]; pf[3] = payload[3];
                /* Actualizar timer directamente */
                if (fs_hz > 0.5f && fs_hz <= 10000.0f) {
                    uint32 period = (uint32)((float)TIMER_CLOCK_HZ / fs_hz) - 1u;
                    Timer_1_Stop();
                    Timer_1_WritePeriod(period);
                    Timer_1_Start();
                }
                ll_putchar((uint8)'K');
            }
            else
            {
                ll_putchar((uint8)'!');
            }
            break;

        /* ---- Set params (NUEVO v6) ---- */
        case (uint8)'p':
            handle_cmd_p();
            break;

        /* ---- Verificar params almacenados (eco de s_p_buf) ---- */
        /* MATLAB envía 'v' tras 'p' para confirmar que el PSoC almacenó         */
        /* exactamente lo que se envió. PSoC responde: [216 bytes] + [xsum XOR]. */
        case (uint8)'v':
        {
            uint16 vi;
            uint8  vxs = 0u;
            for (vi = 0u; vi < P_PAYLOAD_SZ; vi++) {
                ll_putchar(s_p_buf[vi]);
                vxs ^= s_p_buf[vi];
            }
            ll_putchar(vxs);   /* checksum del payload almacenado */
            break;
        }

        /* ---- Iniciar STREAM (control en PSoC) ---- */
        case (uint8)'i':
            Motor_Free();
            g_flag_control = 0u;
            s_rx_state     = RX_IDLE;
            s_rx_ucnt      = 0u;
            ctrl_start(UARTP_PendingRef);
            UARTP_SysMode  = UARTP_SYS_CONTROL;
            ll_putchar((uint8)'K');
            break;

        /* ---- Stop ---- */
        case (uint8)'s':
            Motor_Free();
            ctrl_stop();
            UARTP_SysMode = UARTP_SYS_COMMAND;
            ll_putchar((uint8)'K');
            break;

        default:
            break;
    }
}

/* ============================================================
   UARTP_Rx_ISR()
   ISR conectado a isr_2 → UART_1 rx_interrupt.
   Se ejecuta inmediatamente al llegar cada byte en STREAM mode.
   En COMMAND mode: no consume bytes (los deja en el buffer de UART_1
   para que UARTP_ProcessCommand los procese).

   Protocolo STREAM:
     's'            → stop inmediato + 'K' + vuelve a COMMAND
     'u' + [4B f32] → actualizar ref_inner live (sin respuesta)
   ============================================================ */
CY_ISR(UARTP_Rx_ISR)
{
    uint8  b;
    float  ref_val;
    uint8 *pf = (uint8*)&ref_val;

    if (UARTP_SysMode != UARTP_SYS_CONTROL) return;

    while (UART_1_GetRxBufferSize() > 0u)
    {
        b = UART_1_ReadRxData();

        if (s_rx_state == RX_IDLE)
        {
            if (b == (uint8)'s')
            {
                Motor_Free();
                ctrl_stop();
                UARTP_SysMode = UARTP_SYS_COMMAND;
                ll_putchar((uint8)'K');
                s_rx_state = RX_IDLE;
                return;
            }
            else if (b == (uint8)'u')
            {
                s_rx_state = RX_GOT_U;
                s_rx_ucnt  = 0u;
            }
            /* otros bytes: ignorar */
        }
        else  /* RX_GOT_U: acumulando 4 bytes del nuevo ref */
        {
            s_rx_ubuf[s_rx_ucnt++] = b;
            if (s_rx_ucnt >= 4u)
            {
                pf[0]=s_rx_ubuf[0]; pf[1]=s_rx_ubuf[1];
                pf[2]=s_rx_ubuf[2]; pf[3]=s_rx_ubuf[3];
                ctrl_update_ref(ref_val);
                s_rx_state = RX_IDLE;
            }
        }
    }
}

/* ============================================================
   UARTP_ControlRxPoll()
   Fallback polling — mantenido si isr_2 no está disponible.
   Con isr_2 activo, este código NO debe llamarse.
   ============================================================ */
void UARTP_ControlRxPoll(void)
{
    uint8 b;
    float ref_val;
    uint8 *pf = (uint8*)&ref_val;

    while (UART_1_GetRxBufferSize() > 0u)
    {
        b = UART_1_ReadRxData();

        if (s_rx_state == RX_IDLE)
        {
            if (b == (uint8)'s')
            {
                /* Stop command */
                Motor_Free();
                ctrl_stop();
                UARTP_SysMode = UARTP_SYS_COMMAND;
                ll_putchar((uint8)'K');
                s_rx_state = RX_IDLE;
                return;
            }
            else if (b == (uint8)'u')
            {
                /* Inicio de actualización de referencia */
                s_rx_state = RX_GOT_U;
                s_rx_ucnt  = 0u;
            }
            /* otros bytes: ignorar */
        }
        else  /* RX_GOT_U: acumulando 4 bytes del nuevo ref */
        {
            s_rx_ubuf[s_rx_ucnt++] = b;
            if (s_rx_ucnt >= 4u)
            {
                pf[0]=s_rx_ubuf[0]; pf[1]=s_rx_ubuf[1];
                pf[2]=s_rx_ubuf[2]; pf[3]=s_rx_ubuf[3];
                ctrl_update_ref(ref_val);
                s_rx_state = RX_IDLE;
            }
        }
    }
}
