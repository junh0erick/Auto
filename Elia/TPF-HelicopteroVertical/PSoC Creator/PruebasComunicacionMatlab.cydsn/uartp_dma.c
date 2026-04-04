#include "uartp_dma.h"
#include <string.h>


/* =========================
   Contexto estático
   ========================= */
static uartp_cfg_t g_cfg;
static bool g_inited = false;

/* Handlers */
static uartp_handler_t g_handlers[256];
static uartp_handler_t g_default_handler = 0;

/* RX ping-pong */
static uint8 g_rx_pp[UARTP_RX_PP_SIZE];
static volatile uint8 g_rx_half_ready0 = 0u;
static volatile uint8 g_rx_half_ready1 = 0u;
static volatile uint8 g_rx_half_toggle = 0u;

/* RX DMA */
static uint8 g_dma_rx_chan = 0xFFu;
static uint8 g_dma_rx_td0  = 0xFFu;
static uint8 g_dma_rx_td1  = 0xFFu;

/* TX frame buffer */
static uint8  g_tx_buf[UARTP_TX_BUF_SIZE];
static uint16 g_tx_len = 0u;
static volatile bool g_tx_busy = false;

/* TX DMA */
static uint8 g_dma_tx_chan = 0xFFu;
static uint8 g_dma_tx_td   = 0xFFu;

/* =========================
   Parser (CPU)
   ========================= */
typedef enum {
    RX_WAIT_SOF = 0,
    RX_CMD,
    RX_LEN_L,
    RX_LEN_H,
    RX_PAYLOAD,
    RX_CKSUM
} rx_state_t;

static rx_state_t s_rx_state = RX_WAIT_SOF;
static uint8  s_cmd = 0u;
static uint16 s_len = 0u;
static uint16 s_idx = 0u;
static uint8  s_ck  = 0u;
static uint8  s_payload[UARTP_MAX_PAYLOAD];

static void parser_reset(void)
{
    s_rx_state = RX_WAIT_SOF;
    s_cmd = 0u;
    s_len = 0u;
    s_idx = 0u;
    s_ck  = 0u;
}

uint8 UARTP_ChecksumXor(uint8 cmd, uint16 len_bytes, const uint8* payload)
{
    /* payload puede ser NULL si len=0 */
    uint8 c = 0u;
    uint16 i;

    c ^= cmd;
    c ^= (uint8)(len_bytes & 0xFFu);
    c ^= (uint8)((len_bytes >> 8) & 0xFFu);

    for (i = 0u; i < len_bytes; i++) {
        c ^= payload[i];
    }
    return c;
}

static void dispatch_frame(uint8 cmd, const uint8* payload, uint16 len)
{
    uartp_handler_t h = g_handlers[cmd];
    if (h != 0) {
        h(cmd, payload, len);
    } else if (g_default_handler != 0) {
        g_default_handler(cmd, payload, len);
    }
}

static void parser_feed(uint8 b)
{
    switch (s_rx_state)
    {
        case RX_WAIT_SOF:
            if (b == (uint8)UARTP_SOF) {
                s_rx_state = RX_CMD;
                s_ck = 0u;
            }
            break;

        case RX_CMD:
            s_cmd = b;
            s_ck ^= b;
            s_rx_state = RX_LEN_L;
            break;

        case RX_LEN_L:
            s_len = (uint16)b;
            s_ck ^= b;
            s_rx_state = RX_LEN_H;
            break;

        case RX_LEN_H:
            s_len |= ((uint16)b << 8);
            s_ck ^= b;

            if (s_len > (uint16)UARTP_MAX_PAYLOAD) {
                parser_reset();
            } else if (s_len == 0u) {
                s_idx = 0u;
                s_rx_state = RX_CKSUM;
            } else {
                s_idx = 0u;
                s_rx_state = RX_PAYLOAD;
            }
            break;

        case RX_PAYLOAD:
            s_payload[s_idx++] = b;
            s_ck ^= b;
            if (s_idx >= s_len) {
                s_rx_state = RX_CKSUM;
            }
            break;

        case RX_CKSUM:
            if (b == s_ck) {
                dispatch_frame(s_cmd, s_payload, s_len);
            }
            parser_reset();
            break;

        default:
            parser_reset();
            break;
    }
}

/* =========================
   DMA ISRs (NRQ)
   ========================= */
CY_ISR(UARTP_DmaRxIsr)
{
    /* Cada TD completo = un half listo */
    if (g_rx_half_toggle == 0u) {
        g_rx_half_ready0 = 1u;
        g_rx_half_toggle = 1u;
    } else {
        g_rx_half_ready1 = 1u;
        g_rx_half_toggle = 0u;
    }
}

CY_ISR(UARTP_DmaTxIsr)
{
    g_tx_busy = false;
}

/* =========================
   API
   ========================= */
bool UARTP_Init(const uartp_cfg_t* cfg)
{
    uint16 half;

    if (cfg == 0) return false;
    if (cfg->uart_rxdata_ptr == 0 || cfg->uart_txdata_ptr == 0) return false;
    if (cfg->dma_rx_init == 0 || cfg->dma_tx_init == 0) return false;

    memcpy(&g_cfg, cfg, sizeof(uartp_cfg_t));

    memset(g_handlers, 0, sizeof(g_handlers));
    g_default_handler = 0;

    parser_reset();

    g_rx_half_ready0 = 0u;
    g_rx_half_ready1 = 0u;
    g_rx_half_toggle = 0u;

    g_tx_busy = false;
    g_tx_len = 0u;

    half = (uint16)(UARTP_RX_PP_SIZE / 2u);

    /* =========================
       RX DMA ping-pong
       ========================= */
    g_dma_rx_chan = g_cfg.dma_rx_init(g_cfg.bytes_per_burst,
                                     g_cfg.requests_per_burst,
                                     HI16((uint32)g_cfg.uart_rxdata_ptr),
                                     HI16((uint32)g_rx_pp));

    g_dma_rx_td0 = CyDmaTdAllocate();
    g_dma_rx_td1 = CyDmaTdAllocate();
    if (g_dma_rx_td0 == CY_DMA_INVALID_TD || g_dma_rx_td1 == CY_DMA_INVALID_TD) {
        return false;
    }

    /* TD0: escribe half bytes en g_rx_pp[0..half-1] */
    CyDmaTdSetConfiguration(g_dma_rx_td0,
                           half,
                           g_dma_rx_td1,
                           (uint8)(TD_INC_DST_ADR | TD_TERMOUT0_EN));

    CyDmaTdSetAddress(g_dma_rx_td0,
                      LO16((uint32)g_cfg.uart_rxdata_ptr),
                      LO16((uint32)&g_rx_pp[0]));

    /* TD1: escribe half bytes en g_rx_pp[half..end] */
    CyDmaTdSetConfiguration(g_dma_rx_td1,
                           half,
                           g_dma_rx_td0,
                           (uint8)(TD_INC_DST_ADR | TD_TERMOUT0_EN));

    CyDmaTdSetAddress(g_dma_rx_td1,
                      LO16((uint32)g_cfg.uart_rxdata_ptr),
                      LO16((uint32)&g_rx_pp[half]));

    CyDmaChSetInitialTd(g_dma_rx_chan, g_dma_rx_td0);
    CyDmaChEnable(g_dma_rx_chan, 1u);

    /* =========================
       TX DMA (1 TD reusable)
       ========================= */
    g_dma_tx_chan = g_cfg.dma_tx_init(g_cfg.bytes_per_burst,
                                     g_cfg.requests_per_burst,
                                     HI16((uint32)g_tx_buf),
                                     HI16((uint32)g_cfg.uart_txdata_ptr));

    g_dma_tx_td = CyDmaTdAllocate();
    if (g_dma_tx_td == CY_DMA_INVALID_TD) {
        return false;
    }

    g_inited = true;
    return true;
}

void UARTP_RegisterHandler(uint8 cmd, uartp_handler_t h)
{
    g_handlers[cmd] = h;
}

void UARTP_SetDefaultHandler(uartp_handler_t h)
{
    g_default_handler = h;
}

bool UARTP_TxBusy(void)
{
    return g_tx_busy;
}

bool UARTP_Send(uint8 cmd, const void* payload, uint16 len_bytes)
{
    uint16 k;
    uint8 cks;

    if (!g_inited) return false;
    if (g_tx_busy) return false;
    if (len_bytes > (uint16)UARTP_MAX_PAYLOAD) return false;

    /* Armar frame */
    k = 0u;
    g_tx_buf[k++] = (uint8)UARTP_SOF;
    g_tx_buf[k++] = cmd;
    g_tx_buf[k++] = (uint8)(len_bytes & 0xFFu);
    g_tx_buf[k++] = (uint8)((len_bytes >> 8) & 0xFFu);

    if (len_bytes > 0u) {
        memcpy(&g_tx_buf[k], payload, len_bytes);
        k = (uint16)(k + len_bytes);
    }

    cks = UARTP_ChecksumXor(cmd, len_bytes, (len_bytes > 0u) ? (const uint8*)payload : (const uint8*)0);
    g_tx_buf[k++] = cks;

    g_tx_len = k;
    g_tx_busy = true;

    /* Configurar TD */
    CyDmaTdSetConfiguration(g_dma_tx_td,
                           g_tx_len,
                           CY_DMA_DISABLE_TD,
                           (uint8)(TD_INC_SRC_ADR | TD_TERMOUT0_EN));

    CyDmaTdSetAddress(g_dma_tx_td,
                      LO16((uint32)&g_tx_buf[0]),
                      LO16((uint32)g_cfg.uart_txdata_ptr));

    CyDmaChSetInitialTd(g_dma_tx_chan, g_dma_tx_td);
    CyDmaChEnable(g_dma_tx_chan, 1u);

    return true;
}

void UARTP_Task(void)
{
    uint16 half, i;

    if (!g_inited) return;

    half = (uint16)(UARTP_RX_PP_SIZE / 2u);

    if (g_rx_half_ready0) {
        g_rx_half_ready0 = 0u;
        for (i = 0u; i < half; i++) {
            parser_feed(g_rx_pp[i]);
        }
    }

    if (g_rx_half_ready1) {
        g_rx_half_ready1 = 0u;
        for (i = 0u; i < half; i++) {
            parser_feed(g_rx_pp[half + i]);
        }
    }
}
