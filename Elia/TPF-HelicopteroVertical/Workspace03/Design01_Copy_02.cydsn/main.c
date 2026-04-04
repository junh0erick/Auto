#include "project.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

/* ===================== ESC PWM config ===================== */
#define ESC_MIN_US      (1000u)
#define ESC_MAX_US      (2000u)
#define ESC_ARM_MS      (6000u)

#define LOOP_DT_MS      (20u)   // 50 Hz update
#define RAMP_STEP_US    (5u)    // suavidad de cambio

static uint16 current_us = ESC_MIN_US;
static uint16 target_us  = ESC_MIN_US;

/* ===================== UART Console helpers ===================== */
static void cons_init(void)
{
    UART_CONS_Start();
}

static void cons_puts(const char *s)
{
    UART_CONS_PutString(s);
}

static void cons_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    UART_CONS_PutString(buf);
}

/* ===================== ESC helpers ===================== */
static uint16 clamp_u16(uint16 x, uint16 lo, uint16 hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void esc_write_us(uint16 us)
{
    us = clamp_u16(us, ESC_MIN_US, ESC_MAX_US);
    PWM_ESC_WriteCompare(us);   // CMP Type = Less => pulso ~us microsegundos
}

static void esc_arm(void)
{
    target_us  = ESC_MIN_US;
    current_us = ESC_MIN_US;
    esc_write_us(ESC_MIN_US);
    cons_puts("[ESC] Arming: holding 1000us...\r\n");
    CyDelay(ESC_ARM_MS);
    cons_puts("[ESC] Armed.\r\n");
}

/* ===================== DS18B20 (1-Wire) ===================== */
/* Requiere:
   - Pin component llamado ONEWIRE conectado a DQ del DS18B20
   - Pull-up 4.7k entre DQ y 3.3V (recomendado)
   - ONEWIRE en "Open Drain, Drives Low" (ideal)
   Pinout (como tu foto, texto de frente): Izq=GND, Centro=DQ, Der=VDD
*/
static inline void ow_delay_us(uint16 us) { CyDelayUs(us); }
static inline void ow_drive_low(void)    { ONEWIRE_Write(0); }
static inline void ow_release(void)      { ONEWIRE_Write(1); }
static inline uint8 ow_read_pin(void)    { return (uint8)ONEWIRE_Read(); }

static uint8 ow_reset(void)
{
    uint8 presence;

    ow_drive_low();
    ow_delay_us(480);
    ow_release();
    ow_delay_us(70);

    presence = (ow_read_pin() == 0u) ? 1u : 0u;  // presence pulse LOW

    ow_delay_us(410);
    return presence;
}

static void ow_write_bit(uint8 b)
{
    if (b) {
        ow_drive_low();
        ow_delay_us(6);
        ow_release();
        ow_delay_us(64);
    } else {
        ow_drive_low();
        ow_delay_us(60);
        ow_release();
        ow_delay_us(10);
    }
}

static uint8 ow_read_bit(void)
{
    uint8 b;

    ow_drive_low();
    ow_delay_us(6);
    ow_release();
    ow_delay_us(9);

    b = ow_read_pin();

    ow_delay_us(55);
    return b;
}

static void ow_write_byte(uint8 v)
{
    for (int i = 0; i < 8; i++) {
        ow_write_bit((uint8)(v & 0x01u));
        v >>= 1;
    }
}

static uint8 ow_read_byte(void)
{
    uint8 v = 0;
    for (int i = 0; i < 8; i++) {
        v >>= 1;
        if (ow_read_bit()) v |= 0x80u;
    }
    return v;
}

#define DS_CMD_SKIP_ROM         (0xCCu)
#define DS_CMD_CONVERT_T        (0x44u)
#define DS_CMD_READ_SCRATCHPAD  (0xBEu)

/* Devuelve 1 si OK y escribe temp en *temp_c */
static uint8 ds18b20_read_temp_c(float *temp_c)
{
    if (!ow_reset()) return 0u;

    ow_write_byte(DS_CMD_SKIP_ROM);
    ow_write_byte(DS_CMD_CONVERT_T);

    /* Espera fin de conversión (máx ~750ms a 12-bit).
       Polling cada 10ms, timeout ~800ms */
    for (int k = 0; k < 80; k++) {
        if (ow_read_pin() != 0u) break;
        CyDelay(10);
    }

    if (!ow_reset()) return 0u;

    ow_write_byte(DS_CMD_SKIP_ROM);
    ow_write_byte(DS_CMD_READ_SCRATCHPAD);

    uint8 lsb = ow_read_byte();
    uint8 msb = ow_read_byte();

    // descartamos 7 bytes restantes
    for (int i = 0; i < 7; i++) (void)ow_read_byte();

    int16 raw = (int16)(((uint16)msb << 8) | (uint16)lsb);
    *temp_c = ((float)raw) / 16.0f; // 12-bit: 1/16 °C
    return 1u;
}

static void do_temp(void)
{
    float tC;
    if (ds18b20_read_temp_c(&tC)) {

        /* Imprimir sin %f: convertir a centi-grados */
        int32_t t100 = (int32_t)(tC * 100.0f);   // ej: 2534 = 25.34 C
        int32_t whole = t100 / 100;
        int32_t frac  = t100 % 100;
        if (frac < 0) frac = -frac;

        cons_printf("[TEMP] %ld.%02ld C\r\n", (long)whole, (long)frac);
    } else {
        cons_puts("[TEMP] ERROR: DS18B20 no responde (presence=0)\r\n");
    }
}


/* ===================== Console commands ===================== */
static void print_help(void)
{
    cons_puts("\r\n=== ESC Console (COM3 / KitProg USB-UART) ===\r\n");
    cons_puts("Comandos:\r\n");
    cons_puts("  help              -> esta ayuda\r\n");
    cons_puts("  status            -> imprime current/target\r\n");
    cons_puts("  arm               -> fuerza 1000us por unos segundos\r\n");
    cons_puts("  stop              -> target=1000\r\n");
    cons_puts("  set ####          -> set 1000..2000 (ej: set 1500)\r\n");
    cons_puts("  up ####           -> suma #### us (ej: up 50)\r\n");
    cons_puts("  dn ####           -> resta #### us (ej: dn 50)\r\n");
    cons_puts("  kick              -> 1300us por 250ms y vuelve\r\n");
    cons_puts("  temp              -> lee DS18B20 e imprime temperatura\r\n");
    cons_puts("Tip: tambien podes mandar solo un numero: 1500\r\n\r\n");
}

static void print_status(void)
{
    cons_printf("[STATUS] current=%u us | target=%u us\r\n", current_us, target_us);
}

/* ===================== Command parser ===================== */
static void trim(char *s)
{
    while (isspace((unsigned char)*s)) memmove(s, s+1, strlen(s));
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n-1])) s[--n] = '\0';
}

static int parse_int(const char *s)
{
    while (isspace((unsigned char)*s)) s++;
    return atoi(s);
}

static void do_kick(void)
{
    uint16 saved = target_us;
    cons_puts("[ESC] KICK 1300us 250ms\r\n");
    esc_write_us(1300);
    CyDelay(250);
    esc_write_us(current_us);
    target_us = saved;
}

static void handle_line(char *line)
{
    trim(line);
    if (line[0] == '\0') return;

    char cmd[64];
    strncpy(cmd, line, sizeof(cmd)-1);
    cmd[sizeof(cmd)-1] = '\0';
    for (size_t i=0; cmd[i]; i++) cmd[i] = (char)tolower((unsigned char)cmd[i]);

    if (!strcmp(cmd, "help"))   { print_help();   return; }
    if (!strcmp(cmd, "status")) { print_status(); return; }
    if (!strcmp(cmd, "arm"))    { esc_arm();      return; }
    if (!strcmp(cmd, "stop"))   { target_us = ESC_MIN_US; cons_puts("[CMD] stop -> target=1000\r\n"); return; }
    if (!strcmp(cmd, "kick"))   { do_kick();      return; }
    if (!strcmp(cmd, "temp") || !strcmp(cmd, "temp?")) { do_temp(); return; }

    if (!strncmp(cmd, "set", 3)) {
        int v = parse_int(line + 3);
        if (v < (int)ESC_MIN_US || v > (int)ESC_MAX_US) { cons_puts("[ERR] set: usa 1000..2000\r\n"); return; }
        target_us = (uint16)v;
        cons_printf("[CMD] set -> target=%u\r\n", target_us);
        return;
    }

    if (!strncmp(cmd, "up", 2)) {
        int dv = parse_int(line + 2);
        int v  = (int)target_us + dv;
        v = (v < (int)ESC_MIN_US) ? (int)ESC_MIN_US : v;
        v = (v > (int)ESC_MAX_US) ? (int)ESC_MAX_US : v;
        target_us = (uint16)v;
        cons_printf("[CMD] up -> target=%u\r\n", target_us);
        return;
    }

    if (!strncmp(cmd, "dn", 2)) {
        int dv = parse_int(line + 2);
        int v  = (int)target_us - dv;
        v = (v < (int)ESC_MIN_US) ? (int)ESC_MIN_US : v;
        v = (v > (int)ESC_MAX_US) ? (int)ESC_MAX_US : v;
        target_us = (uint16)v;
        cons_printf("[CMD] dn -> target=%u\r\n", target_us);
        return;
    }

    if (isdigit((unsigned char)cmd[0])) {
        int v = atoi(cmd);
        if (v < (int)ESC_MIN_US || v > (int)ESC_MAX_US) { cons_puts("[ERR] numero invalido: usa 1000..2000\r\n"); return; }
        target_us = (uint16)v;
        cons_printf("[CMD] set -> target=%u\r\n", target_us);
        return;
    }

    cons_puts("[ERR] comando no reconocido. Escribi: help\r\n");
}

/* ===================== UART RX line buffer ===================== */
static void process_cons_rx(void)
{
    static char line[96];
    static uint8 idx = 0;

    while (UART_CONS_GetRxBufferSize() != 0u)
    {
        uint8 ch = (uint8)UART_CONS_GetChar();

        if (ch == '\r' || ch == '\n') {
            line[idx] = '\0';
            handle_line(line);
            idx = 0;
        } else {
            if (idx < sizeof(line)-1) {
                line[idx++] = (char)ch;
            } else {
                idx = 0;
                cons_puts("[ERR] linea demasiado larga\r\n");
            }
        }
    }
}

/* ===================== main ===================== */
int main(void)
{
    CyGlobalIntEnable;

    PWM_ESC_Start();
    esc_write_us(ESC_MIN_US);

    cons_init();

    /* 1-Wire bus idle high */
    ow_release();

    cons_puts("\r\n[BOOT] PSoC ESC Console via KitProg USB-UART (COM3).\r\n");
    cons_puts("[BOOT] Abri PuTTY en COM3 @115200. Escribi: help\r\n");
    print_help();

    /* Recomendación: NO bloquear 6s al boot. Armalo por comando. */
    // esc_arm();

    for (;;)
    {
        process_cons_rx();

        if (current_us < target_us) {
            uint16 next = current_us + RAMP_STEP_US;
            current_us = (next > target_us) ? target_us : next;
        } else if (current_us > target_us) {
            uint16 next = current_us - RAMP_STEP_US;
            current_us = (next < target_us) ? target_us : next;
        }

        esc_write_us(current_us);
        CyDelay(LOOP_DT_MS);
    }
}
