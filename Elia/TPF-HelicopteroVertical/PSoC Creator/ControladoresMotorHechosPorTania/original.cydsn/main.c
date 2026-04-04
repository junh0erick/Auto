#include "project.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>

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
