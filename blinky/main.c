/*
 * Blinky + serial command shell for the NXP FRDM-MCXA153.
 *
 * RGB LED pins (from NXP's frdmmcxa153 board.h), all active-low:
 *   red   = P3_12
 *   green = P3_13
 *   blue  = P3_0
 *
 * Shell runs on LPUART0 (P0_2 RX / P0_3 TX, ALT2), which the on-board
 * MCU-Link bridges to the host as a USB CDC port (/dev/ttyACM0 on Linux).
 * 115200 8N1. Type "help" for commands.
 *
 * A second, independent shell runs on the MCU's own USB FS port (the
 * FRDM-MCXA153's MCU USB connector, J8) as a TinyUSB CDC-ACM device, e.g.
 * /dev/ttyACM1. Both shells accept the same commands; each command's
 * output goes back to the port it was typed on. The banner is printed
 * each time a terminal opens the USB port.
 *
 * Two PWM outputs from CTIMER1, 12 kHz, 1000 counts/period:
 *   channel 1 = P1_4 (header J1-2), match 2
 *   channel 2 = P1_3, match 1
 * They share the period (MR3) but each has its own match register, so the
 * duty cycles are set independently and both run at the same time.
 * Add an RC low-pass (e.g. 10k + 1uF) to each to get an analog voltage of
 * VDD * high_counts / 1000.
 *
 * "ma" / "ma2" drive a 0-3.3 V to 4-20 mA module from each filtered PWM. A
 * two-point calibration per channel ("cal lo/hi <measured mA>", "cal2 ...")
 * maps PWM counts to loop current.
 *
 * "save" stores mode/color/rate/pwm/cal in flash (see settings.c); they are
 * restored at power-up. "defaults" erases them.
 *
 * Core clock is left as the boot ROM set it (FIRC, 48 MHz) and read back from
 * SCG; LPUART0 runs from FRO12M, USB0 from the FIRC 48 MHz peripheral
 * clock. SysTick provides a 1 ms tick.
 */
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "fsl_device_registers.h"
#include "settings.h"
#include "tusb.h"

#define LED_RED   (1UL << 12)
#define LED_GREEN (1UL << 13)
#define LED_BLUE  (1UL << 0)
#define LED_ALL   (LED_RED | LED_GREEN | LED_BLUE)

/* MRCC bit positions (from fsl_clock.h / fsl_reset.h). */
#define MRCC0_CTIMER1 (1UL << 3)
#define MRCC0_LPUART0 (1UL << 19)
#define MRCC0_PORT0   (1UL << 29)
#define MRCC0_USB0    (1UL << 22)
#define MRCC0_PORT1   (1UL << 30)
#define MRCC0_ALL     (MRCC0_CTIMER1 | MRCC0_LPUART0 | MRCC0_USB0 | MRCC0_PORT0 | MRCC0_PORT1)
#define MRCC1_PORT3   (1UL << 0)
#define MRCC1_GPIO3   (1UL << 8)

/* 12 MHz / (OSR 13 * SBR 8) = 115385 baud, 0.16% error. */
#define UART_OSR 13U
#define UART_SBR 8U

/* PWM: CTIMER1 on FRO12M, MR3 = period, MR2 / MR1 = pulse (P1_4 / P1_3, ALT4). */
#define PWM_CLK_HZ  12000000U
#define PWM_COUNTS  1000U
#define PWM_FREQ_HZ (PWM_CLK_HZ / PWM_COUNTS)
#define VDD_MV      3300U /* FRDM-MCXA153 I/O supply */

/* Default calibration: 0 counts = 4 mA, 1000 counts = 20 mA (pots trimmed). */
#define CAL_LO_COUNTS_DEFAULT 0U
#define CAL_LO_UA_DEFAULT     4000U
#define CAL_HI_COUNTS_DEFAULT PWM_COUNTS
#define CAL_HI_UA_DEFAULT     20000U
#define CAL_MIN_SPAN_COUNTS   100U
#define MA_MAX_UA             25000U

#define RX_BUF_SIZE 64U /* power of two */
#define LINE_MAX    64U

#define USB_TX_TIMEOUT_MS 50U /* give up on a host that stops reading */

typedef enum
{
    MODE_CYCLE,
    MODE_SOLID,
    MODE_BLINK,
} led_mode_t;

static volatile uint32_t g_ticks;
static bool g_restored; /* settings came from flash at power-up */
static uint32_t g_reset_srs; /* CMC->SRS at boot: why the chip last reset */

/* One LPUART shell port: its peripheral and interrupt-fed RX ring. */
typedef struct
{
    LPUART_Type *base;
    volatile uint8_t rx_buf[RX_BUF_SIZE];
    volatile uint32_t rx_head;
    volatile uint32_t rx_tail;
    volatile uint32_t rx_overruns;
    volatile uint32_t rx_errors;
} uart_port_t;

static uart_port_t g_uart0 = {.base = LPUART0};

static led_mode_t g_mode     = MODE_CYCLE;
static uint32_t g_color      = LED_RED;
static uint32_t g_period_ms  = 500;
static uint32_t g_last_step;
static uint32_t g_step;

/* One PWM output driving one 4-20 mA module. */
typedef struct
{
    uint32_t mr;      /* CTIMER1 match register for the pulse */
    const char *name; /* channel number as shown in output */
    const char *pin;
    uint32_t high;    /* PWM high time in counts */

    /* Two-point calibration: PWM counts -> measured loop current (uA). */
    uint32_t cal_lo_counts;
    uint32_t cal_lo_ua;
    uint32_t cal_hi_counts;
    uint32_t cal_hi_ua;
} pwm_ch_t;

#define PWM_CHANNELS SETTINGS_CHANNELS

#define CAL_DEFAULTS                                                                   \
    .cal_lo_counts = CAL_LO_COUNTS_DEFAULT, .cal_lo_ua = CAL_LO_UA_DEFAULT,            \
    .cal_hi_counts = CAL_HI_COUNTS_DEFAULT, .cal_hi_ua = CAL_HI_UA_DEFAULT

static pwm_ch_t g_ch[PWM_CHANNELS] = {
    {.mr = 2U, .name = "1", .pin = "P1_4 (J1-2)", CAL_DEFAULTS},
    {.mr = 1U, .name = "2", .pin = "P1_3", CAL_DEFAULTS},
};

static const struct
{
    const char *name;
    uint32_t mask;
} k_colors[] = {
    {"red", LED_RED},
    {"green", LED_GREEN},
    {"blue", LED_BLUE},
    {"yellow", LED_RED | LED_GREEN},
    {"cyan", LED_GREEN | LED_BLUE},
    {"magenta", LED_RED | LED_BLUE},
    {"white", LED_ALL},
    {"off", 0},
};

/* ------------------------------------------------------------------ IRQs */

void SysTick_Handler(void)
{
    g_ticks++;
}

static void uart_rx_irq(uart_port_t *u)
{
    LPUART_Type *base = u->base;
    uint32_t stat     = base->STAT;

    if (stat & LPUART_STAT_OR_MASK)
    {
        base->STAT = LPUART_STAT_OR_MASK; /* write-1-to-clear */
        u->rx_overruns++;
    }
    while (base->STAT & LPUART_STAT_RDRF_MASK)
    {
        uint32_t err  = base->STAT & (LPUART_STAT_FE_MASK | LPUART_STAT_NF_MASK | LPUART_STAT_PF_MASK);
        uint8_t c     = (uint8_t)base->DATA;
        if (err)
        {
            /* Line glitch (e.g. during probe reset): drop the byte. */
            base->STAT = err;
            u->rx_errors++;
            continue;
        }
        uint32_t next = (u->rx_head + 1U) & (RX_BUF_SIZE - 1U);
        if (next != u->rx_tail)
        {
            u->rx_buf[u->rx_head] = c;
            u->rx_head            = next;
        }
        else
        {
            u->rx_overruns++;
        }
    }
}

void LPUART0_IRQHandler(void)
{
    uart_rx_irq(&g_uart0);
}

void USB0_IRQHandler(void)
{
    tusb_int_handler(0, true);
}

/* TinyUSB's time base. */
uint32_t tusb_time_millis_api(void)
{
    return g_ticks;
}

/* -------------------------------------------------------------- hardware */

/* Derive the core clock from SCG/SYSCON. The ROM leaves the core on FIRC
 * (48 MHz on this part), not the 12 MHz DEFAULT_SYSTEM_CLOCK assumes. */
static uint32_t core_clock_hz(void)
{
    static const uint32_t firc_hz[] = {36000000U, 48000000U, 48000000U, 64000000U,
                                       72000000U, 96000000U, 0U,        0U};
    uint32_t src = (SCG0->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT;
    uint32_t hz;

    switch (src)
    {
        case 2U: /* SIRC */
            hz = 12000000U;
            break;
        case 3U: /* FIRC */
            hz = firc_hz[(SCG0->FIRCCFG & SCG_FIRCCFG_FREQ_SEL_MASK) >> SCG_FIRCCFG_FREQ_SEL_SHIFT];
            break;
        default:
            hz = DEFAULT_SYSTEM_CLOCK;
            break;
    }
    return hz / ((SYSCON->AHBCLKDIV & 0xFFU) + 1U);
}

static void clocks_init(void)
{
    /* Route the 12 MHz FRO to peripherals. */
    SCG0->SIRCCSR &= ~SCG_SIRCCSR_LK_MASK;
    SCG0->SIRCCSR |= SCG_SIRCCSR_SIRC_CLK_PERIPH_EN_MASK;
    SCG0->SIRCCSR |= SCG_SIRCCSR_LK_MASK;

    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /* LPUART0 functional clock: FRO12M (sel 0), divide by 1. */
    MRCC0->MRCC_LPUART0_CLKSEL = 0U;
    MRCC0->MRCC_LPUART0_CLKDIV = (1UL << 30) | (1UL << 29); /* halt + reset */
    MRCC0->MRCC_LPUART0_CLKDIV = (1UL << 30);               /* div=1, halted */
    MRCC0->MRCC_LPUART0_CLKDIV = 0U;                        /* run */

    /* CTIMER1 functional clock: FRO12M (sel 0), divide by 1. */
    MRCC0->MRCC_CTIMER1_CLKSEL = 0U;
    MRCC0->MRCC_CTIMER1_CLKDIV = (1UL << 30) | (1UL << 29);
    MRCC0->MRCC_CTIMER1_CLKDIV = (1UL << 30);
    MRCC0->MRCC_CTIMER1_CLKDIV = 0U;

    /* USB0: FIRC 48 MHz peripheral clock (the ROM leaves FIRC at 48 MHz). */
    SCG0->FIRCCSR &= ~SCG_FIRCCSR_LK_MASK;
    SCG0->FIRCCSR |= SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN_MASK;
    SCG0->FIRCCSR |= SCG_FIRCCSR_LK_MASK;
    MRCC0->MRCC_USB0_CLKSEL = 1U; /* CLK_48M */

    /* Ungate bus clocks and release peripherals from reset. */
    MRCC0->MRCC_GLB_CC0_SET  = MRCC0_ALL;
    MRCC0->MRCC_GLB_CC1_SET  = MRCC1_PORT3 | MRCC1_GPIO3;
    MRCC0->MRCC_GLB_RST0_SET = MRCC0_ALL;
    MRCC0->MRCC_GLB_RST1_SET = MRCC1_PORT3 | MRCC1_GPIO3;
    while ((MRCC0->MRCC_GLB_RST0 & MRCC0_ALL) != MRCC0_ALL)
    {
    }
    while ((MRCC0->MRCC_GLB_RST1 & (MRCC1_PORT3 | MRCC1_GPIO3)) != (MRCC1_PORT3 | MRCC1_GPIO3))
    {
    }

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}

static void leds_init(void)
{
    /* Pins as GPIO (ALT0), input buffer enabled, no pulls. */
    PORT3->PCR[12] = PORT_PCR_MUX(0) | PORT_PCR_IBE_MASK;
    PORT3->PCR[13] = PORT_PCR_MUX(0) | PORT_PCR_IBE_MASK;
    PORT3->PCR[0]  = PORT_PCR_MUX(0) | PORT_PCR_IBE_MASK;

    GPIO3->PSOR = LED_ALL; /* all off (active-low) */
    GPIO3->PDDR |= LED_ALL;
}

static void leds_set(uint32_t mask)
{
    GPIO3->PSOR = LED_ALL & ~mask;
    GPIO3->PCOR = mask;
}

/* 115200 8N1, RX interrupt on. */
static void uart_port_init(uart_port_t *u, IRQn_Type irq)
{
    LPUART_Type *base = u->base;

    base->CTRL = 0U;
    base->BAUD = LPUART_BAUD_OSR(UART_OSR - 1U) | LPUART_BAUD_SBR(UART_SBR);
    base->STAT = LPUART_STAT_OR_MASK; /* clear stale overrun */
    base->CTRL = LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK | LPUART_CTRL_RIE_MASK;

    NVIC_EnableIRQ(irq);
}

static void uart_init(void)
{
    /* P0_2 = LPUART0_RXD, P0_3 = LPUART0_TXD (ALT2); pull-up on RX. */
    PORT0->PCR[2] = PORT_PCR_MUX(2) | PORT_PCR_IBE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORT0->PCR[3] = PORT_PCR_MUX(2) | PORT_PCR_IBE_MASK;
    uart_port_init(&g_uart0, LPUART0_IRQn);
}

/* In CTIMER PWM mode the output is low from TC=0 until TC matches the
 * channel's MRn, then high until MR3 resets the counter:
 * high time = PWM_COUNTS - MRn. */
static void pwm_set(pwm_ch_t *ch, uint32_t high_counts)
{
    if (high_counts > PWM_COUNTS)
    {
        high_counts = PWM_COUNTS;
    }
    ch->high           = high_counts;
    CTIMER1->MR[ch->mr] = PWM_COUNTS - high_counts; /* MRn = PWM_COUNTS never matches: 0% */
}

static void pwm_init(void)
{
    PORT1->PCR[4] = PORT_PCR_MUX(4); /* CT1_MAT2, channel 1 */
    PORT1->PCR[3] = PORT_PCR_MUX(4); /* CT1_MAT1, channel 2 */

    CTIMER1->TCR  = CTIMER_TCR_CRST_MASK;
    CTIMER1->PR   = 0U;
    CTIMER1->CTCR = 0U;
    CTIMER1->MCR  = CTIMER_MCR_MR3R_MASK;
    CTIMER1->MR[3] = PWM_COUNTS - 1U;
    for (uint32_t i = 0; i < PWM_CHANNELS; i++)
    {
        pwm_set(&g_ch[i], 0U);
    }
    CTIMER1->PWMC = CTIMER_PWMC_PWMEN2_MASK | CTIMER_PWMC_PWMEN1_MASK;
    CTIMER1->TCR  = CTIMER_TCR_CEN_MASK;
}

/* -------------------------------------------------------------- serial io */

static void lpuart_putc(uart_port_t *u, char c)
{
    while (!(u->base->STAT & LPUART_STAT_TDRE_MASK))
    {
    }
    u->base->DATA = (uint8_t)c;
}

static bool lpuart_getc(uart_port_t *u, char *c)
{
    if (u->rx_tail == u->rx_head)
    {
        return false;
    }
    *c         = (char)u->rx_buf[u->rx_tail];
    u->rx_tail = (u->rx_tail + 1U) & (RX_BUF_SIZE - 1U);
    return true;
}

static void lpuart_flush(uart_port_t *u)
{
    while (!(u->base->STAT & LPUART_STAT_TC_MASK))
    {
    }
}

/* Output is dropped while no terminal has the port open (DTR low), and if
 * the host stops reading, so the main loop never blocks on USB. */
static void usb_putc(char c)
{
    static bool stalled;
    uint32_t start = g_ticks;

    while (tud_cdc_connected() && tud_cdc_write_char(c) == 0U)
    {
        if (stalled || (g_ticks - start) >= USB_TX_TIMEOUT_MS)
        {
            stalled = true; /* drop until the FIFO has room again */
            return;
        }
        tud_task(); /* FIFO full: let the stack send it */
        tud_cdc_write_flush();
    }
    stalled = false;
}

static bool usb_getc(char *c)
{
    return tud_cdc_available() && tud_cdc_read(c, 1U) == 1U;
}

static void usb_flush(void)
{
    uint32_t start = g_ticks;

    tud_cdc_write_flush();
    while (tud_cdc_connected() && tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE &&
           (g_ticks - start) < USB_TX_TIMEOUT_MS)
    {
        tud_task();
        tud_cdc_write_flush();
    }
}

/* -------------------------------------------------------------- shells */

/* One command shell per port. Output from a command goes to the shell that
 * is running it (g_out). */
typedef struct
{
    uart_port_t *uart; /* NULL: the USB CDC port */
    char line[LINE_MAX];
    uint32_t len;
    char prev;
} shell_t;

static shell_t g_uart_shell = {.uart = &g_uart0};
static shell_t g_usb_shell  = {.uart = NULL};
static shell_t *g_out       = &g_uart_shell;

static volatile bool g_usb_greet; /* a terminal opened the USB port */

/* ---------------------------------------------------------------- output */

static bool shell_getc(shell_t *sh, char *c)
{
    return sh->uart ? lpuart_getc(sh->uart, c) : usb_getc(c);
}

/* Wait until queued output has left the MCU. */
static void shell_flush(shell_t *sh)
{
    if (sh->uart)
    {
        lpuart_flush(sh->uart);
    }
    else
    {
        usb_flush();
    }
}

static void out_putc(char c)
{
    if (g_out->uart)
    {
        lpuart_putc(g_out->uart, c);
    }
    else
    {
        usb_putc(c);
    }
}

static void out_puts(const char *s)
{
    while (*s)
    {
        if (*s == '\n')
        {
            out_putc('\r');
        }
        out_putc(*s++);
    }
}

static void out_putu(uint32_t v)
{
    char buf[11];
    int i = sizeof(buf) - 1;
    buf[i] = '\0';
    do
    {
        buf[--i] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v != 0U);
    out_puts(&buf[i]);
}

/* Print v / 10^decimals with a fixed number of decimals. */
static void out_putfix(uint32_t v, uint32_t decimals)
{
    uint32_t div = 1;
    for (uint32_t i = 0; i < decimals; i++)
    {
        div *= 10U;
    }
    out_putu(v / div);
    if (decimals)
    {
        uint32_t frac = v % div;
        out_putc('.');
        for (uint32_t d = div / 10U; d > 0U; d /= 10U)
        {
            out_putc((char)('0' + (frac / d) % 10U));
        }
    }
}

/* -------------------------------------------------------------- commands */

static bool parse_u32(const char *s, uint32_t *out)
{
    uint32_t v = 0;
    if (s == NULL || *s == '\0')
    {
        return false;
    }
    for (; *s; s++)
    {
        if (*s < '0' || *s > '9' || v > 429496728U)
        {
            return false;
        }
        v = v * 10U + (uint32_t)(*s - '0');
    }
    *out = v;
    return true;
}

/* Parse "N" or "N.DDD" with up to `decimals` fraction digits, scaled by
 * 10^decimals, e.g. parse_fixed("37.5", 1) -> 375, parse_fixed("4.2", 3) -> 4200. */
static bool parse_fixed(const char *s, uint32_t decimals, uint32_t *out)
{
    char buf[12];
    const char *dot = strchr(s, '.');
    uint32_t whole, frac = 0, scale = 1;

    for (uint32_t i = 0; i < decimals; i++)
    {
        scale *= 10U;
    }

    if (dot == NULL)
    {
        if (!parse_u32(s, &whole))
        {
            return false;
        }
    }
    else
    {
        size_t n      = (size_t)(dot - s);
        size_t nfrac  = strlen(dot + 1);
        if (n == 0U || n >= sizeof(buf) || nfrac == 0U || nfrac > decimals)
        {
            return false;
        }
        memcpy(buf, s, n);
        buf[n] = '\0';
        if (!parse_u32(buf, &whole) || !parse_u32(dot + 1, &frac))
        {
            return false;
        }
        for (size_t i = nfrac; i < decimals; i++)
        {
            frac *= 10U;
        }
    }
    if (whole > 100000U)
    {
        return false;
    }
    *out = whole * scale + frac;
    return true;
}

static bool lookup_color(const char *name, uint32_t *mask)
{
    for (size_t i = 0; i < sizeof(k_colors) / sizeof(k_colors[0]); i++)
    {
        if (strcmp(name, k_colors[i].name) == 0)
        {
            *mask = k_colors[i].mask;
            return true;
        }
    }
    return false;
}

static const char *color_name(uint32_t mask)
{
    for (size_t i = 0; i < sizeof(k_colors) / sizeof(k_colors[0]); i++)
    {
        if (k_colors[i].mask == mask)
        {
            return k_colors[i].name;
        }
    }
    return "?";
}

static void cmd_help(void)
{
    out_puts("commands:\n"
              "  help               this text\n"
              "  led <color>        solid color\n"
              "  blink <color>      blink a color\n"
              "  cycle              cycle red/green/blue\n"
              "  rate <ms>          step period, 10..10000 (default 500)\n"
              "  pwm                show PWM counts and voltage\n"
              "  pwm <0-100>        set duty in %, one decimal ok (37.5)\n"
              "  pwm counts <n>     set high time, 0..1000 counts\n"
              "  ma                 show loop current\n"
              "  ma <mA>            set loop current, e.g. ma 12.5\n"
              "  cal                show 4-20 mA calibration\n"
              "  cal lo <mA>        record meter reading at current output (do at ~4 mA)\n"
              "  cal hi <mA>        record meter reading at current output (do at ~20 mA)\n"
              "  cal reset          back to 0 counts=4 mA, 1000 counts=20 mA\n"
              "  pwm/ma/cal act on channel 1 (P1_4); pwm2/ma2/cal2 on channel 2 (P1_3)\n"
              "  save               store settings in flash (kept over power-off)\n"
              "  defaults           erase saved settings, restore defaults\n"
              "  status             show current state\n"
              "  reset              reboot the MCU\n"
              "colors: red green blue yellow cyan magenta white off\n");
}

static void cmd_pwm_show(const pwm_ch_t *ch);
static void cmd_ma_show(const pwm_ch_t *ch);

/* Name the reset source(s) recorded in CMC->SRS, e.g. "pin" or "power-on". */
static void out_reset_cause(uint32_t srs)
{
    static const struct
    {
        uint32_t mask;
        const char *name;
    } k_causes[] = {
        {CMC_SRS_POR_MASK, "power-on"},
        {CMC_SRS_VD_MASK, "low-voltage"},
        {CMC_SRS_PIN_MASK, "pin"},
        {CMC_SRS_DAP_MASK, "debugger"},
        {CMC_SRS_SW_MASK, "software"},
        {CMC_SRS_LOCKUP_MASK, "lockup"},
        {CMC_SRS_WWDT0_MASK, "watchdog"},
        {CMC_SRS_CDOG0_MASK, "code-watchdog"},
        {CMC_SRS_SCG_MASK, "clock-loss"},
        {CMC_SRS_FATAL_MASK, "fatal"},
        {CMC_SRS_JTAG_MASK, "jtag"},
        {CMC_SRS_RSTACK_MASK, "reset-timeout"},
        {CMC_SRS_LPACK_MASK, "lowpower-timeout"},
        {CMC_SRS_WAKEUP_MASK, "wakeup"},
    };
    bool any = false;

    for (size_t i = 0; i < sizeof(k_causes) / sizeof(k_causes[0]); i++)
    {
        if (srs & k_causes[i].mask)
        {
            if (any)
            {
                out_putc('+');
            }
            out_puts(k_causes[i].name);
            any = true;
        }
    }
    if (!any)
    {
        out_puts("unknown");
    }
}

static void cmd_status(void)
{
    static const char *const mode_names[] = {"cycle", "solid", "blink"};
    out_puts("mode=");
    out_puts(mode_names[g_mode]);
    if (g_mode != MODE_CYCLE)
    {
        out_puts(" color=");
        out_puts(color_name(g_color));
    }
    out_puts(" rate=");
    out_putu(g_period_ms);
    out_puts("ms uptime=");
    out_putu(g_ticks / 1000U);
    out_puts("s reset=");
    out_reset_cause(g_reset_srs);
    out_puts(" core=");
    out_putu(SystemCoreClock);
    out_puts("Hz rx_overruns=");
    out_putu(g_uart0.rx_overruns);
    out_puts(" rx_errors=");
    out_putu(g_uart0.rx_errors);
    out_puts("\n");
    for (uint32_t i = 0; i < PWM_CHANNELS; i++)
    {
        cmd_pwm_show(&g_ch[i]);
        cmd_ma_show(&g_ch[i]);
    }
}

/* "err: usage: <cmd> <rest>" so the message names pwm or pwm2 as typed. */
static void out_usage(const char *cmd, const char *rest)
{
    out_puts("err: usage: ");
    out_puts(cmd);
    out_puts(rest);
}

static void cmd_pwm_show(const pwm_ch_t *ch)
{
    uint32_t mv = (VDD_MV * ch->high + PWM_COUNTS / 2U) / PWM_COUNTS;

    out_puts("pwm");
    out_puts(ch->name);
    out_puts(": ");
    out_putu(ch->high);
    out_puts("/");
    out_putu(PWM_COUNTS);
    out_puts(" counts (");
    out_putfix(ch->high * 1000U / PWM_COUNTS, 1);
    out_puts("%) @ ");
    out_putu(PWM_FREQ_HZ);
    out_puts(" Hz -> ");
    out_putfix(mv, 3);
    out_puts(" V avg on ");
    out_puts(ch->pin);
    out_puts("\n");
}

static void cmd_pwm(pwm_ch_t *ch, int argc, char **argv)
{
    uint32_t val;

    if (argc >= 2 && strcmp(argv[1], "counts") == 0)
    {
        if (argc < 3 || !parse_u32(argv[2], &val) || val > PWM_COUNTS)
        {
            out_usage(argv[0], " counts <0..1000>\n");
            return;
        }
        pwm_set(ch, val);
    }
    else if (argc >= 2)
    {
        if (!parse_fixed(argv[1], 1, &val) || val > 1000U)
        {
            out_usage(argv[0], " <0..100> | ");
            out_puts(argv[0]);
            out_puts(" counts <0..1000>\n");
            return;
        }
        pwm_set(ch, (val * PWM_COUNTS + 500U) / 1000U);
    }
    cmd_pwm_show(ch);
}

/* ------------------------------------------------------------- 4-20 mA */

/* Loop current predicted from PWM counts by the calibration line. */
static int32_t cal_counts_to_ua(const pwm_ch_t *ch, uint32_t counts)
{
    int32_t dc = (int32_t)ch->cal_hi_counts - (int32_t)ch->cal_lo_counts;
    int32_t du = (int32_t)ch->cal_hi_ua - (int32_t)ch->cal_lo_ua;
    int32_t c  = (int32_t)counts - (int32_t)ch->cal_lo_counts;
    return (int32_t)ch->cal_lo_ua + (int32_t)(((int64_t)c * du + (dc / 2)) / dc);
}

/* PWM counts needed for `ua`; false if outside 0..PWM_COUNTS. */
static bool cal_ua_to_counts(const pwm_ch_t *ch, uint32_t ua, uint32_t *counts)
{
    int32_t dc = (int32_t)ch->cal_hi_counts - (int32_t)ch->cal_lo_counts;
    int32_t du = (int32_t)ch->cal_hi_ua - (int32_t)ch->cal_lo_ua;
    int32_t u  = (int32_t)ua - (int32_t)ch->cal_lo_ua;
    int64_t num = (int64_t)u * dc;
    int32_t c   = (int32_t)ch->cal_lo_counts + (int32_t)((num + (num >= 0 ? du / 2 : -du / 2)) / du);

    if (c < 0 || c > (int32_t)PWM_COUNTS)
    {
        return false;
    }
    *counts = (uint32_t)c;
    return true;
}

static bool cal_valid(uint32_t lo_c, uint32_t lo_ua, uint32_t hi_c, uint32_t hi_ua)
{
    return lo_c <= PWM_COUNTS && hi_c <= PWM_COUNTS && hi_c >= lo_c + CAL_MIN_SPAN_COUNTS &&
           hi_ua > lo_ua && hi_ua <= MA_MAX_UA;
}

static void out_ma(int32_t ua)
{
    out_putfix(ua < 0 ? 0U : (uint32_t)ua, 3);
    out_puts(" mA");
}

static void cmd_ma_show(const pwm_ch_t *ch)
{
    out_puts("ma");
    out_puts(ch->name);
    out_puts(": ");
    out_ma(cal_counts_to_ua(ch, ch->high));
    out_puts(" (");
    out_putu(ch->high);
    out_puts("/");
    out_putu(PWM_COUNTS);
    out_puts(" counts)\n");
}

static void cmd_cal_show(const pwm_ch_t *ch)
{
    out_puts("cal");
    out_puts(ch->name);
    out_puts(": ");
    out_putu(ch->cal_lo_counts);
    out_puts(" counts = ");
    out_ma((int32_t)ch->cal_lo_ua);
    out_puts(", ");
    out_putu(ch->cal_hi_counts);
    out_puts(" counts = ");
    out_ma((int32_t)ch->cal_hi_ua);
    if (ch->cal_lo_counts == CAL_LO_COUNTS_DEFAULT && ch->cal_lo_ua == CAL_LO_UA_DEFAULT &&
        ch->cal_hi_counts == CAL_HI_COUNTS_DEFAULT && ch->cal_hi_ua == CAL_HI_UA_DEFAULT)
    {
        out_puts(" (default)");
    }
    out_puts("\n      reachable ");
    out_ma(cal_counts_to_ua(ch, 0U));
    out_puts(" .. ");
    out_ma(cal_counts_to_ua(ch, PWM_COUNTS));
    out_puts(", ");
    out_putu((ch->cal_hi_ua - ch->cal_lo_ua) * 1000U / (ch->cal_hi_counts - ch->cal_lo_counts));
    out_puts(" nA/count\n");
}

static void cmd_ma(pwm_ch_t *ch, int argc, char **argv)
{
    uint32_t ua, counts;

    if (argc >= 2)
    {
        if (!parse_fixed(argv[1], 3, &ua) || ua > MA_MAX_UA)
        {
            out_usage(argv[0], " <0.000..25.000>\n");
            return;
        }
        if (!cal_ua_to_counts(ch, ua, &counts))
        {
            out_puts("err: out of range, reachable ");
            out_ma(cal_counts_to_ua(ch, 0U));
            out_puts(" .. ");
            out_ma(cal_counts_to_ua(ch, PWM_COUNTS));
            out_puts("\n");
            return;
        }
        pwm_set(ch, counts);
    }
    cmd_ma_show(ch);
}

static void cal_reset(pwm_ch_t *ch)
{
    ch->cal_lo_counts = CAL_LO_COUNTS_DEFAULT;
    ch->cal_lo_ua     = CAL_LO_UA_DEFAULT;
    ch->cal_hi_counts = CAL_HI_COUNTS_DEFAULT;
    ch->cal_hi_ua     = CAL_HI_UA_DEFAULT;
}

static void cmd_cal(pwm_ch_t *ch, int argc, char **argv)
{
    uint32_t ua;

    if (argc < 2)
    {
        cmd_cal_show(ch);
        return;
    }
    if (strcmp(argv[1], "reset") == 0)
    {
        cal_reset(ch);
        cmd_cal_show(ch);
        return;
    }

    bool lo = strcmp(argv[1], "lo") == 0;
    if ((!lo && strcmp(argv[1], "hi") != 0) || argc < 3 || !parse_fixed(argv[2], 3, &ua) || ua > MA_MAX_UA)
    {
        out_usage(argv[0], " lo|hi <measured mA> | ");
        out_puts(argv[0]);
        out_puts(" reset\n");
        return;
    }

    uint32_t lo_c = lo ? ch->high : ch->cal_lo_counts;
    uint32_t lo_u = lo ? ua : ch->cal_lo_ua;
    uint32_t hi_c = lo ? ch->cal_hi_counts : ch->high;
    uint32_t hi_u = lo ? ch->cal_hi_ua : ua;
    if (!cal_valid(lo_c, lo_u, hi_c, hi_u))
    {
        out_puts("err: lo and hi points must be at least 100 counts apart, hi above lo\n");
        return;
    }
    ch->cal_lo_counts = lo_c;
    ch->cal_lo_ua     = lo_u;
    ch->cal_hi_counts = hi_c;
    ch->cal_hi_ua     = hi_u;
    cmd_cal_show(ch);
}

static void set_mode(led_mode_t mode, uint32_t color);

static void apply_defaults(void)
{
    g_period_ms = 500;
    for (uint32_t i = 0; i < PWM_CHANNELS; i++)
    {
        cal_reset(&g_ch[i]);
        pwm_set(&g_ch[i], 0U);
    }
    set_mode(MODE_CYCLE, 0);
}

static void state_to_settings(settings_t *st)
{
    st->mode      = (uint32_t)g_mode;
    st->color     = g_color;
    st->period_ms = g_period_ms;
    for (uint32_t i = 0; i < PWM_CHANNELS; i++)
    {
        st->ch[i] = (settings_channel_t){
            .pwm_high      = g_ch[i].high,
            .cal_lo_counts = g_ch[i].cal_lo_counts,
            .cal_lo_ua     = g_ch[i].cal_lo_ua,
            .cal_hi_counts = g_ch[i].cal_hi_counts,
            .cal_hi_ua     = g_ch[i].cal_hi_ua,
        };
    }
}

/* Restore saved settings; falls back to defaults if none or out of range. */
static bool apply_saved(void)
{
    settings_t st;
    state_to_settings(&st); /* a record from older firmware only fills channel 1 */
    if (!settings_load(&st) || st.mode > MODE_BLINK || st.period_ms < 10U || st.period_ms > 10000U ||
        (st.color & ~LED_ALL) != 0U)
    {
        return false;
    }
    for (uint32_t i = 0; i < PWM_CHANNELS; i++)
    {
        const settings_channel_t *c = &st.ch[i];
        if (c->pwm_high > PWM_COUNTS || !cal_valid(c->cal_lo_counts, c->cal_lo_ua, c->cal_hi_counts, c->cal_hi_ua))
        {
            return false;
        }
    }
    g_period_ms = st.period_ms;
    for (uint32_t i = 0; i < PWM_CHANNELS; i++)
    {
        g_ch[i].cal_lo_counts = st.ch[i].cal_lo_counts;
        g_ch[i].cal_lo_ua     = st.ch[i].cal_lo_ua;
        g_ch[i].cal_hi_counts = st.ch[i].cal_hi_counts;
        g_ch[i].cal_hi_ua     = st.ch[i].cal_hi_ua;
        pwm_set(&g_ch[i], st.ch[i].pwm_high);
    }
    set_mode((led_mode_t)st.mode, st.color);
    return true;
}

static void print_flash_result(int32_t st)
{
    if (st == 0)
    {
        out_puts("ok (");
        out_putu(settings_used_slots());
        out_puts("/");
        out_putu(SETTINGS_SLOTS);
        out_puts(" slots used)\n");
    }
    else
    {
        out_puts("err: flash status ");
        out_putu((uint32_t)st);
        out_puts("\n");
    }
}

static void cmd_save(void)
{
    settings_t st;
    state_to_settings(&st);
    print_flash_result(settings_save(&st));
}

static void cmd_defaults(void)
{
    apply_defaults();
    print_flash_result(settings_erase());
}

static void set_mode(led_mode_t mode, uint32_t color)
{
    g_mode      = mode;
    g_color     = color;
    g_step      = 0;
    g_last_step = g_ticks;
    leds_set(mode == MODE_CYCLE ? LED_RED : color);
}

/* "pwm", "pwm1" -> channel 1, "pwm2" -> channel 2; NULL if cmd isn't base[N]. */
static pwm_ch_t *channel_cmd(const char *cmd, const char *base)
{
    size_t n = strlen(base);
    if (strncmp(cmd, base, n) != 0)
    {
        return NULL;
    }
    if (cmd[n] == '\0')
    {
        return &g_ch[0];
    }
    for (uint32_t i = 0; i < PWM_CHANNELS; i++)
    {
        if (strcmp(cmd + n, g_ch[i].name) == 0)
        {
            return &g_ch[i];
        }
    }
    return NULL;
}

static void run_command(char *line)
{
    char *argv[3];
    int argc = 0;

    for (char *tok = strtok(line, " \t"); tok && argc < 3; tok = strtok(NULL, " \t"))
    {
        argv[argc++] = tok;
    }
    if (argc == 0)
    {
        return;
    }

    uint32_t val;
    pwm_ch_t *ch;
    if (strcmp(argv[0], "help") == 0 || strcmp(argv[0], "?") == 0)
    {
        cmd_help();
    }
    else if (strcmp(argv[0], "status") == 0)
    {
        cmd_status();
    }
    else if (strcmp(argv[0], "cycle") == 0)
    {
        set_mode(MODE_CYCLE, 0);
        out_puts("ok\n");
    }
    else if (strcmp(argv[0], "led") == 0 || strcmp(argv[0], "blink") == 0)
    {
        if (argc < 2 || !lookup_color(argv[1], &val))
        {
            out_puts("err: usage: ");
            out_puts(argv[0]);
            out_puts(" <red|green|blue|yellow|cyan|magenta|white|off>\n");
            return;
        }
        set_mode(argv[0][0] == 'l' ? MODE_SOLID : MODE_BLINK, val);
        out_puts("ok\n");
    }
    else if ((ch = channel_cmd(argv[0], "pwm")) != NULL)
    {
        cmd_pwm(ch, argc, argv);
    }
    else if ((ch = channel_cmd(argv[0], "ma")) != NULL)
    {
        cmd_ma(ch, argc, argv);
    }
    else if ((ch = channel_cmd(argv[0], "cal")) != NULL)
    {
        cmd_cal(ch, argc, argv);
    }
    else if (strcmp(argv[0], "save") == 0)
    {
        cmd_save();
    }
    else if (strcmp(argv[0], "defaults") == 0)
    {
        cmd_defaults();
    }
    else if (strcmp(argv[0], "rate") == 0)
    {
        if (argc < 2 || !parse_u32(argv[1], &val) || val < 10U || val > 10000U)
        {
            out_puts("err: usage: rate <10..10000>\n");
            return;
        }
        g_period_ms = val;
        out_puts("ok\n");
    }
    else if (strcmp(argv[0], "reset") == 0)
    {
        out_puts("resetting\n");
        shell_flush(g_out);
        NVIC_SystemReset();
    }
    else
    {
        out_puts("err: unknown command '");
        out_puts(argv[0]);
        out_puts("', try 'help'\n");
    }
}

/* Line editor: echo, backspace, CR or LF terminates. */
static void shell_poll(shell_t *sh)
{
    char c;

    g_out = sh;
    while (shell_getc(sh, &c))
    {
        if (c == '\r' || c == '\n')
        {
            bool crlf = (c == '\n' && sh->prev == '\r');
            sh->prev  = c;
            if (crlf)
            {
                continue; /* second half of CRLF */
            }
            out_puts("\n");
            sh->line[sh->len] = '\0';
            run_command(sh->line);
            sh->len = 0;
            out_puts("> ");
            continue;
        }
        sh->prev = c;

        if (c == '\b' || c == 0x7F)
        {
            if (sh->len > 0U)
            {
                sh->len--;
                out_puts("\b \b");
            }
        }
        else if (c >= ' ' && c <= '~' && sh->len < LINE_MAX - 1U)
        {
            sh->line[sh->len++] = c;
            out_putc(c);
        }
    }
}

static void shell_banner(shell_t *sh)
{
    g_out = sh;
    out_puts("\nFRDM-MCXA153 shell ready. Type 'help'.\n");
    out_puts(g_restored ? "settings: restored from flash\n" : "settings: defaults\n");
    out_puts("last reset: ");
    out_reset_cause(g_reset_srs);
    out_puts("\n> ");
}

/* USB CDC shell: greet each time a terminal opens the port. */
static void usb_shell_poll(void)
{
    if (g_usb_greet)
    {
        g_usb_greet     = false;
        g_usb_shell.len = 0U;
        tud_cdc_read_flush(); /* drop anything typed before the terminal opened */
        shell_banner(&g_usb_shell);
    }
    shell_poll(&g_usb_shell);
    tud_cdc_write_flush();
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    (void)rts;
    if (dtr)
    {
        g_usb_greet = true;
    }
}

/* ---------------------------------------------------------------- leds */

static void leds_poll(void)
{
    static const uint32_t sequence[] = {LED_RED, LED_GREEN, LED_BLUE};

    if ((g_ticks - g_last_step) < g_period_ms)
    {
        return;
    }
    g_last_step += g_period_ms;
    g_step++;

    switch (g_mode)
    {
        case MODE_CYCLE:
            leds_set(sequence[g_step % 3U]);
            break;
        case MODE_BLINK:
            leds_set((g_step & 1U) ? 0U : g_color);
            break;
        case MODE_SOLID:
        default:
            break;
    }
}

int main(void)
{
    g_reset_srs = CMC->SRS;
    clocks_init();
    leds_init();
    uart_init();
    pwm_init();
    SystemCoreClock = core_clock_hz();
    SysTick_Config(SystemCoreClock / 1000U);

    g_restored = apply_saved();
    if (!g_restored)
    {
        apply_defaults();
    }
    shell_banner(&g_uart_shell);

    tusb_rhport_init_t usb_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    tusb_init(0, &usb_init);

    for (;;)
    {
        tud_task();
        shell_poll(&g_uart_shell);
        usb_shell_poll();
        leds_poll();
        __WFI(); /* woken by SysTick, UART RX or USB */
    }
}
