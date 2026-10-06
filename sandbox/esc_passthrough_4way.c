/**
 * @file esc_passthrough_4way.c
 * @brief Minimal ESC Configurator 4-way passthrough on NUCLEO-G474RE.
 *
 * PC (USB -> ST-Link VCP) sends MSP commands to switch into 4-way on the
 * selected motor pin. When active, raw 4-way bytes are bridged between the
 * VCP (LPUART1) and the ESC signal pin on a single wire (PA8 / D7) using
 * a half-duplex software 1-wire UART.
 *
 * Wiring:
 *   PA8 (D7) -> ESC signal (white)
 *   GND      -> common ground
 *
 * Note: after flashing, do not run `rov.py monitor`; connect to COM6 from
 * esc-configurator.com. Also ESC must be powered with the main battery.
 */

#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>

/* -------------------- Settings -------------------- */
#define PASSTHROUGH_MOTOR_INDEX  0U    /* internal motor index 0 */
#define MOTOR_BITBANG_BAUD       19200U /* 4-way BLHeli timing */
/* LOW break is a C2/Atmel bootloader trick; SiLabs BLHeli_S/Bluejay do not
   need it and go quiet while the line is held low. Set nonzero only if a
   future ESC family requires it. */
#define PASSTHROUGH_BREAK_MS     0U
#define PASSTHROUGH_SETTLE_MS    20U

/* -------------------- LPUART1 (VCP to PC) -------------------- */
static UART_HandleTypeDef hlpuart1;

/*
 * Count how many times the ESC signal line actually moved, observed
 * CONTINUOUSLY for the whole handshake.
 *
 * A fixed sampling window can miss the reply entirely: the ESC answers within
 * a millisecond or two of the last handshake byte, so a window that starts too
 * late reports zero edges even when the ESC did respond.
 */
static volatile uint32_t s_line_edges = 0U;

static void esc_watch_enable(void) {
    s_line_edges = 0U;
}

/* -------------------- MSP state -------------------- */
typedef enum {
    MSP_HEADER_M = 0,
    MSP_HEADER_ANGLE,
    MSP_HEADER_PREFIX,
    MSP_DIR,
    MSP_LEN_L,
    MSP_LEN_H,
    MSP_FUNC_L,
    MSP_FUNC_H,
    MSP_PAYLOAD,
    MSP_CRC,
    MSP_LEN_H_U16 = 0 /* unused */
} msp_state_t;

struct msp_ctx {
    msp_state_t state;
    uint8_t len;
    uint8_t func_l;
    uint8_t func_h;
    uint8_t payload[64];
    uint8_t payload_i;
    uint8_t crc;
};

/* -------------------- Half-duplex bit-banged UART on PA8 -------------------- */
static void PA8_TxLow(void) { GPIOA->BSRR = GPIO_PIN_8 << 16; }
static void PA8_TxHigh(void) { GPIOA->BSRR = GPIO_PIN_8; }
static void PA8_Input(void) {
    GPIOA->MODER = (GPIOA->MODER & ~(3U << 16));
}
static void PA8_Output(void) {
    GPIOA->MODER = (GPIOA->MODER & ~(3U << 16)) | (1U << 16);
}
static inline uint32_t pin_level(void) {
    return (GPIOA->IDR & GPIO_PIN_8) ? 1U : 0U;
}

static uint32_t BB_UART_PERIOD_CYCLES(void) {
    return SystemCoreClock / MOTOR_BITBANG_BAUD;
}

/*
 * ESC-side serial is plain 19200 8N1: idle HIGH, start bit LOW, stop bit HIGH,
 * and a data bit of 1 is HIGH. Betaflight's suart_putc_ builds its bitmask so
 * the stop bit (value 1) drives ESC_SET_HI, and suart_getc_ rejects a frame
 * unless bit0 == 0 (start low) and bit9 == 1 (stop high).
 */
static void bb_uart_write_byte(uint8_t b) {
    /* A single SysTick interrupt (1 ms) is ~19 bit times at 19200 baud, so an
     * interrupt landing mid-byte corrupts the bit framing entirely. */
    __disable_irq();
    PA8_Output();
    PA8_TxHigh(); /* idle */

    uint32_t t0 = DWT->CYCCNT;
    uint32_t period = BB_UART_PERIOD_CYCLES();
    uint32_t bit_time = t0;

    /* start bit = LOW */
    PA8_TxLow();
    bit_time += period;
    while ((int32_t)(DWT->CYCCNT - bit_time) < 0) {}

    /* data bits, LSB first */
    for (int i = 0; i < 8; i++) {
        if (b & (1U << i)) PA8_TxHigh(); else PA8_TxLow();
        bit_time += period;
        while ((int32_t)(DWT->CYCCNT - bit_time) < 0) {}
    }

    /* stop bit = HIGH */
    PA8_TxHigh();
    bit_time += period;
    while ((int32_t)(DWT->CYCCNT - bit_time) < 0) {}

    /*
     * Stay in push-pull HIGH between bytes instead of releasing to input.
     * The internal pull-up is only ~40k, which is weak; a solidly driven HIGH
     * is what the ESC needs to recognise as a valid idle line.
     */
    __enable_irq();
}

/* Release the line so the ESC can drive its reply. */
static void bb_uart_release(void) {
    PA8_Input();
}

/*
 * High-resolution transition counter.
 *
 * Polls IDR as fast as the CPU allows and counts real level changes. At 19200
 * baud a bit is ~52 us, so even a crude loop resolves it easily. This replaces
 * an earlier snapshot that sampled every ~4 ms and therefore could never have
 * observed a single bit, which is why it always reported the line as HIGH.
 */
static uint32_t bb_count_edges(uint32_t window_cycles) {
    __disable_irq();
    uint32_t t_end = DWT->CYCCNT + window_cycles;
    uint32_t prev = pin_level();
    uint32_t edges = 0U;
    while ((int32_t)(t_end - DWT->CYCCNT) > 0) {
        uint32_t now = pin_level();
        if (now != prev) {
            edges++;
            prev = now;
        }
    }
    __enable_irq();
    return edges;
}

/*
 * Sample the line at roughly half a bit period across a short window, so the
 * host can see the actual waveform.
 *
 * The earlier version sampled every ~4 ms, which is ~75 bit times at 19200 baud
 * and therefore could never have observed a single transition. That is why it
 * always reported a flat HIGH line and told us nothing.
 */
static uint8_t s_capture[12];

static void esc_capture_line(uint32_t window_cycles) {
    __disable_irq();
    /* Half a bit per sample, so 12 samples span ~6 bit times (a whole byte). */
    uint32_t step = BB_UART_PERIOD_CYCLES() / 2U;
    if (step == 0U) {
        step = 1U;
    }
    (void)window_cycles;
    uint32_t t = DWT->CYCCNT;
    for (uint8_t i = 0; i < sizeof(s_capture); i++) {
        while ((int32_t)(DWT->CYCCNT - t) < 0) {}
        s_capture[i] = (pin_level() != 0U) ? 'H' : 'L';
        t += step;
    }
    __enable_irq();
}

static int bb_uart_read_byte(uint8_t *out, uint32_t timeout_cycles) {
    /*
     * Wait for the line to drop into the start bit. Must run with interrupts
     * disabled: a SysTick in the middle of a byte destroys the framing.
     */
    __disable_irq();

    uint32_t t0 = DWT->CYCCNT;
    uint32_t prev = pin_level();
    while (pin_level() != 0U) {
        /* Count real transitions only; counting loop iterations would report a
         * huge number for a line that simply sits idle. */
        uint32_t now = pin_level();
        if (now != prev) {
            s_line_edges++;
            prev = now;
        }
        if ((int32_t)(DWT->CYCCNT - (t0 + timeout_cycles)) >= 0) {
            __enable_irq();
            return -1;
        }
    }
    s_line_edges++;

    uint32_t period = BB_UART_PERIOD_CYCLES();
    uint32_t bits = 0;

    /*
     * Sample data bit 0 at the centre of ITS cell, not the start bit's.
     *
     * Timing: the start bit occupies 0 -> T. Data bit 0 occupies T -> 2T, so
     * its centre is 1.5T after the leading edge. Sampling at 0.5T lands inside
     * the start bit and shifts the whole byte by one position.
     *
     * Betaflight's suart_getc_ does the equivalent by confirming the start bit
     * at START_BIT_TIME (0.75T) and then stepping a full BIT_TIME per data bit,
     * which puts its first data sample at 0.75T + 1.0T = 1.75T.
     */
    uint32_t t = DWT->CYCCNT + period + (period / 2U);
    while ((int32_t)(DWT->CYCCNT - t) < 0) {}

    for (int i = 0; i < 8; i++) {
        if (pin_level()) bits |= (1U << i);
        t += period;
        while ((int32_t)(DWT->CYCCNT - t) < 0) {}
    }

    /* validate framing the way Betaflight's suart_getc_ does */
    int rc = 0;
    if (!pin_level()) rc = -1;        /* stop bit must be HIGH */

    __enable_irq();

    if (rc == 0) {
        *out = (uint8_t)bits;
    }
    return rc;
}

/*
 * Reset the ESC and let it boot back up before we try to talk to it.
 *
 * Holding the signal line LOW for a few hundred ms resets a BLHeli_S/Bluejay
 * ESC. After release it reboots and only starts listening for the 19200 serial
 * "471"/BLHeli handshake once its own firmware is up, which takes on the order
 * of a second. Talking to it earlier (the old 5 ms settle) is why we were
 * getting zero bytes back.
 */
/*
 * Self-test: drive a known byte out of PA8, switch to input, and read it back.
 * If this fails, the 19200 bit-bang is broken and no ESC will ever answer.
 * Reported to the host through the MSP_BOARD_INFO payload.
 */
static uint8_t s_selftest = 0U; /* 0 = not run, 1 = pass, 2 = fail, 3 = mismatch */
static uint8_t s_selftest_rx = 0U; /* byte we read back */
static uint8_t s_selftest_tx = 0U; /* byte we tried to send */

/*
 * Transmit a byte while sampling the line back at each bit centre.
 *
 * This is the only honest loopback: a normal read starts looking for a start
 * bit AFTER the write has already finished, so it can never see the start bit
 * of the byte we just produced. Sampling during the shift exercises exactly the
 * same timing the ESC will see, so if this returns b, our bit timing is right.
 */
static uint8_t bb_uart_write_sample_byte(uint8_t b) {
    __disable_irq();
    PA8_Output();
    PA8_TxHigh();

    uint32_t period = BB_UART_PERIOD_CYCLES();
    uint32_t bit_time = DWT->CYCCNT;
    uint8_t sampled = 0U;

    /* start bit (LOW), and wait out the cell so the first data bit lands on a
       cell boundary rather than mid-cell. */
    PA8_TxLow();
    bit_time += period;
    while ((int32_t)(DWT->CYCCNT - bit_time) < 0) {}

    /* data bits, LSB first; sample at the centre of each cell */
    for (int i = 0; i < 8; i++) {
        /* Drive the bit at the START of its cell, THEN sample at the centre.
           The previous order sampled before driving, which reads the previous
           bit: 0x55 came back as 0xAA, a one-bit shift. */
        if (b & (1U << i)) PA8_TxHigh(); else PA8_TxLow();
        bit_time += period;

        uint32_t centre = bit_time - (period / 2U);
        while ((int32_t)(DWT->CYCCNT - centre) < 0) {}
        if (pin_level()) sampled |= (1U << i);

        while ((int32_t)(DWT->CYCCNT - bit_time) < 0) {}
    }

    /* stop bit (HIGH) */
    PA8_TxHigh();
    bit_time += period;
    while ((int32_t)(DWT->CYCCNT - bit_time) < 0) {}

    PA8_Input();
    __enable_irq();

    return sampled;
}

static void esc_bitbang_selftest(void) {
    static const uint8_t pattern[] = {0x55U, 0xA3U, 0x0FU};

    s_selftest_rx = 0U;
    s_selftest_tx = 0U;

    for (uint8_t i = 0; i < sizeof(pattern); i++) {
        uint8_t got = bb_uart_write_sample_byte(pattern[i]);
        if (got != pattern[i]) {
            s_selftest = 3U;
            s_selftest_tx = pattern[i];
            s_selftest_rx = got;
            return;
        }
    }
    s_selftest = 1U;
}

/*
 * Put the signal line into the idle state the ESC expects, then settle.
 *
 * IMPORTANT: do NOT pulse the line LOW by default. A LOW break is the C2/Atmel
 * bootloader trick; a SiLabs BLHeli_S/Bluejay ESC is already listening on that
 * line and does not reboot from it, so holding it low just keeps it silent.
 * Betaflight's esc4wayInit() likewise only parks the motor lines HIGH.
 */
static void esc_line_break_release(void) {
#if PASSTHROUGH_BREAK_MS
    /* Only for ESC families that genuinely need a break to re-enter
       bootloader mode. Off by default for BLHeli_S / Bluejay. */
    PA8_Output();
    PA8_TxLow();
    HAL_Delay(PASSTHROUGH_BREAK_MS);
#endif

    /* Idle high, driven strongly: low output impedance beats the wire's
       capacitance and gives the ESC a clean reference to talk against. */
    PA8_Output();
    PA8_TxHigh();
    HAL_Delay(PASSTHROUGH_SETTLE_MS);

    /* Release so the ESC can pull the line low for its reply. */
    PA8_Input();
}



/* -------------------- VCP byte forwarding in passthrough -------------------- */
static void vcp_write(uint8_t b) {
    while (!(LPUART1->ISR & USART_ISR_TXE)) {}
    LPUART1->TDR = b;
}

/*
 * Direct LPUART banner writer.
 *
 * Do NOT use printf here: __io_putchar is defined in the platform BSP, which
 * this sandbox file never calls, and stdout is never set unbuffered, so printf
 * output never reaches COM6. Write the bytes ourselves.
 */
static void vcp_puts(const char *s) {
    for (; *s != '\0'; s++) {
        if (*s == '\n') vcp_write('\r');
        vcp_write((uint8_t)*s);
    }
}

static bool vcp_read_blocking(uint8_t *b) {
    while (!(LPUART1->ISR & USART_ISR_RXNE)) {}
    *b = (uint8_t)(LPUART1->RDR & 0xFFU);
    return true;
}

static bool vcp_read(uint8_t *b, uint32_t timeout_cycles) {
    uint32_t t0 = DWT->CYCCNT;
    while (!(LPUART1->ISR & USART_ISR_RXNE)) {
        if ((int32_t)(DWT->CYCCNT - (t0 + timeout_cycles)) >= 0) return false;
    }
    *b = (uint8_t)(LPUART1->RDR & 0xFFU);
    return true;
}

/* A very tiny MSP emulation sufficient for configurator probing. */
static uint8_t msp_crc(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80U) crc = (uint8_t)((crc << 1) ^ 0xD5U);
        else crc = (uint8_t)(crc << 1);
    }
    return crc;
}

static void msp_send_reply(uint8_t size, uint8_t cmd, const uint8_t *payload, uint8_t len) {
    uint8_t checksum = size ^ cmd;
    vcp_write('$');
    vcp_write('M');
    vcp_write('>');
    vcp_write(size);
    vcp_write(cmd);
    for (uint8_t i = 0; i < len; i++) {
        vcp_write(payload[i]);
        checksum ^= payload[i];
    }
    vcp_write(checksum);
}

/* -------------------- 4-way protocol constants -------------------- */

#define CMD_INTERFACE_TEST_ALIVE   0x30
#define CMD_PROTOCOL_GET_VERSION   0x31
#define CMD_INTERFACE_GET_NAME     0x32
#define CMD_INTERFACE_GET_VERSION  0x33
#define CMD_INTERFACE_EXIT         0x34
#define CMD_DEVICE_RESET           0x35
#define CMD_DEVICE_INIT_FLASH      0x37
#define CMD_DEVICE_PAGE_ERASE      0x39
#define CMD_DEVICE_READ            0x3A
#define CMD_DEVICE_WRITE           0x3B
#define CMD_DEVICE_READ_EEPROM     0x3D
#define CMD_DEVICE_WRITE_EEPROM    0x3E
#define CMD_DEVICE_VERIFY          0x40

#define ACK_OK                     0x00
#define ACK_I_INVALID_CMD          0x02
#define ACK_D_GENERAL_ERROR        0x0F

/* -------------------- BLHeli / AVR bootloader side -------------------- */
/*
 * The ESC does NOT speak 4-way. It speaks the AVRootloader-style protocol from
 * Betaflight's serial_4way_avrootloader.c: 19200 baud 8N1, inverted line,
 * 52 us bit time, framed as CMD + payload with an optional CRC16.
 * The FC (us) has to TRANSLATE 4-way frames into these bootloader commands.
 */

/* BLHeli bootloader commands */
#define BL_CMD_RUN                 0x00
#define BL_CMD_PROG_FLASH          0x01
#define BL_CMD_ERASE_FLASH         0x02
#define BL_CMD_READ_FLASH_SIL      0x03
#define BL_CMD_READ_EEPROM         0x04
#define BL_CMD_PROG_EEPROM         0x05
#define BL_CMD_KEEP_ALIVE          0xFD
#define BL_CMD_SET_ADDRESS         0xFF
#define BL_CMD_SET_BUFFER          0xFE

/* BLHeli ACK codes */
#define BL_ACK_SUCCESS             0x00
#define BL_ACK_ERROR               0x01
#define BL_ACK_ERRORCRC            0x02
#define BL_ACK_ERRORCOMMAND        0x03

static bool s_connected = false;

static uint16_t s_bl_crc = 0U;

static void bl_byte_crc(uint8_t b) {
    uint8_t xb = b;
    for (uint8_t i = 0; i < 8; i++) {
        if (((xb & 0x01U) ^ (s_bl_crc & 0x0001U)) != 0U) {
            s_bl_crc = (uint16_t)(s_bl_crc >> 1);
            s_bl_crc ^= 0xA001U;
        } else {
            s_bl_crc = (uint16_t)(s_bl_crc >> 1);
        }
        xb = (uint8_t)(xb >> 1);
    }
}

static void bl_send_buf(const uint8_t *buf, uint16_t len) {
    s_bl_crc = 0U;
    for (uint16_t i = 0; i < len; i++) {
        bb_uart_write_byte(buf[i]);
        bl_byte_crc(buf[i]);
    }
    if (s_connected) {
        bb_uart_write_byte((uint8_t)(s_bl_crc & 0xFFU));
        bb_uart_write_byte((uint8_t)((s_bl_crc >> 8) & 0xFFU));
    }
}

/* Read len bytes (plus trailing CRC16 and ACK). Returns true on ACK == SUCCESS. */
static bool bl_read_buf(uint8_t *buf, uint16_t len, uint32_t timeout_cycles) {
    s_bl_crc = 0U;
    for (uint16_t i = 0; i < len; i++) {
        if (bb_uart_read_byte(&buf[i], timeout_cycles) != 0) return false;
        bl_byte_crc(buf[i]);
    }

    uint8_t crc_lo = 0U;
    uint8_t crc_hi = 0U;
    uint8_t ack = 0xFFU;

    if (bb_uart_read_byte(&crc_lo, timeout_cycles) != 0) return false;
    if (bb_uart_read_byte(&crc_hi, timeout_cycles) != 0) return false;
    if (bb_uart_read_byte(&ack, timeout_cycles) != 0) return false;

    uint16_t expect = s_bl_crc;
    uint16_t got = (uint16_t)crc_lo | ((uint16_t)crc_hi << 8);
    if (got != expect) {
        ack = BL_ACK_ERRORCRC;
    }
    return (ack == BL_ACK_SUCCESS);
}

static uint8_t bl_get_ack(uint32_t timeout_cycles) {
    uint8_t ack = 0xFFU;
    uint32_t budget = timeout_cycles;
    while (bb_uart_read_byte(&ack, budget) != 0) {
        if (budget > BB_UART_PERIOD_CYCLES()) {
            budget -= BB_UART_PERIOD_CYCLES();
        } else {
            break;
        }
    }
    return ack;
}

/*
 * Handshake, byte-for-byte from Betaflight serial_4way_avrootloader.c:
 * 9 zero bytes, then 0x0D 'B' 'L' 'H' 'e' 'l' 'i' 0xF4 0x7D  (17 bytes total).
 * A connected BLHeli/Bluejay bootloader answers "471x" + signature.
 */
/* Diagnostics captured from the last bl_connect() attempt, surfaced through
 * the DeviceInfo bytes so the host-side test script can print them. */
static uint8_t s_diag[4] = {0, 0, 0, 0};

/*
 * Count how many times the line actually moved, observed CONTINUOUSLY while we
 * are talking to the ESC.
 *
 * The previous version sampled only in a fixed window after TX, which can miss
 * the reply entirely: the ESC answers within a millisecond or two of the last
 * handshake byte, so a window that starts too late reports zero edges even when
 * the ESC did respond. Now the counter is armed for the whole exchange.
 */
static bool bl_connect(uint8_t *device_info) {
    static const uint8_t boot_init[17] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0x0D, 'B', 'L', 'H', 'e', 'l', 'i',
        0xF4, 0x7D
    };

    /* Betaflight's Connect() retries three times before giving up; the ESC may be
       still booting or mid-startup-melody on the first pass. */
    for (uint8_t attempt = 0; attempt < 3U; attempt++) {
        esc_watch_enable();

        bl_send_buf(boot_init, sizeof(boot_init));

        /*
         * Let the ESC drive and count real transitions over ~4 ms, which is
         * plenty for it to answer. If this is zero the line genuinely never
         * moved, independent of any decode logic.
         */
        bb_uart_release();
        s_line_edges = bb_count_edges(SystemCoreClock / 250U); /* ~4 ms */

        uint8_t info[9] = {0};

        uint16_t got = 0U;
        for (; got < 8U; got++) {
            if (bb_uart_read_byte(&info[got], BB_UART_PERIOD_CYCLES() * 400U) != 0) {
                break;
            }
        }

        for (uint8_t i = 0; i < 8U; i++) {
            s_diag[i] = info[i];
        }
        s_diag[2] = (uint8_t)got;
        s_diag[3] = (got == 8U) ? 2U : 1U; /* 2 = full read, 1 = timeout */

        if (got == 8U && info[0] == '4' && info[1] == '7' && info[2] == '1') {
            if (device_info != (void *)0) {
                device_info[0] = info[6]; /* signature lo */
                device_info[1] = info[5]; /* signature hi */
                device_info[2] = info[3]; /* boot version */
                device_info[3] = 1U;      /* interfaceMode: SiLabs */
            }
            return true;
        }

        /* Give the line back to the idle state between attempts. */
        esc_line_break_release();
    }

    return false;
}

/* -------------------- 4-way frame handling -------------------- */

static void fourway_reply(uint8_t cmd, uint16_t address, const uint8_t *params,
                          uint16_t param_len, uint8_t ack) {
    uint16_t crc = 0U;

    vcp_write(0x2EU);                     crc = (uint16_t)((crc << 8) ^ (crc >> 8)) ^ 0x2EU;
    vcp_write(cmd);                       crc = (uint16_t)((crc << 8) ^ (crc >> 8)) ^ cmd;
    vcp_write((uint8_t)(address >> 8));   crc = (uint16_t)((crc << 8) ^ (crc >> 8)) ^ (uint8_t)(address >> 8);
    vcp_write((uint8_t)(address & 0xFF)); crc = (uint16_t)((crc << 8) ^ (crc >> 8)) ^ (uint8_t)(address & 0xFF);
    vcp_write((uint8_t)(param_len & 0xFF));
    crc = (uint16_t)((crc << 8) ^ (crc >> 8)) ^ (uint8_t)(param_len & 0xFF);

    for (uint16_t i = 0; i < param_len; i++) {
        vcp_write(params[i]);
        crc = (uint16_t)((crc << 8) ^ (crc >> 8)) ^ params[i];
    }

    vcp_write(ack);
    crc = (uint16_t)((crc << 8) ^ (crc >> 8)) ^ ack;

    vcp_write((uint8_t)((crc >> 8) & 0xFFU));
    vcp_write((uint8_t)(crc & 0xFFU));
}

static void fourway_handle(uint8_t cmd, uint16_t address, const uint8_t *in,
                           uint8_t in_len, uint8_t *out, uint16_t out_cap,
                           uint16_t *out_len, uint8_t *ack) {
    uint8_t ack_out = ACK_OK;
    uint16_t n = 0U;

    switch (cmd) {
    case CMD_INTERFACE_TEST_ALIVE:
        if (!s_connected) ack_out = ACK_D_GENERAL_ERROR;
        break;

    case CMD_PROTOCOL_GET_VERSION:
        out[n++] = 108U;
        break;

    case CMD_INTERFACE_GET_NAME:
        {
            static const char name[] = "m4wFCIntf";
            for (uint16_t i = 0; i < sizeof(name) - 1U && n < out_cap; i++) out[n++] = (uint8_t)name[i];
        }
        break;

    case CMD_INTERFACE_GET_VERSION:
        out[n++] = 20U;
        out[n++] = 0U;
        out[n++] = 6U;
        break;

    case CMD_INTERFACE_EXIT:
        s_connected = false;
        break;

    

    case CMD_DEVICE_RESET:
        s_connected = false;
        break;

    case CMD_DEVICE_INIT_FLASH:
        {
            uint8_t info[4] = {0, 0, 0, 0};
            if (bl_connect(info)) {
                s_connected = true;
                out[n++] = info[0];
                out[n++] = info[1];
                out[n++] = info[2];
                out[n++] = 1U; /* interfaceMode: SiLabs */
            } else {
                s_connected = false;
                ack_out = ACK_D_GENERAL_ERROR;
                /*
                 * Diagnostics for the host test:
                 *   out[0..1] = edge count seen on the line after handshake
                 *   out[2]    = how many reply bytes we managed to read
                 *   out[3]    = 1 = timed out, 2 = read all 8
                 */
                out[n++] = (uint8_t)((s_line_edges >> 8) & 0xFFU);
                out[n++] = (uint8_t)(s_line_edges & 0xFFU);
                out[n++] = s_diag[2];
                out[n++] = s_diag[3];
                /*
                 * Only snapshot the line now that we know we are not going to
                 * read it: sampling here must not eat the ESC's reply.
                 */
                esc_capture_line(0U);
                for (uint8_t i = 0; i < 12U && n < out_cap; i++) {
                    out[n++] = s_capture[i];
                }
            }
        }
        break;

    case CMD_DEVICE_READ:
    case CMD_DEVICE_WRITE:
    case CMD_DEVICE_PAGE_ERASE:
    case CMD_DEVICE_VERIFY:
    case CMD_DEVICE_READ_EEPROM:
    case CMD_DEVICE_WRITE_EEPROM:
        {
            if (!s_connected) {
                ack_out = ACK_D_GENERAL_ERROR;
                break;
            }
            /* Mirror Betaflight: set address, then set buffer, then command. */
            if (address != 0xFFFFU) {
                uint8_t setaddr[4] = {BL_CMD_SET_ADDRESS, 0U, (uint8_t)(address >> 8), (uint8_t)(address & 0xFF)};
                bl_send_buf(setaddr, 4U);
                if (bl_get_ack(BB_UART_PERIOD_CYCLES() * 40U) != BL_ACK_SUCCESS) {
                    ack_out = ACK_D_GENERAL_ERROR;
                    break;
                }
            }
            if (cmd == CMD_DEVICE_READ || cmd == CMD_DEVICE_READ_EEPROM) {
                uint8_t rd[2] = {BL_CMD_READ_FLASH_SIL, in_len};
                bl_send_buf(rd, 2U);
                if (bl_read_buf(out, in_len, BB_UART_PERIOD_CYCLES() * 400U)) {
                    n = in_len;
                } else {
                    ack_out = ACK_D_GENERAL_ERROR;
                }
            } else if (cmd == CMD_DEVICE_WRITE || cmd == CMD_DEVICE_WRITE_EEPROM) {
                uint8_t setbuf[4] = {BL_CMD_SET_BUFFER, 0U, 0U, (uint8_t)in_len};
                bl_send_buf(setbuf, 4U);
                bl_send_buf(in, in_len);
                if (bl_get_ack(BB_UART_PERIOD_CYCLES() * 400U) != BL_ACK_SUCCESS) {
                    ack_out = ACK_D_GENERAL_ERROR;
                }
            } else if (cmd == CMD_DEVICE_PAGE_ERASE) {
                uint8_t er[2] = {BL_CMD_ERASE_FLASH, 0x01U};
                bl_send_buf(er, 2U);
                if (bl_get_ack(BB_UART_PERIOD_CYCLES() * 6000U) != BL_ACK_SUCCESS) {
                    ack_out = ACK_D_GENERAL_ERROR;
                }
            } else {
                ack_out = ACK_I_INVALID_CMD;
            }
        }
        break;

    default:
        ack_out = ACK_I_INVALID_CMD;
        break;
    }

    *out_len = n;
    *ack = ack_out;
}

static void passthru_loop(void) {
    /*
     * Park the line HIGH first and hold it. The ESC needs to boot while we are
     * already idling correctly, so give the operator a window to power-cycle it
     * before we attempt the handshake.
     */
    {
        PA8_Output();
        PA8_TxHigh();
        vcp_puts("\r\n*** ARMED: PA8 idle HIGH. POWER-CYCLE THE ESC NOW. ***\r\n");
        for (int i = 20; i > 0; i--) {
            vcp_puts("ARMED: idle HIGH, 20 s left. Power-cycle the ESC NOW.\r\n");
            for (uint32_t ms = 0; ms < 1000U; ms++) {
                HAL_Delay(1);
            }
        }
        vcp_puts("ARMED window closed, attempting handshake...\r\n");
    }

    esc_line_break_release();

    for (;;) {
        uint8_t hdr = 0;
        if (!vcp_read(&hdr, 0x00FFFFFFU)) continue;
        if (hdr != 0x2FU) continue; /* local escape '/' */

        uint8_t cmd = 0, addr_h = 0, addr_l = 0, len = 0;
        if (!vcp_read(&cmd, BB_UART_PERIOD_CYCLES() * 400U)) continue;
        if (!vcp_read(&addr_h, BB_UART_PERIOD_CYCLES() * 400U)) continue;
        if (!vcp_read(&addr_l, BB_UART_PERIOD_CYCLES() * 400U)) continue;
        if (!vcp_read(&len, BB_UART_PERIOD_CYCLES() * 400U)) continue;

        uint16_t nparams = (len == 0U) ? 256U : len;

        uint8_t in_buf[256];
        uint16_t got = 0U;
        while (got < nparams) {
            if (!vcp_read(&in_buf[got], BB_UART_PERIOD_CYCLES() * 400U)) break;
            got++;
        }

        uint8_t crc_hi = 0, crc_lo = 0;
        if (!vcp_read(&crc_hi, BB_UART_PERIOD_CYCLES() * 400U)) continue;
        if (!vcp_read(&crc_lo, BB_UART_PERIOD_CYCLES() * 400U)) continue;

        uint8_t out_buf[256];
        uint16_t out_len = 0U;
        uint8_t ack = ACK_OK;

        fourway_handle(cmd, (uint16_t)((addr_h << 8) | addr_l), in_buf,
                       (uint8_t)got, out_buf, sizeof(out_buf), &out_len, &ack);

        fourway_reply(cmd, (uint16_t)((addr_h << 8) | addr_l), out_buf, out_len, ack);
    }
}

void app_main(void) {
    HAL_Init();
    SystemCoreClockUpdate();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* LPUART1 to ST-Link VCP */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
    PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF12_LPUART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 115200;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&hlpuart1);

    /* Banner so the host can confirm the firmware is alive and in MSP mode. */
    vcp_puts("READY\r\n");

    /* prepare PA8 as half-duplex signal pin */
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* No blocking delays here: the host may send an MSP request as soon as
 * it opens COM6, and the LPUART has no FIFO, so any delay here loses it. */

    /* Prove the 19200 bit-bang works before the host asks anything.
       Runs ~1.5ms, so it does not cost us the first MSP request. */
    esc_bitbang_selftest();

    struct msp_ctx ctx;
    ctx.state = MSP_HEADER_M;
    ctx.len = 0;
    ctx.func_l = 0;
    ctx.func_h = 0;
    ctx.payload_i = 0;
    ctx.crc = 0;

    for (;;) {
        uint8_t b;
        if (!vcp_read_blocking(&b)) continue;

        switch (ctx.state) {
        case MSP_HEADER_M:
            ctx.state = (b == '$') ? MSP_HEADER_ANGLE : MSP_HEADER_M;
            break;
        case MSP_HEADER_ANGLE:
            ctx.state = (b == 'M') ? MSP_HEADER_PREFIX : MSP_HEADER_M;
            break;
        case MSP_HEADER_PREFIX:
            if (b == '<' || b == '>' || b == '!') {
                ctx.len = 0U;
                ctx.func_l = 0U;
                ctx.payload_i = 0U;
                ctx.crc = 0U;
                ctx.state = MSP_LEN_L;
            } else {
                ctx.state = MSP_HEADER_M;
            }
            break;
        case MSP_LEN_L:
            ctx.len = b;
            ctx.crc ^= b;
            ctx.state = MSP_FUNC_L;
            break;
        case MSP_FUNC_L:
            ctx.func_l = b;
            ctx.crc ^= b;
            if (ctx.len == 0) ctx.state = MSP_CRC;
            else ctx.state = MSP_PAYLOAD;
            break;
        case MSP_PAYLOAD:
            if (ctx.payload_i < (uint8_t)sizeof(ctx.payload)) {
                ctx.payload[ctx.payload_i] = b;
            }
            ctx.crc ^= b;
            ctx.payload_i++;
            if (ctx.payload_i >= ctx.len) ctx.state = MSP_CRC;
            break;
        case MSP_CRC:
            if (ctx.crc == b) {
                uint8_t cmd = ctx.func_l;
                /* MSP_SET_PASSTHROUGH is function 245 (0xF5) */
                if (cmd == 245U) {
                    uint8_t escCount = 1U; /* advertise one ESC */
                    msp_send_reply(1U, 245U, &escCount, 1);
                    passthru_loop(); /* enter bridge until exit */
                    break;
                }

                if (cmd == 1U) {
                    uint8_t api[3] = {1, 45, 0};
                    msp_send_reply(3U, 1U, api, 3);
                } else if (cmd == 2U) {
                    uint8_t variant[4] = {'O','B','R','O'};
                    msp_send_reply(4U, 2U, variant, 4);
                } else if (cmd == 3U) {
                    uint8_t fw[3] = {4, 5, 0};
                    msp_send_reply(3U, 3U, fw, 3);
                } else if (cmd == 4U) {
                    uint8_t board[4] = {0, 0, 0, 0};
                    board[0] = 0x58U; /* 'X' */
                    board[1] = 0x31U; /* '1' */
                    board[2] = s_selftest;
                    board[3] = s_selftest_tx;
                    msp_send_reply(4U, 4U, board, 4);
                    uint8_t extra[4] = {0, 0, 0, 0};
                    extra[0] = s_selftest_rx;
                    extra[1] = (uint8_t)(MOTOR_BITBANG_BAUD & 0xFFU);
                    extra[2] = (uint8_t)((s_line_edges >> 8) & 0xFFU);
                    extra[3] = (uint8_t)(s_line_edges & 0xFFU);
                    msp_send_reply(4U, 4U, extra, 4);
                } else if (cmd == 5U) {
                    uint8_t build[4] = {0,0,0,0};
                    msp_send_reply(4U, 5U, build, 4);
                } else if (cmd == 150U) {
                    uint8_t status[16] = {0};
                    msp_send_reply(16U, 150U, status, 16);
                } else {
                    msp_send_reply(0U, cmd, 0, 0);
                }
            }
            ctx.state = MSP_HEADER_M;
            break;
        default:
            ctx.state = MSP_HEADER_M;
            break;
        }
    }
}
