/**
  ******************************************************************************
  * @file    esc_passthrough.c
  * @brief   BLHeli ESC programming passthrough.
  *
  * The host (esc-configurator.com / BLHeliSuite) talks MSP to us as if we were
  * a Betaflight flight controller. MSP_SET_PASSTHROUGH switches the link to the
  * BLHeli 4-way interface protocol, whose device commands are forwarded to the
  * ESC bootloader as 19200 baud half-duplex serial on the ESC signal wire (PA8).
  * Protocol handling follows Betaflight's serial_4way.c / serial_4way_avrootloader.c.
  ******************************************************************************
  */
#include "esc_passthrough.h"
#include <string.h>

/* ---- ESC signal pin ------------------------------------------------------ */
#define ESC_GPIO                GPIOA
#define ESC_PIN_NUM             8U
#define ESC_PIN                 (1U << ESC_PIN_NUM)

#define ESC_COUNT               1U

/* ---- One-wire bootloader serial ------------------------------------------ */
#define BL_BAUD                 19200U
#define START_BIT_TIMEOUT_MS    2U

/* Bootloader commands */
#define CMD_RUN                 0x00
#define CMD_PROG_FLASH          0x01
#define CMD_ERASE_FLASH         0x02
#define CMD_READ_FLASH_SIL      0x03
#define CMD_VERIFY_FLASH_ARM    0x04
#define CMD_READ_EEPROM         0x04
#define CMD_PROG_EEPROM         0x05
#define CMD_READ_FLASH_ATM      0x07
#define CMD_KEEP_ALIVE          0xFD
#define CMD_SET_BUFFER          0xFE
#define CMD_SET_ADDRESS         0xFF

/* Bootloader responses */
#define BR_SUCCESS              0x30
#define BR_ERRORVERIFY          0xC0
#define BR_ERRORCOMMAND         0xC1
#define BR_ERRORCRC             0xC2
#define BR_NONE                 0xFF

/* ---- BLHeli 4-way interface ---------------------------------------------- */
#define FW_LOCAL_ESCAPE         0x2F
#define FW_REMOTE_ESCAPE        0x2E

#define FW_PROTOCOL_VER         108
#define FW_INTERFACE_NAME       "m4wFCIntf"
#define FW_VERSION_HI           200   /* 20.0.03 -> 20003 / 100 */
#define FW_VERSION_LO           3     /* 20003 % 100 */

#define cmd_InterfaceTestAlive  0x30
#define cmd_ProtocolGetVersion  0x31
#define cmd_InterfaceGetName    0x32
#define cmd_InterfaceGetVersion 0x33
#define cmd_InterfaceExit       0x34
#define cmd_DeviceReset         0x35
#define cmd_DeviceInitFlash     0x37
#define cmd_DeviceEraseAll      0x38
#define cmd_DevicePageErase     0x39
#define cmd_DeviceRead          0x3A
#define cmd_DeviceWrite         0x3B
#define cmd_DeviceC2CK_LOW      0x3C
#define cmd_DeviceReadEEprom    0x3D
#define cmd_DeviceWriteEEprom   0x3E
#define cmd_InterfaceSetMode    0x3F
#define cmd_DeviceVerify        0x40

#define ACK_OK                  0x00
#define ACK_I_INVALID_CMD       0x02
#define ACK_I_INVALID_CRC       0x03
#define ACK_I_VERIFY_ERROR      0x04
#define ACK_D_COMMAND_FAILED    0x06
#define ACK_I_INVALID_CHANNEL   0x08
#define ACK_I_INVALID_PARAM     0x09
#define ACK_D_GENERAL_ERROR     0x0F

/* Interface modes */
#define imC2                    0
#define imSIL_BLB               1
#define imATM_BLB               2
#define imSK                    3
#define imARM_BLB               4

/* ---- MSP ----------------------------------------------------------------- */
#define MSP_API_VERSION         1
#define MSP_FC_VARIANT          2
#define MSP_FC_VERSION          3
#define MSP_BOARD_INFO          4
#define MSP_BUILD_INFO          5
#define MSP_FEATURE_CONFIG      36
#define MSP_STATUS              101
#define MSP_MOTOR               104
#define MSP_MOTOR_3D_CONFIG     124
#define MSP_BATTERY_STATE       130
#define MSP_MOTOR_CONFIG        131
#define MSP_UID                 160
#define MSP_SET_MOTOR           214
#define MSP_SET_PASSTHROUGH     245

#define MSP_PASSTHROUGH_ESC_4WAY 0xFF
#define ARMING_DISABLED_RX_FAILSAFE (1U << 2)

#define HOST_BYTE_TIMEOUT_MS    1000U

static USART_TypeDef *host;
static uint32_t bit_cycles;
static uint32_t half_bit_cycles;
static uint32_t ms_cycles;

/* [0] = signature low, [1] = signature high, [2] = boot version, [3] = mode */
static uint8_t dev_info[4];
static uint8_t intf_mode = imSIL_BLB;
static uint8_t param_buf[256];

/* ---- Timing (DWT cycle counter) ------------------------------------------ */
static inline uint32_t cycles(void)
{
  return DWT->CYCCNT;
}

static inline void wait_until(uint32_t t)
{
  while ((int32_t)(cycles() - t) < 0) {}
}

/* ---- ESC pin ------------------------------------------------------------- */
static inline void esc_input(void)
{
  ESC_GPIO->MODER &= ~(3U << (ESC_PIN_NUM * 2U));
}

static inline void esc_output(void)
{
  ESC_GPIO->BSRR = ESC_PIN;  /* idle high before driving */
  ESC_GPIO->MODER = (ESC_GPIO->MODER & ~(3U << (ESC_PIN_NUM * 2U)))
                  | (1U << (ESC_PIN_NUM * 2U));
}

static inline int esc_is_hi(void)
{
  return (ESC_GPIO->IDR & ESC_PIN) != 0U;
}

/* ---- Software UART on the ESC pin ---------------------------------------- */
static int suart_getc(uint8_t *bt)
{
  uint32_t deadline = cycles() + START_BIT_TIMEOUT_MS * ms_cycles;
  while (esc_is_hi())
  {
    if ((int32_t)(cycles() - deadline) >= 0) return 0;
  }

  /* Sample start, 8 data and stop bits in the middle of each bit */
  uint32_t t = cycles() + half_bit_cycles;
  uint16_t bits = 0;
  for (uint32_t i = 0; i < 10U; i++)
  {
    wait_until(t);
    if (esc_is_hi()) bits |= (uint16_t)(1U << i);
    t += bit_cycles;
  }
  if ((bits & 1U) || !(bits & (1U << 9)))
  {
    return 0;
  }
  *bt = (uint8_t)(bits >> 1);
  return 1;
}

static void suart_putc(uint8_t b)
{
  /* idle bit, start bit, 8 data bits LSB first, stop bit (not waited out so
   * the line is released as soon as possible after the last byte) */
  uint16_t frame = (uint16_t)((b << 2) | 1U | (1U << 10));
  uint32_t t = cycles();
  for (;;)
  {
    if (frame & 1U) ESC_GPIO->BSRR = ESC_PIN;
    else            ESC_GPIO->BRR = ESC_PIN;
    frame >>= 1;
    t += bit_cycles;
    if (frame == 0U) break;
    wait_until(t);
  }
}

/* ---- CRCs ---------------------------------------------------------------- */
/* Bootloader CRC (AVR _crc16_update, poly 0xA001) */
static uint16_t crc16_bl(uint16_t crc, uint8_t b)
{
  crc ^= b;
  for (uint32_t i = 0; i < 8U; i++)
  {
    crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
  }
  return crc;
}

/* 4-way interface CRC (XMODEM, poly 0x1021) */
static uint16_t crc16_xmodem(uint16_t crc, uint8_t b)
{
  crc ^= (uint16_t)b << 8;
  for (uint32_t i = 0; i < 8U; i++)
  {
    crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
  }
  return crc;
}

/* ---- ESC bootloader ------------------------------------------------------ */
static inline int mcu_connected(void)
{
  return dev_info[0] > 0U;
}

static void bl_send(const uint8_t *buf, uint16_t len)
{
  uint16_t crc = 0;
  esc_output();
  for (uint16_t i = 0; i < len; i++)
  {
    suart_putc(buf[i]);
    crc = crc16_bl(crc, buf[i]);
  }
  if (mcu_connected())
  {
    suart_putc((uint8_t)(crc & 0xFFU));
    suart_putc((uint8_t)(crc >> 8));
  }
  esc_input();
}

/* len 0 means 256 */
static int bl_read(uint8_t *buf, uint8_t len)
{
  uint16_t n = len ? len : 256U;
  uint16_t crc = 0;
  uint8_t crc_lo, crc_hi, ack = BR_NONE;

  for (uint16_t i = 0; i < n; i++)
  {
    if (!suart_getc(&buf[i])) return 0;
    crc = crc16_bl(crc, buf[i]);
  }
  if (mcu_connected())
  {
    if (!suart_getc(&crc_lo) || !suart_getc(&crc_hi) || !suart_getc(&ack)) return 0;
    if (crc != (uint16_t)(crc_lo | (crc_hi << 8))) return 0;
  }
  else if (!suart_getc(&ack))
  {
    return 0;
  }
  return ack == BR_SUCCESS;
}

/* tries: number of START_BIT_TIMEOUT_MS windows to wait */
static uint8_t bl_get_ack(uint32_t tries)
{
  uint8_t ack = BR_NONE;
  while (!suart_getc(&ack) && tries)
  {
    tries--;
  }
  return ack;
}

static int bl_connect(void)
{
  static const uint8_t boot_init[] = {0, 0, 0, 0, 0, 0, 0, 0, 0x0D,
                                      'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
  uint8_t info[8];

  bl_send(boot_init, sizeof(boot_init));
  /* "471c", signature high, signature low, boot version, boot pages (+ ACK) */
  if (!bl_read(info, sizeof(info))) return 0;
  if (memcmp(info, "471", 3) != 0) return 0;

  dev_info[2] = info[3];
  dev_info[1] = info[4];
  dev_info[0] = info[5];
  return 1;
}

static int bl_set_address(uint8_t hi, uint8_t lo)
{
  if (hi == 0xFFU && lo == 0xFFU) return 1;
  uint8_t cmd[] = {CMD_SET_ADDRESS, 0, hi, lo};
  bl_send(cmd, sizeof(cmd));
  return bl_get_ack(2) == BR_SUCCESS;
}

static int bl_set_buffer(const uint8_t *data, uint8_t len)
{
  uint8_t cmd[] = {CMD_SET_BUFFER, 0, (uint8_t)(len == 0U ? 1U : 0U), len};
  bl_send(cmd, sizeof(cmd));
  if (bl_get_ack(2) != BR_NONE) return 0;
  bl_send(data, len ? len : 256U);
  return bl_get_ack(40) == BR_SUCCESS;
}

static int bl_read_cmd(uint8_t cmd, uint8_t hi, uint8_t lo, uint8_t *buf, uint8_t len)
{
  if (!bl_set_address(hi, lo)) return 0;
  uint8_t c[] = {cmd, len};
  bl_send(c, sizeof(c));
  return bl_read(buf, len);
}

static int bl_write_cmd(uint8_t cmd, uint8_t hi, uint8_t lo,
                        const uint8_t *buf, uint8_t len, uint32_t tries)
{
  if (!bl_set_address(hi, lo) || !bl_set_buffer(buf, len)) return 0;
  uint8_t c[] = {cmd, 0x01};
  bl_send(c, sizeof(c));
  return bl_get_ack(tries) == BR_SUCCESS;
}

static int bl_page_erase(uint8_t hi, uint8_t lo)
{
  if (!bl_set_address(hi, lo)) return 0;
  uint8_t c[] = {CMD_ERASE_FLASH, 0x01};
  bl_send(c, sizeof(c));
  return bl_get_ack(3000U / START_BIT_TIMEOUT_MS) == BR_SUCCESS;
}

static uint8_t bl_verify(uint8_t hi, uint8_t lo, const uint8_t *buf, uint8_t len)
{
  if (!bl_set_address(hi, lo) || !bl_set_buffer(buf, len)) return BR_NONE;
  uint8_t c[] = {CMD_VERIFY_FLASH_ARM, 0x01};
  bl_send(c, sizeof(c));
  return bl_get_ack(40U / START_BIT_TIMEOUT_MS);
}

static int bl_keep_alive(void)
{
  uint8_t c[] = {CMD_KEEP_ALIVE, 0};
  bl_send(c, sizeof(c));
  return bl_get_ack(1) == BR_ERRORCOMMAND;
}

static void bl_restart(void)
{
  uint8_t c[] = {CMD_RUN, 0};
  dev_info[0] = 1;  /* force CRC to be sent */
  bl_send(c, sizeof(c));
}

static int device_connect(void)
{
  for (uint32_t attempt = 0; attempt < 3U; attempt++)
  {
    memset(dev_info, 0, sizeof(dev_info));
    if (!bl_connect()) continue;

    uint16_t sig = (uint16_t)((dev_info[1] << 8) | dev_info[0]);
    switch (sig)
    {
      case 0xF310: case 0xF330: case 0xF410: case 0xF390:
      case 0xF850: case 0xE8B1: case 0xE8B2:
        intf_mode = imSIL_BLB;
        return 1;
      case 0x9307: case 0x930A: case 0x930F: case 0x940B:
        intf_mode = imATM_BLB;
        return 1;
      case 0x1F06: case 0x3306: case 0x3406: case 0x2B06:
      case 0x4706: case 0x6206: case 0x6906:
        intf_mode = imARM_BLB;
        return 1;
      default:
        break;
    }
  }
  memset(dev_info, 0, sizeof(dev_info));
  return 0;
}

/* ---- Host UART (polled, register level) ---------------------------------- */
/* timeout_ms 0 waits forever */
static int host_getc(uint8_t *b, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while (!(host->ISR & USART_ISR_RXNE_RXFNE))
  {
    if (host->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
    {
      host->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    }
    if (timeout_ms && (HAL_GetTick() - start) >= timeout_ms) return 0;
  }
  *b = (uint8_t)host->RDR;
  return 1;
}

static void host_putc(uint8_t b)
{
  while (!(host->ISR & USART_ISR_TXE_TXFNF)) {}
  host->TDR = b;
}

static void host_flush(void)
{
  while (!(host->ISR & USART_ISR_TC)) {}
}

/* ---- 4-way interface ----------------------------------------------------- */
static void fourway_run(void)
{
  memset(dev_info, 0, sizeof(dev_info));

  for (;;)
  {
    uint8_t b, cmd, addr_hi, addr_lo, in_len, crc_hi, crc_lo;
    uint16_t crc;

    do
    {
      (void)host_getc(&b, 0);
    } while (b != FW_LOCAL_ESCAPE);
    crc = crc16_xmodem(0, b);

    if (!host_getc(&cmd, HOST_BYTE_TIMEOUT_MS)) continue;
    crc = crc16_xmodem(crc, cmd);
    if (!host_getc(&addr_hi, HOST_BYTE_TIMEOUT_MS)) continue;
    crc = crc16_xmodem(crc, addr_hi);
    if (!host_getc(&addr_lo, HOST_BYTE_TIMEOUT_MS)) continue;
    crc = crc16_xmodem(crc, addr_lo);
    if (!host_getc(&in_len, HOST_BYTE_TIMEOUT_MS)) continue;
    crc = crc16_xmodem(crc, in_len);

    uint16_t n = in_len ? in_len : 256U;  /* 0 means 256 */
    uint16_t i;
    for (i = 0; i < n; i++)
    {
      if (!host_getc(&param_buf[i], HOST_BYTE_TIMEOUT_MS)) break;
      crc = crc16_xmodem(crc, param_buf[i]);
    }
    if (i < n) continue;
    if (!host_getc(&crc_hi, HOST_BYTE_TIMEOUT_MS)) continue;
    if (!host_getc(&crc_lo, HOST_BYTE_TIMEOUT_MS)) continue;

    uint8_t ack = (crc == (uint16_t)((crc_hi << 8) | crc_lo)) ? ACK_OK : ACK_I_INVALID_CRC;
    uint8_t dummy[2] = {0, 0};
    const uint8_t *out = dummy;
    uint16_t out_len = 1;
    int exit_requested = 0;

    if (ack == ACK_OK)
    {
      switch (cmd)
      {
        case cmd_InterfaceTestAlive:
          if (mcu_connected() && !bl_keep_alive())
          {
            ack = ACK_D_GENERAL_ERROR;
            memset(dev_info, 0, sizeof(dev_info));
          }
          break;

        case cmd_ProtocolGetVersion:
          dummy[0] = FW_PROTOCOL_VER;
          break;

        case cmd_InterfaceGetName:
          out = (const uint8_t *)FW_INTERFACE_NAME;
          out_len = sizeof(FW_INTERFACE_NAME) - 1U;
          break;

        case cmd_InterfaceGetVersion:
          dummy[0] = FW_VERSION_HI;
          dummy[1] = FW_VERSION_LO;
          out_len = 2;
          break;

        case cmd_InterfaceExit:
          exit_requested = 1;
          break;

        case cmd_InterfaceSetMode:
          if (param_buf[0] >= imSIL_BLB && param_buf[0] <= imARM_BLB && param_buf[0] != imSK)
          {
            intf_mode = param_buf[0];
          }
          else
          {
            ack = ACK_I_INVALID_PARAM;
          }
          break;

        case cmd_DeviceReset:
          if (param_buf[0] < ESC_COUNT)
          {
            bl_restart();
            memset(dev_info, 0, sizeof(dev_info));
          }
          else
          {
            ack = ACK_I_INVALID_CHANNEL;
          }
          break;

        case cmd_DeviceInitFlash:
          if (param_buf[0] >= ESC_COUNT)
          {
            ack = ACK_I_INVALID_CHANNEL;
            break;
          }
          if (device_connect())
          {
            dev_info[3] = intf_mode;
          }
          else
          {
            ack = ACK_D_GENERAL_ERROR;
          }
          out = dev_info;
          out_len = sizeof(dev_info);
          break;

        case cmd_DeviceRead:
        {
          uint8_t len = param_buf[0];
          uint8_t rd = (intf_mode == imATM_BLB) ? CMD_READ_FLASH_ATM : CMD_READ_FLASH_SIL;
          if (bl_read_cmd(rd, addr_hi, addr_lo, param_buf, len))
          {
            out = param_buf;
            out_len = len ? len : 256U;
          }
          else
          {
            ack = ACK_D_COMMAND_FAILED;
          }
          break;
        }

        case cmd_DeviceReadEEprom:
        {
          uint8_t len = param_buf[0];
          if (intf_mode != imATM_BLB)
          {
            ack = ACK_I_INVALID_CMD;
          }
          else if (bl_read_cmd(CMD_READ_EEPROM, addr_hi, addr_lo, param_buf, len))
          {
            out = param_buf;
            out_len = len ? len : 256U;
          }
          else
          {
            ack = ACK_D_COMMAND_FAILED;
          }
          break;
        }

        case cmd_DeviceWrite:
          if (!bl_write_cmd(CMD_PROG_FLASH, addr_hi, addr_lo, param_buf, in_len,
                            500U / START_BIT_TIMEOUT_MS))
          {
            ack = ACK_D_COMMAND_FAILED;
          }
          break;

        case cmd_DeviceWriteEEprom:
          if (intf_mode != imATM_BLB)
          {
            ack = ACK_I_INVALID_CMD;
          }
          else if (!bl_write_cmd(CMD_PROG_EEPROM, addr_hi, addr_lo, param_buf, in_len,
                                 3000U / START_BIT_TIMEOUT_MS))
          {
            ack = ACK_D_COMMAND_FAILED;
          }
          break;

        case cmd_DevicePageErase:
          if (intf_mode == imSIL_BLB || intf_mode == imARM_BLB)
          {
            /* SiLabs pages are 512 bytes, ARM pages 1024 bytes */
            uint8_t page_hi = (intf_mode == imARM_BLB) ? (uint8_t)(param_buf[0] << 2)
                                                       : (uint8_t)(param_buf[0] << 1);
            if (!bl_page_erase(page_hi, 0))
            {
              ack = ACK_D_COMMAND_FAILED;
            }
          }
          else
          {
            ack = ACK_I_INVALID_CMD;
          }
          break;

        case cmd_DeviceVerify:
          if (intf_mode == imARM_BLB)
          {
            uint8_t r = bl_verify(addr_hi, addr_lo, param_buf, in_len);
            if (r == BR_ERRORVERIFY)      ack = ACK_I_VERIFY_ERROR;
            else if (r != BR_SUCCESS)     ack = ACK_D_GENERAL_ERROR;
          }
          else
          {
            ack = ACK_I_INVALID_CMD;
          }
          break;

        default:
          ack = ACK_I_INVALID_CMD;
          break;
      }
    }

    /* Response: escape, cmd, addr, len, params, ack, crc */
    crc = 0;
    const uint8_t hdr[] = {FW_REMOTE_ESCAPE, cmd, addr_hi, addr_lo, (uint8_t)out_len};
    for (i = 0; i < sizeof(hdr); i++)
    {
      host_putc(hdr[i]);
      crc = crc16_xmodem(crc, hdr[i]);
    }
    for (i = 0; i < out_len; i++)
    {
      host_putc(out[i]);
      crc = crc16_xmodem(crc, out[i]);
    }
    host_putc(ack);
    crc = crc16_xmodem(crc, ack);
    host_putc((uint8_t)(crc >> 8));
    host_putc((uint8_t)(crc & 0xFFU));
    host_flush();

    if (exit_requested)
    {
      return;
    }
  }
}

/* ---- MSP v1 -------------------------------------------------------------- */
static void msp_reply(uint8_t cmd, const uint8_t *data, uint8_t len, int error)
{
  uint8_t ck = len ^ cmd;
  host_putc('$');
  host_putc('M');
  host_putc(error ? '!' : '>');
  host_putc(len);
  host_putc(cmd);
  for (uint8_t i = 0; i < len; i++)
  {
    host_putc(data[i]);
    ck ^= data[i];
  }
  host_putc(ck);
  host_flush();
}

static void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
  put_u16(p, (uint16_t)(v & 0xFFFFU));
  put_u16(p + 2, (uint16_t)(v >> 16));
}

/* Returns 1 when the host requested 4-way passthrough */
static int msp_handle(uint8_t cmd, const uint8_t *in, uint8_t in_len)
{
  uint8_t out[32] = {0};
  uint8_t len = 0;

  switch (cmd)
  {
    case MSP_API_VERSION:
      out[0] = 0;   /* MSP protocol version */
      out[1] = 1;   /* API major */
      out[2] = 44;  /* API minor */
      len = 3;
      break;

    case MSP_FC_VARIANT:
      memcpy(out, "BTFL", 4);
      len = 4;
      break;

    case MSP_FC_VERSION:
      out[0] = 4;
      out[1] = 3;
      out[2] = 0;
      len = 3;
      break;

    case MSP_BUILD_INFO:
      memcpy(out, __DATE__, 11);
      memcpy(out + 11, __TIME__, 8);
      memcpy(out + 19, "0000000", 7);
      len = 26;
      break;

    case MSP_BOARD_INFO:
      memcpy(out, "S474", 4);  /* board identifier */
      put_u16(out + 4, 0);     /* hardware revision */
      len = 6;
      break;

    case MSP_STATUS:
      /* Report RX failsafe so the configurator knows no radio is active.
       * Offsets: 15 extra mode flag byte count (0), 16 arming flag count,
       * 17 arming disable flags. */
      out[13] = 1;                                   /* PID profile count */
      out[16] = 25;                                  /* arming flag count */
      put_u32(out + 17, ARMING_DISABLED_RX_FAILSAFE);
      len = 22;
      break;

    case MSP_FEATURE_CONFIG:
      put_u32(out, 0);
      len = 4;
      break;

    case MSP_MOTOR:
      /* 8 motors; nonzero entries tell the host how many ESCs exist */
      put_u16(out, 1000);
      len = 16;
      break;

    case MSP_MOTOR_3D_CONFIG:
      put_u16(out, 1406);
      put_u16(out + 2, 1514);
      put_u16(out + 4, 1460);
      len = 6;
      break;

    case MSP_BATTERY_STATE:
      len = 11;
      break;

    case MSP_MOTOR_CONFIG:
      put_u16(out, 1070);      /* min throttle */
      put_u16(out + 2, 2000);  /* max throttle */
      put_u16(out + 4, 1000);  /* min command */
      out[6] = ESC_COUNT;      /* motor count */
      out[7] = 14;             /* motor poles */
      len = 10;
      break;

    case MSP_UID:
      put_u32(out, HAL_GetUIDw0());
      put_u32(out + 4, HAL_GetUIDw1());
      put_u32(out + 8, HAL_GetUIDw2());
      len = 12;
      break;

    case MSP_SET_MOTOR:
      /* Motor output is disabled in programming mode */
      break;

    case MSP_SET_PASSTHROUGH:
      if (in_len == 0U || in[0] == MSP_PASSTHROUGH_ESC_4WAY)
      {
        out[0] = ESC_COUNT;
        msp_reply(cmd, out, 1, 0);
        return 1;
      }
      out[0] = 0;
      len = 1;
      break;

    default:
      msp_reply(cmd, NULL, 0, 1);
      return 0;
  }

  msp_reply(cmd, out, len, 0);
  return 0;
}

static void msp_run(void)
{
  uint8_t payload[255];

  for (;;)
  {
    uint8_t b, len, cmd, ck;

    (void)host_getc(&b, 0);
    if (b != '$') continue;
    if (!host_getc(&b, HOST_BYTE_TIMEOUT_MS) || b != 'M') continue;
    if (!host_getc(&b, HOST_BYTE_TIMEOUT_MS) || b != '<') continue;
    if (!host_getc(&len, HOST_BYTE_TIMEOUT_MS)) continue;
    if (!host_getc(&cmd, HOST_BYTE_TIMEOUT_MS)) continue;

    uint8_t sum = len ^ cmd;
    uint16_t i;
    for (i = 0; i < len; i++)
    {
      if (!host_getc(&payload[i], HOST_BYTE_TIMEOUT_MS)) break;
      sum ^= payload[i];
    }
    if (i < len) continue;
    if (!host_getc(&ck, HOST_BYTE_TIMEOUT_MS) || ck != sum) continue;

    if (msp_handle(cmd, payload, len))
    {
      return;
    }
  }
}

/* ---- Entry point --------------------------------------------------------- */
void ESC_Passthrough_Run(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef gpio = {0};

  host = huart->Instance;

  /* Cycle counter for bit timing */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  bit_cycles = SystemCoreClock / BL_BAUD;
  half_bit_cycles = bit_cycles / 2U;
  ms_cycles = SystemCoreClock / 1000U;

  /* Release PA8 from TIM1 and idle it high so the ESC enters its bootloader */
  gpio.Pin = ESC_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ESC_GPIO, &gpio);
  ESC_GPIO->BSRR = ESC_PIN;

  for (;;)
  {
    msp_run();
    fourway_run();
  }
}
