/*
 * TinyUSB configuration: one CDC-ACM port on USB0 (full speed, device only),
 * used as the second command shell. See main.c.
 */
#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#define CFG_TUSB_MCU      OPT_MCU_MCXA15
#define CFG_TUSB_OS       OPT_OS_NONE
#define CFG_TUSB_DEBUG    0

#define CFG_TUD_ENABLED   1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))

#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC    1
#define CFG_TUD_MSC    0
#define CFG_TUD_HID    0
#define CFG_TUD_MIDI   0
#define CFG_TUD_VENDOR 0

#define CFG_TUD_CDC_NOTIFY     1
#define CFG_TUD_CDC_RX_BUFSIZE 64
#define CFG_TUD_CDC_TX_BUFSIZE 256 /* "help" is ~1.3 kB; fewer stalls in usb_putc */
#define CFG_TUD_CDC_RX_EPSIZE  64
#define CFG_TUD_CDC_TX_EPSIZE  64

#endif
