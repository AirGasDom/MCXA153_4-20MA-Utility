/*
 * USB descriptors for the CDC-ACM shell port (one CDC interface pair).
 *
 * VID 0xCafe is TinyUSB's test VID, fine for lab use; get a real VID/PID
 * before shipping hardware.
 */
#include <string.h>
#include "tusb.h"

#define USB_VID 0xCafe
#define USB_PID 0x4001 /* TinyUSB example convention: 0x4000 | CDC */

enum
{
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

enum
{
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC,
};

static const tusb_desc_device_t k_desc_device = {
    .bLength         = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB          = 0x0200,

    /* IAD: required for a composite-style CDC function */
    .bDeviceClass    = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor  = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,

    .iManufacturer = STRID_MANUFACTURER,
    .iProduct      = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,

    .bNumConfigurations = 1,
};

static const uint8_t k_desc_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

static const char *const k_strings[] = {
    [STRID_MANUFACTURER] = "BAM1",
    [STRID_PRODUCT]      = "MCXA153 4-20 mA shell",
    [STRID_SERIAL]       = "0001",
    [STRID_CDC]          = "MCXA153 shell",
};

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&k_desc_device;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return k_desc_config;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    static uint16_t desc[32 + 1];
    size_t n;

    (void)langid;
    if (index == STRID_LANGID)
    {
        desc[1] = 0x0409; /* English (US) */
        n       = 1;
    }
    else
    {
        if (index >= sizeof(k_strings) / sizeof(k_strings[0]) || k_strings[index] == NULL)
        {
            return NULL;
        }
        n = strlen(k_strings[index]);
        if (n > 32U)
        {
            n = 32U;
        }
        for (size_t i = 0; i < n; i++)
        {
            desc[1 + i] = (uint8_t)k_strings[index][i];
        }
    }
    desc[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2U * n + 2U));
    return desc;
}
