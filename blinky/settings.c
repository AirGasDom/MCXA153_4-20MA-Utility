#include <string.h>
#include "fsl_device_registers.h"
#include "settings.h"

/*
 * Minimal declarations for the MCXA153 boot ROM flash API. Layout copied
 * from NXP's fsl_romapi.h (MCXA153 DFP 26.09.00); only the order of the
 * function pointers matters.
 */
typedef int32_t status_t;

typedef struct
{
    uint32_t ffrBlockBase;
    uint32_t ffrTotalSize;
    uint32_t ffrPageSize;
    uint32_t sectorSize;
    uint32_t cfpaPageVersion;
    uint32_t cfpaPageOffset;
} flash_ffr_config_t;

typedef struct
{
    uint32_t PFlashBlockBase;
    uint32_t PFlashTotalSize;
    uint32_t PFlashBlockCount;
    uint32_t PFlashPageSize;
    uint32_t PFlashSectorSize;
    flash_ffr_config_t ffrConfig;
} flash_config_t;

typedef struct
{
    status_t (*flash_init)(flash_config_t *config);
    status_t (*flash_erase_sector)(flash_config_t *config, uint32_t start, uint32_t len, uint32_t key);
    status_t (*flash_program_phrase)(flash_config_t *config, uint32_t start, uint8_t *src, uint32_t len);
    status_t (*flash_program_page)(flash_config_t *config, uint32_t start, uint8_t *src, uint32_t len);
    status_t (*flash_verify_program)(flash_config_t *config, uint32_t start, uint32_t len,
                                     const uint8_t *expected, uint32_t *failedAddress, uint32_t *failedData);
    status_t (*flash_verify_erase_phrase)(flash_config_t *config, uint32_t start, uint32_t len);
    status_t (*flash_verify_erase_page)(flash_config_t *config, uint32_t start, uint32_t len);
    status_t (*flash_verify_erase_sector)(flash_config_t *config, uint32_t start, uint32_t len);
} flash_driver_interface_t;

typedef struct
{
    void (*run_bootloader)(void *arg);
    const flash_driver_interface_t *flash_driver;
    void (*jump)(void *arg);
} bootloader_tree_t;

#define ROM_API       ((const bootloader_tree_t *)0x03003FE0U)
#define FLASH_API     (ROM_API->flash_driver)
#define FLASH_ERASE_KEY 0x6B65666CU /* 'lfek' */

/* On-flash record: four 16-byte phrases. */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    settings_t s;
    uint32_t crc;
} record_t;

_Static_assert(sizeof(record_t) == SETTINGS_RECORD, "record size must match SETTINGS_RECORD");

/* Single-channel record written by older firmware (three phrases). Only
 * read, so a saved calibration carries over to channel 1; the next save
 * erases the sector and starts over with v3 records. */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t mode;
    uint32_t color;
    uint32_t period_ms;
    settings_channel_t ch0;
    uint32_t reserved;
    uint32_t crc;
} record_v2_t;

_Static_assert(sizeof(record_v2_t) == 48U, "v2 record is 48 bytes");

#define RECORD_MAGIC      0x5358434DU /* "MCXS" */
#define RECORD_VERSION    3U /* v3 adds the second output channel */
#define RECORD_VERSION_V2 2U /* v2 adds calibration */
#define ERASED_WORD       0xFFFFFFFFU

static const volatile uint32_t *slot(uint32_t stride, uint32_t i)
{
    return (const volatile uint32_t *)(SETTINGS_ADDR + i * stride);
}

/* Record size used by what is in the sector: 48 for v2, else 64. */
static uint32_t sector_stride(void)
{
    const volatile uint32_t *w = slot(SETTINGS_RECORD, 0);
    return (w[0] == RECORD_MAGIC && w[1] == RECORD_VERSION_V2) ? sizeof(record_v2_t) : SETTINGS_RECORD;
}

static uint32_t crc32(const void *data, uint32_t len)
{
    const uint8_t *p = data;
    uint32_t crc     = 0xFFFFFFFFU;
    while (len--)
    {
        crc ^= *p++;
        for (int b = 0; b < 8; b++)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

/* Both layouts start with magic, version and end with a CRC of the rest. */
static bool record_valid(const void *r, uint32_t size, uint32_t version)
{
    const uint32_t *w = r;
    return w[0] == RECORD_MAGIC && w[1] == version && w[size / 4U - 1U] == crc32(r, size - 4U);
}

/* After erasing or programming, drop stale flash data from the LPCAC and
 * the flash speculation buffers (as NXP's flashiap example does). */
static void flash_caches_clear(void)
{
    if ((SYSCON->NVM_CTRL & SYSCON_NVM_CTRL_DIS_FLASH_SPEC_MASK) == 0U)
    {
        SYSCON->NVM_CTRL |= SYSCON_NVM_CTRL_DIS_FLASH_SPEC_MASK;
        SYSCON->NVM_CTRL &= ~SYSCON_NVM_CTRL_DIS_FLASH_SPEC_MASK;
    }
    if ((SYSCON->NVM_CTRL & SYSCON_NVM_CTRL_DIS_DATA_SPEC_MASK) == 0U)
    {
        SYSCON->NVM_CTRL |= SYSCON_NVM_CTRL_DIS_DATA_SPEC_MASK;
        SYSCON->NVM_CTRL &= ~SYSCON_NVM_CTRL_DIS_DATA_SPEC_MASK;
    }
    if ((SYSCON->LPCAC_CTRL & SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK) == 0U)
    {
        SYSCON->LPCAC_CTRL |= SYSCON_LPCAC_CTRL_CLR_LPCAC_MASK;
    }
}

/* Interrupts are masked while the ROM works on flash so no handler runs
 * (from flash) in the middle of an erase or program. */
static status_t do_erase(flash_config_t *cfg)
{
    __disable_irq();
    status_t st = FLASH_API->flash_erase_sector(cfg, SETTINGS_ADDR, SETTINGS_SIZE, FLASH_ERASE_KEY);
    if (st == 0)
    {
        st = FLASH_API->flash_verify_erase_sector(cfg, SETTINGS_ADDR, SETTINGS_SIZE);
    }
    flash_caches_clear();
    __enable_irq();
    return st;
}

static status_t do_program(flash_config_t *cfg, uint32_t addr, record_t *r)
{
    __disable_irq();
    status_t st = FLASH_API->flash_program_phrase(cfg, addr, (uint8_t *)r, sizeof(*r));
    flash_caches_clear();
    __enable_irq();
    return st;
}

static status_t flash_open(flash_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    return FLASH_API->flash_init(cfg);
}

static uint32_t used_slots(uint32_t stride)
{
    uint32_t i = 0;
    while (i < SETTINGS_SIZE / stride && slot(stride, i)[0] != ERASED_WORD)
    {
        i++;
    }
    return i;
}

uint32_t settings_used_slots(void)
{
    return used_slots(sector_stride());
}

bool settings_load(settings_t *out)
{
    uint32_t stride = sector_stride();

    for (uint32_t i = used_slots(stride); i > 0U; i--)
    {
        const void *src = (const void *)slot(stride, i - 1U);
        if (stride == SETTINGS_RECORD)
        {
            record_t r;
            memcpy(&r, src, sizeof(r));
            if (record_valid(&r, sizeof(r), RECORD_VERSION))
            {
                *out = r.s;
                return true;
            }
        }
        else
        {
            record_v2_t r;
            memcpy(&r, src, sizeof(r));
            if (record_valid(&r, sizeof(r), RECORD_VERSION_V2))
            {
                out->mode      = r.mode;
                out->color     = r.color;
                out->period_ms = r.period_ms;
                out->ch[0]     = r.ch0;
                return true;
            }
        }
    }
    return false;
}

int32_t settings_save(const settings_t *s)
{
    flash_config_t cfg;
    status_t st = flash_open(&cfg);
    if (st != 0)
    {
        return st;
    }

    /* Start a fresh sector when it is full or holds another record layout
     * (records of different sizes can't share it). */
    const volatile uint32_t *first = slot(SETTINGS_RECORD, 0);
    uint32_t i = used_slots(SETTINGS_RECORD);
    if (i >= SETTINGS_SLOTS || (first[0] != ERASED_WORD && first[1] != RECORD_VERSION))
    {
        st = do_erase(&cfg);
        if (st != 0)
        {
            return st;
        }
        i = 0;
    }

    record_t r = {
        .magic   = RECORD_MAGIC,
        .version = RECORD_VERSION,
        .s       = *s,
    };
    r.crc = crc32(&r, offsetof(record_t, crc));

    st = do_program(&cfg, SETTINGS_ADDR + i * SETTINGS_RECORD, &r);
    if (st != 0)
    {
        return st;
    }

    /* Read back through the bus to confirm. */
    record_t check;
    memcpy(&check, (const void *)slot(SETTINGS_RECORD, i), sizeof(check));
    return memcmp(&check, &r, sizeof(r)) == 0 ? 0 : -1;
}

int32_t settings_erase(void)
{
    flash_config_t cfg;
    status_t st = flash_open(&cfg);
    return st != 0 ? st : do_erase(&cfg);
}
