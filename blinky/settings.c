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

/* On-flash record: three 16-byte phrases. */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    settings_t s;
    uint32_t reserved;
    uint32_t crc;
} record_t;

_Static_assert(sizeof(record_t) == SETTINGS_RECORD, "record size must match SETTINGS_RECORD");

#define RECORD_MAGIC   0x5358434DU /* "MCXS" */
#define RECORD_VERSION 2U /* v2 adds calibration */
#define ERASED_WORD    0xFFFFFFFFU

static const volatile record_t *slot(uint32_t i)
{
    return (const volatile record_t *)(SETTINGS_ADDR + i * SETTINGS_RECORD);
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

static bool record_valid(const record_t *r)
{
    return r->magic == RECORD_MAGIC && r->version == RECORD_VERSION &&
           r->crc == crc32(r, offsetof(record_t, crc));
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

uint32_t settings_used_slots(void)
{
    uint32_t i = 0;
    while (i < SETTINGS_SLOTS && slot(i)->magic != ERASED_WORD)
    {
        i++;
    }
    return i;
}

bool settings_load(settings_t *out)
{
    for (uint32_t i = settings_used_slots(); i > 0U; i--)
    {
        record_t r;
        memcpy(&r, (const void *)slot(i - 1U), sizeof(r));
        if (record_valid(&r))
        {
            *out = r.s;
            return true;
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

    uint32_t i = settings_used_slots();
    if (i >= SETTINGS_SLOTS)
    {
        st = do_erase(&cfg);
        if (st != 0)
        {
            return st;
        }
        i = 0;
    }

    record_t r = {
        .magic    = RECORD_MAGIC,
        .version  = RECORD_VERSION,
        .s        = *s,
        .reserved = ERASED_WORD,
    };
    r.crc = crc32(&r, offsetof(record_t, crc));

    st = do_program(&cfg, SETTINGS_ADDR + i * SETTINGS_RECORD, &r);
    if (st != 0)
    {
        return st;
    }

    /* Read back through the bus to confirm. */
    record_t check;
    memcpy(&check, (const void *)slot(i), sizeof(check));
    return memcmp(&check, &r, sizeof(r)) == 0 ? 0 : -1;
}

int32_t settings_erase(void)
{
    flash_config_t cfg;
    status_t st = flash_open(&cfg);
    return st != 0 ? st : do_erase(&cfg);
}
