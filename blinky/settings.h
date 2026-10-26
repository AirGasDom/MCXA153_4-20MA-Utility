/*
 * Persistent settings in the last 8 KB flash sector of the MCXA153.
 *
 * Records are appended to the sector, one per save; the sector is only
 * erased when it fills up (170 records), so each save costs one phrase
 * program instead of a full sector erase.
 */
#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t mode;
    uint32_t color;
    uint32_t period_ms;
    uint32_t pwm_high;
    uint32_t cal_lo_counts;
    uint32_t cal_lo_ua;
    uint32_t cal_hi_counts;
    uint32_t cal_hi_ua;
} settings_t;

/* Load the newest valid record. Returns false if none is stored. */
bool settings_load(settings_t *out);

/* Append a record, erasing the sector first if it is full. Returns 0 on
 * success, otherwise the ROM flash API status code. */
int32_t settings_save(const settings_t *s);

/* Erase the settings sector. Returns 0 on success. */
int32_t settings_erase(void);

/* Number of records currently stored in the sector (0..SETTINGS_SLOTS). */
uint32_t settings_used_slots(void);

#define SETTINGS_ADDR   0x0001E000U /* keep in sync with MCXA153_flash.ld */
#define SETTINGS_SIZE   0x2000U
#define SETTINGS_RECORD 48U /* multiple of the 16-byte flash phrase */
#define SETTINGS_SLOTS  (SETTINGS_SIZE / SETTINGS_RECORD)

#endif /* SETTINGS_H_ */
