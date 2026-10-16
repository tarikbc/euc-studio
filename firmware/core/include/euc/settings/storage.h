/*
 * Settings storage: a journal of 128 records of 16 bytes in the flash page at 0x0800F000, mirrored in the
 * backup registers DR3..DR10. The core decides what to write; the port erases and programs flash and writes
 * the backup registers (euc_storage_write).
 *
 * Boot (euc_storage_load):
 *   stock  The record is the last slot before the first erased one (none: the default record). A full page is
 *          erased and that record goes back to slot 0. Then the backup registers win if any of them is not 0.
 *   clean  The record is the newest slot with a valid CRC (or a stock record, whose CRC bits are all 0), and
 *          the backup registers win only with a valid CRC. Torn slots are skipped and never reused. Nothing
 *          is erased at boot.
 *
 * Saves (euc_storage_save: power-off paths and the drive-mode toggle; euc_storage_tick: clean only):
 *   stock  Every save goes to the slot found at boot, so a second save in one power cycle reprograms a used
 *          slot, which the port does as read-erase-rewrite of the whole page (quirk SAVE_SLOT_REUSE).
 *          Only the power-off paths save (quirk SAVE_ONLY_POWEROFF).
 *   clean  Each save takes the next erased slot; a full page is erased only by a save that is allowed to
 *          erase (power-off, or the drive is disabled). The backup registers follow every change at once,
 *          and the journal saves a changed record after 1 s at standstill.
 */
#ifndef EUC_SETTINGS_STORAGE_H
#define EUC_SETTINGS_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

#include "euc/settings/settings.h"

#define EUC_STORAGE_SLOTS          128U
#define EUC_STORAGE_PAGE_HALFWORDS 1024U /* one 2 KiB flash page */

_Static_assert(EUC_STORAGE_PAGE_HALFWORDS == (EUC_STORAGE_SLOTS * EUC_SETTINGS_RECORD_HALFWORDS),
               "the journal fills exactly one page");

/* What the port writes, in this order: backup registers, page erase, one slot. */
struct euc_storage_write {
    bool write_backup; /* DR3..DR10 = record */
    bool erase_page;   /* erase the page; the slot below is then the only record */
    bool program_slot;
    bool rewrite_page; /* the slot is not erased: read the page, erase it, write it back with the new slot */
    uint8_t slot;
    uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS];
};

struct euc_storage_state {
    uint8_t next_slot;    /* EUC_STORAGE_SLOTS: the page is full */
    bool fixed_slot_used; /* SAVE_SLOT_REUSE: the boot slot already holds a record of this power cycle */
    uint16_t journal_record[EUC_SETTINGS_RECORD_HALFWORDS]; /* last record in the journal (clean) */
    uint16_t backup_record[EUC_SETTINGS_RECORD_HALFWORDS];  /* last record in the backup registers (clean) */
    uint16_t standstill_ticks;
};

struct euc_storage_tick_in {
    int16_t speed_cm_per_s;
    bool drive_disabled;
};

/* Boot. `page` is the journal page (memory-mapped flash on the target). `write` may ask for a page rewrite.
 */
void euc_storage_load(struct euc_storage_state *state, const uint16_t page[EUC_STORAGE_PAGE_HALFWORDS],
                      const uint16_t backup[EUC_SETTINGS_RECORD_HALFWORDS], struct euc_settings *settings,
                      struct euc_storage_write *write);

/* Save now (power-off paths). Always writes the backup registers and one slot. */
void euc_storage_save(struct euc_storage_state *state, const struct euc_settings *settings,
                      struct euc_storage_write *write);

/* Every RTC tick. Stock: nothing. Clean: mirror changes to the backup registers, journal them at standstill.
 */
void euc_storage_tick(struct euc_storage_state *state, const struct euc_settings *settings,
                      const struct euc_storage_tick_in *in, struct euc_storage_write *write);

#endif
