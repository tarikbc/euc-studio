/*
 * Settings storage (see euc/settings/storage.h).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "euc/quirks.h"
#include "euc/settings/settings.h"
#include "euc/settings/storage.h"

#define ERASED_HALFWORD 0xFFFFU

#define STANDSTILL_CM_PER_S         49
#define SAVE_AFTER_STANDSTILL_TICKS 496U /* 1 s */

static const uint16_t *slot_record(const uint16_t *page, uint32_t slot)
{
    return &page[(size_t)slot * EUC_SETTINGS_RECORD_HALFWORDS];
}

static bool record_all(const uint16_t *record, uint16_t value)
{
    for (uint32_t i = 0U; i < EUC_SETTINGS_RECORD_HALFWORDS; i++) {
        if (record[i] != value) {
            return false;
        }
    }
    return true;
}

static bool records_equal(const uint16_t *a, const uint16_t *b)
{
    return memcmp(a, b, EUC_SETTINGS_RECORD_HALFWORDS * sizeof(a[0])) == 0;
}

static void copy_record(uint16_t *to, const uint16_t *from)
{
    (void)memcpy(to, from, EUC_SETTINGS_RECORD_HALFWORDS * sizeof(to[0]));
}

static bool crc_matches(const uint16_t *record)
{
    return euc_settings_record_stored_crc(record) == euc_settings_record_crc(record);
}

/* Clean: a record with a valid CRC, or one written by the stock firmware (CRC bits all 0). */
static bool journal_record_is_valid(const uint16_t *record)
{
    return crc_matches(record) || (euc_settings_record_stored_crc(record) == 0U);
}

static void clear_write(struct euc_storage_write *write)
{
    (void)memset(write, 0, sizeof(*write));
}

/* The record the journal holds: stock takes the slot before the first erased one, clean the newest valid. */
static void journal_newest(const uint16_t *page, uint32_t first_erased, int32_t last_used, uint16_t *record)
{
    euc_settings_default_record(record);
    if (EUC_QUIRK(NO_RECORD_CRC)) {
        if (first_erased > 0U) {
            copy_record(record, slot_record(page, first_erased - 1U));
        }
        return;
    }
    for (int32_t slot = last_used; slot >= 0; slot--) {
        const uint16_t *candidate = slot_record(page, (uint32_t)slot);
        if ((!record_all(candidate, ERASED_HALFWORD)) && journal_record_is_valid(candidate)) {
            copy_record(record, candidate);
            return;
        }
    }
}

void euc_storage_load(struct euc_storage_state *state, const uint16_t page[EUC_STORAGE_PAGE_HALFWORDS],
                      const uint16_t backup[EUC_SETTINGS_RECORD_HALFWORDS], struct euc_settings *settings,
                      struct euc_storage_write *write)
{
    (void)memset(state, 0, sizeof(*state));
    clear_write(write);

    uint32_t first_erased = EUC_STORAGE_SLOTS;
    int32_t last_used = -1;
    for (uint32_t slot = 0U; slot < EUC_STORAGE_SLOTS; slot++) {
        if (record_all(slot_record(page, slot), ERASED_HALFWORD)) {
            if (first_erased == EUC_STORAGE_SLOTS) {
                first_erased = slot;
            }
        } else {
            last_used = (int32_t)slot;
        }
    }

    uint16_t journal[EUC_SETTINGS_RECORD_HALFWORDS];
    journal_newest(page, first_erased, last_used, journal);

    if (EUC_QUIRK(SAVE_SLOT_REUSE)) {
        state->next_slot = (uint8_t)first_erased;
        if (first_erased == EUC_STORAGE_SLOTS) {
            /* Full page: erase it and keep the last record in slot 0. */
            write->erase_page = true;
            write->program_slot = true;
            write->slot = 0U;
            copy_record(write->record, journal);
            state->next_slot = 1U;
        }
    } else {
        const int32_t next_slot = last_used + 1; /* 0..128 */
        state->next_slot = (uint8_t)next_slot;
    }

    const bool backup_wins = EUC_QUIRK(NO_RECORD_CRC) ? (!record_all(backup, 0U)) : crc_matches(backup);
    const uint16_t *record = backup_wins ? backup : journal;
    euc_settings_unpack(record, settings);

    copy_record(state->journal_record, journal);
    if (backup_wins) {
        copy_record(state->backup_record, backup);
    }
}

/* Picks the slot of the next save; false when that needs a page erase and `allow_erase` is false. */
static bool place_record(struct euc_storage_state *state, bool allow_erase, struct euc_storage_write *write)
{
    if (EUC_QUIRK(SAVE_SLOT_REUSE)) {
        /* Always the slot found at boot. Once it holds a record, the port rewrites the whole page. */
        if (state->fixed_slot_used && (!allow_erase)) {
            return false;
        }
        write->rewrite_page = state->fixed_slot_used;
        write->slot = state->next_slot;
        write->program_slot = true;
        state->fixed_slot_used = true;
        return true;
    }
    if (state->next_slot >= EUC_STORAGE_SLOTS) {
        if (!allow_erase) {
            return false;
        }
        write->erase_page = true;
        state->next_slot = 0U;
    }
    write->slot = state->next_slot;
    write->program_slot = true;
    state->next_slot++;
    return true;
}

void euc_storage_save(struct euc_storage_state *state, const struct euc_settings *settings,
                      struct euc_storage_write *write)
{
    clear_write(write);
    euc_settings_pack(settings, write->record);
    write->write_backup = true;
    (void)place_record(state, true, write);
    copy_record(state->journal_record, write->record);
    copy_record(state->backup_record, write->record);
}

void euc_storage_tick(struct euc_storage_state *state, const struct euc_settings *settings,
                      const struct euc_storage_tick_in *in, struct euc_storage_write *write)
{
    clear_write(write);
    if (EUC_QUIRK(SAVE_ONLY_POWEROFF)) {
        return;
    }

    euc_settings_pack(settings, write->record);
    if (!records_equal(write->record, state->backup_record)) {
        write->write_backup = true;
        copy_record(state->backup_record, write->record);
    }

    const bool standstill =
        (in->speed_cm_per_s >= -STANDSTILL_CM_PER_S) && (in->speed_cm_per_s <= STANDSTILL_CM_PER_S);
    if (!standstill) {
        state->standstill_ticks = 0U;
    } else if (state->standstill_ticks < UINT16_MAX) {
        state->standstill_ticks++;
    } else {
        /* saturated */
    }

    if ((state->standstill_ticks >= SAVE_AFTER_STANDSTILL_TICKS) &&
        (!records_equal(write->record, state->journal_record)) &&
        place_record(state, in->drive_disabled, write)) {
        copy_record(state->journal_record, write->record);
    }
}
