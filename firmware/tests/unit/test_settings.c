/*
 * Unit tests for the settings record, the odometer and the flash journal (core/settings/). The journal page
 * and the backup registers are plain arrays; apply_write() does what the port does with a storage write.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "euc/settings/settings.h"
#include "euc/settings/storage.h"
#include "euc/sil.h"
#include "unity.h"

#define RECORD EUC_SETTINGS_RECORD_HALFWORDS

static uint16_t page[EUC_STORAGE_PAGE_HALFWORDS];
static uint16_t backup[RECORD];

static void use_profile(uint32_t profile)
{
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profile));
}

void setUp(void)
{
    use_profile(EUC_SIL_PROFILE_COMPAT);
    for (size_t i = 0U; i < EUC_STORAGE_PAGE_HALFWORDS; i++) {
        page[i] = 0xFFFFU;
    }
    (void)memset(backup, 0, sizeof(backup));
}

void tearDown(void)
{
}

static void apply_write(const struct euc_storage_write *write)
{
    if (write->write_backup) {
        (void)memcpy(backup, write->record, sizeof(backup));
    }
    if (write->erase_page) {
        for (size_t i = 0U; i < EUC_STORAGE_PAGE_HALFWORDS; i++) {
            page[i] = 0xFFFFU;
        }
    }
    if (write->program_slot) {
        /* A rewrite keeps the other slots, so the result is the same as programming the slot alone. */
        (void)memcpy(&page[(size_t)write->slot * RECORD], write->record, RECORD * sizeof(page[0]));
    }
}

static void store_slot(uint32_t slot, const uint16_t record[RECORD])
{
    (void)memcpy(&page[(size_t)slot * RECORD], record, RECORD * sizeof(page[0]));
}

static struct euc_settings sample_settings(void)
{
    struct euc_settings settings;
    (void)memset(&settings, 0, sizeof(settings));
    settings.odometer_low = 0x5678U;
    settings.odometer_high = 0x0016U;
    settings.pedal_mode = 2U;
    settings.speed_alarm_mode = 3U;
    settings.tiltback_speed_kmh = 69U;
    settings.pedal_trim = 5U;
    settings.roll_limit_step = 7U;
    settings.wc_value = 4U;
    settings.wm_value = 1U;
    settings.roll_cutoff_mode = 2U;
    settings.turn_compensation = 5U;
    settings.pwm_tiltback = 1U;
    settings.drive_mode = 1U;
    settings.lock_state = 1U;
    settings.imu_fault = 0U;
    settings.units_miles = 1U;
    settings.pwm_alarm_percent = 80U;
    settings.beeper_volume = 9U;
    return settings;
}

/* Record ----------------------------------------------------------------------------------------------- */

static void test_default_record(void)
{
    uint16_t record[RECORD];
    struct euc_settings settings;
    euc_settings_default_record(record);
    euc_settings_unpack(record, &settings);
    TEST_ASSERT_EQUAL_UINT8(1U, settings.pedal_mode);
    TEST_ASSERT_EQUAL_UINT16(20U, settings.tiltback_speed_kmh);
    TEST_ASSERT_EQUAL_UINT8(3U, settings.turn_compensation);
    TEST_ASSERT_EQUAL_UINT8(6U, settings.beeper_volume);
    TEST_ASSERT_EQUAL_UINT8(80U, settings.pwm_alarm_percent);
    TEST_ASSERT_EQUAL_UINT32(0U, euc_settings_odometer_units(&settings));
}

static void test_pack_layout_compat(void)
{
    const struct euc_settings settings = sample_settings();
    uint16_t record[RECORD];
    euc_settings_pack(&settings, record);
    const uint16_t expected[RECORD] = {0x5678U, 0x0016U, 0x2345U, 0x0005U,
                                       0x7412U, 0x0511U, 0x0101U, 0x5009U};
    TEST_ASSERT_EQUAL_HEX16_ARRAY(expected, record, RECORD);
}

static void test_pack_unpack_round_trip(void)
{
    const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        const struct euc_settings settings = sample_settings();
        uint16_t record[RECORD];
        struct euc_settings back;
        (void)memset(&back, 0, sizeof(back)); /* padding too, for the memory compare */
        euc_settings_pack(&settings, record);
        euc_settings_unpack(record, &back);
        TEST_ASSERT_EQUAL_MEMORY(&settings, &back, sizeof(settings));
        if (profiles[p] == EUC_SIL_PROFILE_COMPAT) {
            TEST_ASSERT_EQUAL_HEX16(0U, euc_settings_record_stored_crc(record));
        } else {
            TEST_ASSERT_EQUAL_HEX16(euc_settings_record_crc(record), euc_settings_record_stored_crc(record));
            TEST_ASSERT_TRUE(euc_settings_record_stored_crc(record) != 0U);
        }
    }
}

static void test_unpack_applies_the_stock_load_rules(void)
{
    const uint16_t record[RECORD] = {0U, 0U, 0x0F00U, 0x000CU, 0x0030U, 0x0A00U, 0x0300U, 0x000CU};
    struct euc_settings settings;
    euc_settings_unpack(record, &settings);
    TEST_ASSERT_EQUAL_UINT8(15U, settings.speed_alarm_mode); /* not clamped */
    TEST_ASSERT_EQUAL_UINT16(20U, settings.tiltback_speed_kmh);
    TEST_ASSERT_EQUAL_UINT8(9U, settings.pedal_trim);
    TEST_ASSERT_EQUAL_UINT8(0U, settings.wm_value);
    TEST_ASSERT_EQUAL_UINT8(0U, settings.turn_compensation);
    TEST_ASSERT_EQUAL_UINT8(EUC_SETTINGS_LOCKED, settings.lock_state);
    TEST_ASSERT_EQUAL_UINT8(80U, settings.pwm_alarm_percent);
    TEST_ASSERT_EQUAL_UINT8(9U, settings.beeper_volume);
}

static void test_odometer_carry_per_profile(void)
{
    const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        const bool compat = profiles[p] == EUC_SIL_PROFILE_COMPAT;
        struct euc_settings settings = sample_settings();
        uint16_t trip = 7U;
        settings.odometer_low = 0xFFFEU;
        settings.odometer_high = 1U;
        euc_settings_add_odometer_unit(&settings, &trip);
        TEST_ASSERT_EQUAL_UINT16(8U, trip);
        /* 0x1FFFF: stock already carried, so it reads 0x2FFFF. */
        TEST_ASSERT_EQUAL_UINT32(compat ? 0x2FFFFU : 0x1FFFFU, euc_settings_odometer_units(&settings));
        euc_settings_add_odometer_unit(&settings, &trip);
        TEST_ASSERT_EQUAL_UINT32(0x20000U, euc_settings_odometer_units(&settings));
    }
}

/* Journal, stock ---------------------------------------------------------------------------------------- */

static void test_empty_journal_loads_defaults(void)
{
    struct euc_storage_state state;
    struct euc_storage_write write;
    struct euc_settings settings;
    euc_storage_load(&state, page, backup, &settings, &write);
    TEST_ASSERT_EQUAL_UINT8(1U, settings.pedal_mode);
    TEST_ASSERT_EQUAL_UINT8(0U, state.next_slot);
    TEST_ASSERT_FALSE(write.program_slot || write.erase_page || write.write_backup);
}

static void test_compat_load_takes_the_last_slot_then_backup(void)
{
    struct euc_storage_state state;
    struct euc_storage_write write;
    struct euc_settings settings = sample_settings();
    uint16_t record[RECORD];
    for (uint32_t slot = 0U; slot < 3U; slot++) {
        settings.odometer_low = (uint16_t)(100U + slot);
        euc_settings_pack(&settings, record);
        store_slot(slot, record);
    }
    euc_storage_load(&state, page, backup, &settings, &write);
    TEST_ASSERT_EQUAL_UINT16(102U, settings.odometer_low);
    TEST_ASSERT_EQUAL_UINT8(3U, state.next_slot);

    backup[0] = 555U; /* any non-zero register: the backup copy wins */
    euc_storage_load(&state, page, backup, &settings, &write);
    TEST_ASSERT_EQUAL_UINT16(555U, settings.odometer_low);
}

static void test_compat_full_page_is_rewritten_at_boot(void)
{
    struct euc_storage_state state;
    struct euc_storage_write write;
    struct euc_settings settings = sample_settings();
    uint16_t record[RECORD];
    for (uint32_t slot = 0U; slot < EUC_STORAGE_SLOTS; slot++) {
        settings.odometer_low = (uint16_t)slot;
        euc_settings_pack(&settings, record);
        store_slot(slot, record);
    }
    euc_storage_load(&state, page, backup, &settings, &write);
    TEST_ASSERT_TRUE(write.erase_page);
    TEST_ASSERT_TRUE(write.program_slot);
    TEST_ASSERT_EQUAL_UINT8(0U, write.slot);
    TEST_ASSERT_EQUAL_UINT16(127U, write.record[0]);
    TEST_ASSERT_EQUAL_UINT8(1U, state.next_slot);
}

static void test_compat_saves_reuse_the_boot_slot(void)
{
    struct euc_storage_state state;
    struct euc_storage_write write;
    struct euc_settings settings;
    euc_storage_load(&state, page, backup, &settings, &write);

    const struct euc_storage_tick_in still = {0, true};
    for (unsigned tick = 0U; tick < 2000U; tick++) {
        euc_storage_tick(&state, &settings, &still, &write);
        TEST_ASSERT_FALSE(write.program_slot || write.write_backup); /* SAVE_ONLY_POWEROFF */
    }

    euc_storage_save(&state, &settings, &write);
    TEST_ASSERT_TRUE(write.write_backup);
    TEST_ASSERT_EQUAL_UINT8(0U, write.slot);
    TEST_ASSERT_FALSE(write.rewrite_page);
    apply_write(&write);
    settings.odometer_low = 9U;
    euc_storage_save(&state, &settings, &write);
    TEST_ASSERT_EQUAL_UINT8(0U, write.slot); /* same slot: the page is rewritten */
    TEST_ASSERT_TRUE(write.rewrite_page);
}

/* Journal, clean ---------------------------------------------------------------------------------------- */

static void test_clean_load_skips_torn_and_bad_slots(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_storage_state state;
    struct euc_storage_write write;
    struct euc_settings settings = sample_settings();
    uint16_t record[RECORD];

    settings.odometer_low = 1000U;
    euc_settings_pack(&settings, record);
    store_slot(0U, record);
    settings.odometer_low = 1001U;
    euc_settings_pack(&settings, record);
    record[1] ^= 0x0100U; /* corrupted: CRC no longer matches */
    store_slot(1U, record);
    settings.odometer_low = 1002U;
    euc_settings_pack(&settings, record);
    for (size_t i = 4U; i < RECORD; i++) {
        record[i] = 0xFFFFU; /* torn: the write stopped after four halfwords */
    }
    store_slot(2U, record);

    euc_storage_load(&state, page, backup, &settings, &write);
    TEST_ASSERT_EQUAL_UINT16(1000U, settings.odometer_low);
    TEST_ASSERT_EQUAL_UINT8(3U, state.next_slot); /* after the last used slot, never into a torn one */
    TEST_ASSERT_FALSE(write.erase_page);
}

static void test_clean_reads_stock_records(void)
{
    use_profile(EUC_SIL_PROFILE_COMPAT);
    struct euc_settings settings = sample_settings();
    uint16_t record[RECORD];
    euc_settings_pack(&settings, record);
    store_slot(0U, record);
    backup[0] = 0x1234U; /* stock backup copy: no CRC, so clean ignores it */

    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_storage_state state;
    struct euc_storage_write write;
    struct euc_settings loaded;
    (void)memset(&loaded, 0, sizeof(loaded)); /* padding too, for the memory compare */
    euc_storage_load(&state, page, backup, &loaded, &write);
    TEST_ASSERT_EQUAL_MEMORY(&settings, &loaded, sizeof(settings));
}

static void test_clean_backup_wins_only_with_a_valid_crc(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_settings settings = sample_settings();
    uint16_t record[RECORD];
    euc_settings_pack(&settings, record);
    store_slot(0U, record);
    settings.odometer_low = 4242U;
    euc_settings_pack(&settings, backup);

    struct euc_storage_state state;
    struct euc_storage_write write;
    struct euc_settings loaded;
    euc_storage_load(&state, page, backup, &loaded, &write);
    TEST_ASSERT_EQUAL_UINT16(4242U, loaded.odometer_low);

    (void)memset(backup, 0, sizeof(backup)); /* backup domain reset */
    euc_storage_load(&state, page, backup, &loaded, &write);
    TEST_ASSERT_EQUAL_UINT16(0x5678U, loaded.odometer_low);
}

static void test_clean_tick_mirrors_and_saves_at_standstill(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_storage_state state;
    struct euc_storage_write write;
    struct euc_settings settings;
    euc_storage_load(&state, page, backup, &settings, &write);

    const struct euc_storage_tick_in moving = {300, false};
    const struct euc_storage_tick_in still = {0, false};
    settings.odometer_low = 10U;
    euc_storage_tick(&state, &settings, &moving, &write);
    TEST_ASSERT_TRUE(write.write_backup); /* the backup copy follows at once */
    TEST_ASSERT_FALSE(write.program_slot);
    apply_write(&write);
    euc_storage_tick(&state, &settings, &moving, &write);
    TEST_ASSERT_FALSE(write.write_backup);

    unsigned saves = 0U;
    for (unsigned tick = 0U; tick < 1000U; tick++) {
        euc_storage_tick(&state, &settings, &still, &write);
        if (write.program_slot) {
            saves++;
            TEST_ASSERT_EQUAL_UINT8(0U, write.slot);
            TEST_ASSERT_EQUAL_UINT32(495U, tick); /* after 1 s still */
            apply_write(&write);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(1U, saves);
    settings.odometer_low = 11U;
    for (unsigned tick = 0U; tick < 10U; tick++) {
        euc_storage_tick(&state, &settings, &still, &write);
        apply_write(&write);
    }
    TEST_ASSERT_EQUAL_UINT8(2U, state.next_slot); /* the next slot, not the same one */
}

static void test_clean_full_page_waits_for_a_safe_erase(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_settings settings = sample_settings();
    uint16_t record[RECORD];
    for (uint32_t slot = 0U; slot < EUC_STORAGE_SLOTS; slot++) {
        settings.odometer_low = (uint16_t)slot;
        euc_settings_pack(&settings, record);
        store_slot(slot, record);
    }
    struct euc_storage_state state;
    struct euc_storage_write write;
    euc_storage_load(&state, page, backup, &settings, &write);
    TEST_ASSERT_FALSE(write.erase_page); /* no erase at boot */
    TEST_ASSERT_EQUAL_UINT16(127U, settings.odometer_low);

    settings.odometer_low = 128U;
    const struct euc_storage_tick_in enabled = {0, false};
    for (unsigned tick = 0U; tick < 1000U; tick++) {
        euc_storage_tick(&state, &settings, &enabled, &write);
        TEST_ASSERT_FALSE(write.program_slot); /* erasing would stall the drive */
    }
    const struct euc_storage_tick_in disabled = {0, true};
    euc_storage_tick(&state, &settings, &disabled, &write);
    TEST_ASSERT_TRUE(write.erase_page);
    TEST_ASSERT_TRUE(write.program_slot);
    TEST_ASSERT_EQUAL_UINT8(0U, write.slot);
    TEST_ASSERT_EQUAL_UINT8(1U, state.next_slot);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_default_record);
    RUN_TEST(test_pack_layout_compat);
    RUN_TEST(test_pack_unpack_round_trip);
    RUN_TEST(test_unpack_applies_the_stock_load_rules);
    RUN_TEST(test_odometer_carry_per_profile);
    RUN_TEST(test_empty_journal_loads_defaults);
    RUN_TEST(test_compat_load_takes_the_last_slot_then_backup);
    RUN_TEST(test_compat_full_page_is_rewritten_at_boot);
    RUN_TEST(test_compat_saves_reuse_the_boot_slot);
    RUN_TEST(test_clean_load_skips_torn_and_bad_slots);
    RUN_TEST(test_clean_reads_stock_records);
    RUN_TEST(test_clean_backup_wins_only_with_a_valid_crc);
    RUN_TEST(test_clean_tick_mirrors_and_saves_at_standstill);
    RUN_TEST(test_clean_full_page_waits_for_a_safe_erase);
    return UNITY_END();
}
