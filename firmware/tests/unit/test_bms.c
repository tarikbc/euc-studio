/*
 * Unit tests for the BMS link (core/bms/): reply decoding, health check, request cadence and short replies.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "euc/bms/bms.h"
#include "euc/sil.h"
#include "unity.h"

static uint8_t reply[EUC_BMS_REPLY_BYTES];

static void use_profile(uint32_t profile)
{
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profile));
}

static void put16(uint8_t *at, uint16_t value)
{
    at[0] = (uint8_t)(value >> 8U);
    at[1] = (uint8_t)(value & 0xFFU);
}

/* A healthy reply: block k has temperatures 20 + k, 21 + k, 22 + k, cells 4000 + 16k + i, status 0x7D01. */
static void healthy_reply(void)
{
    for (size_t block = 0U; block < EUC_BMS_BLOCKS; block++) {
        uint8_t *data = &reply[block * 44U];
        for (size_t i = 0U; i < 3U; i++) {
            put16(&data[2U * i], (uint16_t)(20U + block + i));
        }
        for (size_t i = 0U; i < 16U; i++) {
            put16(&data[6U + (2U * i)], (uint16_t)(4000U + (16U * block) + i));
        }
        put16(&data[38], 0x7D01U);
        put16(&data[40], 1000U);
        put16(&data[42], 651U);
    }
}

void setUp(void)
{
    use_profile(EUC_SIL_PROFILE_COMPAT);
    healthy_reply();
}

void tearDown(void)
{
}

static void test_request_bytes(void)
{
    const uint8_t expected[EUC_BMS_REQUEST_BYTES] = {0x1CU, 0x03U, 0x40U, 0x31U, 0xFEU};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, euc_bms_request, EUC_BMS_REQUEST_BYTES);
}

static void test_parse_fields(void)
{
    struct euc_bms_state state;
    euc_bms_init(&state);
    TEST_ASSERT_EQUAL_INT16(5000, state.temperature_min_c); /* stock power-on value, until the first reply */
    TEST_ASSERT_EQUAL_INT16(0, state.temperature_max_c);
    put16(&reply[44U + 40U], (uint16_t)(int16_t)-1000);
    euc_bms_parse(&state, reply);
    TEST_ASSERT_EQUAL_INT16(20, state.temperature_c[0]);
    TEST_ASSERT_EQUAL_INT16(25, state.temperature_c[11]);
    TEST_ASSERT_EQUAL_UINT16(4000U, state.cell_mv[0]);
    TEST_ASSERT_EQUAL_UINT16(4063U, state.cell_mv[63]);
    TEST_ASSERT_EQUAL_HEX16(0x7D01U, state.blocks[3].status_word);
    TEST_ASSERT_EQUAL_UINT16(651U, state.blocks[2].word42_raw);
    /* 1000 x 90 / 32.768 / 100 = 27.47 -> 27 (truncated); -1000 -> -27. */
    TEST_ASSERT_EQUAL_INT16(27, state.blocks[0].current_da);
    TEST_ASSERT_EQUAL_INT16(-27, state.blocks[1].current_da);
    TEST_ASSERT_EQUAL_INT16(20, state.temperature_min_c);
    TEST_ASSERT_EQUAL_INT16(25, state.temperature_max_c);
    TEST_ASSERT_FALSE(state.fault);
}

static void test_health_check_latches_after_three_failures(void)
{
    const uint16_t bad[3] = {0x4D01U, 0x7101U, 0x7D11U}; /* bits 13:12 = 0; bits 11:10 != 3; bit 4 set */
    for (size_t i = 0U; i < 3U; i++) {
        struct euc_bms_state state;
        euc_bms_init(&state);
        put16(&reply[2U * 44U + 38U], bad[i]);
        euc_bms_parse(&state, reply);
        euc_bms_parse(&state, reply);
        TEST_ASSERT_FALSE(state.fault);
        euc_bms_parse(&state, reply);
        TEST_ASSERT_TRUE(state.fault);
        healthy_reply();
        euc_bms_parse(&state, reply);
        TEST_ASSERT_TRUE(state.fault); /* latched */
    }
}

static void test_request_cadence(void)
{
    struct euc_bms_state state;
    struct euc_bms_out out;
    struct euc_bms_in in = {0U, reply};
    euc_bms_init(&state);
    for (unsigned tick = 0U; tick < 1500U; tick++) {
        euc_bms_tick(&state);
        euc_bms_poll(&state, &in, &out);
        TEST_ASSERT_FALSE(out.send_request || out.parsed);
    }
    euc_bms_tick(&state);
    euc_bms_poll(&state, &in, &out);
    TEST_ASSERT_TRUE(out.send_request);
    in.bytes_received = 100U;
    euc_bms_tick(&state);
    euc_bms_poll(&state, &in, &out);
    TEST_ASSERT_FALSE(out.parsed);
    in.bytes_received = EUC_BMS_REPLY_BYTES;
    euc_bms_poll(&state, &in, &out);
    TEST_ASSERT_TRUE(out.parsed);
    euc_bms_poll(&state, &in, &out);
    TEST_ASSERT_FALSE(out.parsed); /* once per request */
}

static void test_short_reply_per_profile(void)
{
    const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        struct euc_bms_state state;
        struct euc_bms_out out;
        struct euc_bms_in in = {0U, reply};
        euc_bms_init(&state);
        for (unsigned request = 0U; request < 3U; request++) {
            state.ticks_since_request = 1501U;
            euc_bms_poll(&state, &in, &out);
            TEST_ASSERT_TRUE(out.send_request);
            in.bytes_received = 30U; /* the reply stops short */
            state.ticks_since_request = 501U;
            euc_bms_poll(&state, &in, &out);
            TEST_ASSERT_EQUAL(profiles[p] == EUC_SIL_PROFILE_COMPAT, out.parsed);
            in.bytes_received = 0U;
        }
        /* Compat parses the old buffer (healthy); clean counts three failed exchanges. */
        TEST_ASSERT_EQUAL(profiles[p] == EUC_SIL_PROFILE_CLEAN, state.fault);
    }
}

static void test_max_temperature_below_zero(void)
{
    const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        struct euc_bms_state state;
        euc_bms_init(&state);
        put16(&reply[0], (uint16_t)(int16_t)-5);
        euc_bms_parse(&state, reply);
        TEST_ASSERT_EQUAL_INT16(-5, state.temperature_min_c);
        TEST_ASSERT_EQUAL_INT16((profiles[p] == EUC_SIL_PROFILE_COMPAT) ? -5 : 25, state.temperature_max_c);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_request_bytes);
    RUN_TEST(test_parse_fields);
    RUN_TEST(test_health_check_latches_after_three_failures);
    RUN_TEST(test_request_cadence);
    RUN_TEST(test_short_reply_per_profile);
    RUN_TEST(test_max_temperature_below_zero);
    return UNITY_END();
}
