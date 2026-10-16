# Host unit tests (Unity). Run with: ctest --preset host-debug   (or host-sil)

# Shared test data at the repository root (real captures from wheels).
cmake_path(SET EUC_TEST_DATA_DIR NORMALIZE "${PROJECT_SOURCE_DIR}/../tests/data")

# euc_add_unit_test(<name> SOURCES <file>... [LIBRARIES <target>...] [DEFINITIONS <macro>...])
function(euc_add_unit_test name)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "" "SOURCES;LIBRARIES;DEFINITIONS")
    add_executable(${name} ${arg_SOURCES})
    euc_target_defaults(${name})
    target_link_libraries(${name} PRIVATE unity ${arg_LIBRARIES})
    target_compile_definitions(${name} PRIVATE ${arg_DEFINITIONS})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# Header-only parts of core, with the quirk profile each test picks.
euc_add_unit_test(test_fixed
    SOURCES unit/test_fixed.c
    LIBRARIES euc_core_headers)

euc_add_unit_test(test_quirks_compat
    SOURCES unit/test_quirks.c
    LIBRARIES euc_core_headers
    DEFINITIONS EUC_PROFILE_COMPAT=1)

euc_add_unit_test(test_quirks_clean
    SOURCES unit/test_quirks.c
    LIBRARIES euc_core_headers
    DEFINITIONS EUC_PROFILE_CLEAN=1)

# Core modules as the SIL library builds them: run-time quirks, switched through the SIL port
# (euc_port_sim brings euc_core along).
euc_add_unit_test(test_sil
    SOURCES unit/test_sil.c
    LIBRARIES euc_port_sim)

euc_add_unit_test(test_telemetry
    SOURCES unit/test_telemetry.c
    LIBRARIES euc_port_sim
    DEFINITIONS "EUC_TEST_DATA_DIR=\"${EUC_TEST_DATA_DIR}\"")

euc_add_unit_test(test_commands
    SOURCES unit/test_commands.c
    LIBRARIES euc_port_sim)

euc_add_unit_test(test_math
    SOURCES unit/test_math.c
    LIBRARIES euc_port_sim)

euc_add_unit_test(test_attitude
    SOURCES unit/test_attitude.c
    LIBRARIES euc_port_sim)

euc_add_unit_test(test_balance
    SOURCES unit/test_balance.c
    LIBRARIES euc_port_sim)
