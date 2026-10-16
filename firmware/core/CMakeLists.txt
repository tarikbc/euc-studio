# core: pure C logic shared by target images and the SIL library. No HAL calls, no mutable globals.

# Headers only: core/include and config/quirks.def. For code that picks its own quirk profile (unit tests of
# the compile-time switches).
add_library(euc_core_headers INTERFACE)
target_include_directories(euc_core_headers INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/config")

# FOC lookup tables (quarter-wave sine, circle limitation), computed from their formulas for every build.
set(EUC_MOTOR_TABLES_C "${CMAKE_CURRENT_BINARY_DIR}/generated/motor_tables.c")
add_custom_command(
    OUTPUT "${EUC_MOTOR_TABLES_C}"
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/gen_motor_tables.py" "${EUC_MOTOR_TABLES_C}"
    DEPENDS "${PROJECT_SOURCE_DIR}/tools/gen_motor_tables.py"
    COMMENT "Generating the FOC lookup tables"
    VERBATIM)
# Generated data in the build tree: clang-tidy would look for its configuration there, and has nothing to
# check in two constant arrays.
set_source_files_properties("${EUC_MOTOR_TABLES_C}" PROPERTIES SKIP_LINTING ON)

# The core modules, built once per configuration with its quirk profile: compat or clean constants for
# target images, run-time switches on the host (the SIL port owns the mask and provides
# euc_quirk_is_active(), so host consumers link euc_port_sim too).
add_library(euc_core STATIC
    alarms/alarms.c
    attitude/attitude.c
    balance/balance.c
    balance/cutoff.c
    balance/law.c
    balance/tiltback.c
    bms/bms.c
    display/display.c
    lights/lights.c
    lights/strip.c
    math/libm.c
    motor/current.c
    motor/foc_math.c
    motor/hall.c
    motor/motor.c
    motor/pid.c
    power/power.c
    protocol/commands.c
    protocol/telemetry.c
    sensors/sensors.c
    service/service.c
    settings/settings.c
    settings/storage.c
    "${EUC_MOTOR_TABLES_C}")
euc_target_defaults(euc_core)
target_link_libraries(euc_core PUBLIC euc_core_headers)
if(CMAKE_CROSSCOMPILING)
    target_link_libraries(euc_core PUBLIC euc_profile)
else()
    target_compile_definitions(euc_core PUBLIC EUC_PROFILE_RUNTIME=1)
    set_target_properties(euc_core PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        C_VISIBILITY_PRESET hidden)
endif()
