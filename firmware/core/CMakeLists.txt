# core: pure C logic shared by target images and the SIL library. No HAL calls, no mutable globals.

# Headers only: core/include and config/quirks.def. For code that picks its own quirk profile (unit tests of
# the compile-time switches).
add_library(euc_core_headers INTERFACE)
target_include_directories(euc_core_headers INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/config")

# The core modules, built once per configuration with its quirk profile: compat or clean constants for
# target images, run-time switches on the host (the SIL port owns the mask and provides
# euc_quirk_is_active(), so host consumers link euc_port_sim too).
add_library(euc_core STATIC
    protocol/commands.c
    protocol/telemetry.c)
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
