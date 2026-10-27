# SIL port: the firmware compiled for the host as a shared library the simulator engine loads (docs/sil.md).
# Host builds compile core with the runtime profile (core/CMakeLists.txt), so quirks are switches here.

# The quirk switches alone, for the unit tests and the library.
add_library(euc_port_sim OBJECT sil.c)
euc_target_defaults(euc_port_sim)
target_include_directories(euc_port_sim PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(euc_port_sim PUBLIC euc_core)
set_target_properties(euc_port_sim PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    C_VISIBILITY_PRESET hidden)

# The board the library runs: the Begode Extreme app (its own sources, as the image builds them) on the host
# port, which drives the board's registers through the engine.
set(EUC_SIL_BOARD "begode-extreme")
set(sil_app_dir "${PROJECT_SOURCE_DIR}/apps/${EUC_SIL_BOARD}")
set(sil_board_dir "${PROJECT_SOURCE_DIR}/boards/${EUC_SIL_BOARD}")

# The storage page addresses, from the board's linker script: the one place the memory map is written down.
file(STRINGS "${sil_board_dir}/link_app.ld" sil_page_lines REGEX "^ld_[a-z_]+_page = 0x[0-9A-Fa-f]+;")
set(sil_definitions "")
foreach(page settings calibrated imu_offsets update_request)
    set(address "")
    foreach(line IN LISTS sil_page_lines)
        if(line MATCHES "^ld_${page}_page = (0x[0-9A-Fa-f]+);")
            set(address "${CMAKE_MATCH_1}")
        endif()
    endforeach()
    if(NOT address)
        message(FATAL_ERROR "link_app.ld of ${EUC_SIL_BOARD} has no ld_${page}_page")
    endif()
    string(TOUPPER "${page}" page_upper)
    list(APPEND sil_definitions "SIL_${page_upper}_PAGE_ADDRESS=${address}U")
endforeach()
# Where the image (its vector table) starts, which the reset path points VTOR at.
file(STRINGS "${sil_board_dir}/link_app.ld" sil_flash_line REGEX "^ *FLASH \\(rx\\) *: *ORIGIN = 0x[0-9A-Fa-f]+")
if(NOT sil_flash_line MATCHES "ORIGIN = (0x[0-9A-Fa-f]+)")
    message(FATAL_ERROR "link_app.ld of ${EUC_SIL_BOARD} has no FLASH origin")
endif()
list(APPEND sil_definitions "SIL_IMAGE_ADDRESS=${CMAKE_MATCH_1}U")
if(EUC_FEATURE_IMU_TLM)
    list(APPEND sil_definitions "EUC_FEATURE_IMU_TLM=1")
endif()

# The probe list (tools/gen_probes.py, the source of profiles/begode-extreme/probes.toml), as a C X-macro.
set(sil_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(sil_probes_h "${sil_generated_dir}/euc_sil_probes.h")
add_custom_command(
    OUTPUT "${sil_probes_h}"
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/gen_probes.py" --emit-sil "${sil_probes_h}"
    DEPENDS "${PROJECT_SOURCE_DIR}/tools/gen_probes.py"
    COMMENT "SIL probe list"
    VERBATIM)

add_library(euc_sil_firmware OBJECT
    host.c
    port.c
    "${sil_app_dir}/control.c"
    "${sil_app_dir}/main.c"
    "${sil_probes_h}")
euc_target_defaults(euc_sil_firmware)
target_include_directories(euc_sil_firmware PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${sil_app_dir}"
    "${sil_board_dir}"
    "${PROJECT_SOURCE_DIR}/ports/stm32f1/include"
    "${sil_generated_dir}")
target_compile_definitions(euc_sil_firmware PRIVATE ${sil_definitions})
target_link_libraries(euc_sil_firmware PUBLIC euc_port_sim)
set_target_properties(euc_sil_firmware PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    C_VISIBILITY_PRESET hidden)

# build/<preset>/libeuc_sil.{so,dylib}
add_library(euc_sil SHARED)
target_link_libraries(euc_sil PRIVATE euc_port_sim euc_sil_firmware)
set_target_properties(euc_sil PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
# The app reads its storage pages through the linker symbols ld_*_page, which the target's linker script puts
# on the flash pages. Here the link makes them aliases of the port's host copies (host.c).
foreach(page settings calibrated imu_offsets update_request)
    if(APPLE)
        target_link_options(euc_sil PRIVATE
            "LINKER:-alias,_sil_${page}_page,_ld_${page}_page"
            "LINKER:-unexported_symbol,_ld_${page}_page")
    else()
        target_link_options(euc_sil PRIVATE "LINKER:--defsym=ld_${page}_page=sil_${page}_page")
    endif()
endforeach()
