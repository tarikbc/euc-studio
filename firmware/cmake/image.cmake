# euc_add_image(<name> BOARD <board-id> SOURCES <file>...)
#
# Links one firmware image for a board and writes, next to each other in the build directory:
#   <name>.elf   linked image with debug info (for the emulator, GDB, nm)
#   <name>.bin   raw flash image starting at the link address (what the OTA flasher sends)
#   <name>.map   GNU ld map file
#   size-report.json   per-module flash/RAM totals, checked against size-budget.toml (over budget = failed build)
# For the Begode Extreme it also checks that profiles/begode-extreme/probes.toml matches struct app
# (tools/gen_probes.py --check; regenerate with the `probes` target).
#
# The board target (euc_board_<board-id>) brings the linker script, the board headers and the MCU port.
function(euc_add_image name)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "BOARD" "SOURCES")
    if(NOT arg_BOARD OR NOT arg_SOURCES)
        message(FATAL_ERROR "euc_add_image(${name}): BOARD and SOURCES are required")
    endif()

    # board.h owns the board id string written into .euc_info; it must name the same board as the directory.
    file(STRINGS "${PROJECT_SOURCE_DIR}/boards/${arg_BOARD}/board.h" board_id_line REGEX "^#define BOARD_ID ")
    if(NOT board_id_line MATCHES "\"${arg_BOARD}\"")
        message(FATAL_ERROR
            "boards/${arg_BOARD}/board.h: BOARD_ID must be \"${arg_BOARD}\" (found: ${board_id_line})")
    endif()

    string(REPLACE "-" "_" board_target "euc_board_${arg_BOARD}")
    set(out_dir "${CMAKE_BINARY_DIR}")
    set(elf "${out_dir}/${name}.elf")
    set(bin "${out_dir}/${name}.bin")
    set(map "${out_dir}/${name}.map")

    add_executable(${name} ${arg_SOURCES})
    euc_target_defaults(${name})
    target_link_libraries(${name} PRIVATE ${board_target} euc_core euc_profile euc_build_info)
    set_target_properties(${name} PROPERTIES
        SUFFIX ".elf"
        RUNTIME_OUTPUT_DIRECTORY "${out_dir}")

    # -nostdlib: no C library and no crt0; ports/stm32f1 provides the reset path and the few libc functions
    # the compiler may call. libgcc is linked back explicitly for soft-float and 64-bit division helpers.
    target_link_options(${name} PRIVATE
        -nostdlib
        "LINKER:--gc-sections"
        "LINKER:-Map=${map}"
        "LINKER:--fatal-warnings")
    target_link_libraries(${name} PRIVATE gcc)

    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O binary "${elf}" "${bin}"
        COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/size_report.py"
            --map "${map}"
            --bin "${bin}"
            --budget "${PROJECT_SOURCE_DIR}/size-budget.toml"
            --board "${arg_BOARD}"
            --out "${out_dir}/size-report.json"
        BYPRODUCTS "${bin}" "${map}" "${out_dir}/size-report.json"
        COMMENT "Image ${name}.bin and size report"
        VERBATIM)
    if(arg_BOARD STREQUAL "begode-extreme")
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/gen_probes.py"
                --cc "${CMAKE_C_COMPILER}" --check
            COMMENT "Simulator probes match struct app"
            VERBATIM)
    endif()
endfunction()
