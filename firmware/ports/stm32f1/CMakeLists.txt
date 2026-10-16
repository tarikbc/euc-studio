# STM32F1 port: reset path, vector table, system set-up, register-level drivers and the libc pieces a
# -nostdlib image needs. A static library: the linker pulls startup.c through ENTRY(Reset_Handler) in the board
# linker script, and the rest through the references it makes.
add_library(euc_port_stm32f1 STATIC
    backup.c
    clock.c
    freestanding.c
    gpio.c
    motor_hw.c
    peripherals.c
    serial.c
    startup.c
    system.c)
euc_target_defaults(euc_port_stm32f1)
target_include_directories(euc_port_stm32f1 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
# The drivers test quirks (HSE start-up) like core does.
target_link_libraries(euc_port_stm32f1 PUBLIC cmsis_core euc_core_headers euc_profile)

# Keep GCC from recognising the loops in memcpy/memset as memcpy/memset calls (infinite recursion).
# clang-tidy cannot parse that GCC-only option, so this one file is not linted (SKIP_LINTING, CMake 3.27+).
set_source_files_properties(freestanding.c PROPERTIES
    COMPILE_OPTIONS "-fno-tree-loop-distribute-patterns"
    SKIP_LINTING ON)
