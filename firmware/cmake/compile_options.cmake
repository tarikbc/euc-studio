# Compiler settings shared by every firmware target. Fetched third-party code (Unity, CMSIS) does not get
# them.

set(EUC_C_WARNINGS
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wdouble-promotion
    -Wstrict-prototypes
    -Wmissing-prototypes
    -Wundef
    -Wvla
    -Wcast-align
    -Werror)

# Sanitizers must cover everything that is linked together, so they are global, not per target.
if(EUC_SANITIZERS AND NOT CMAKE_CROSSCOMPILING)
    add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

if(EUC_CLANG_TIDY)
    # Homebrew keeps clang-tidy in the keg-only llvm formula.
    find_program(EUC_CLANG_TIDY_EXE clang-tidy
        HINTS /opt/homebrew/opt/llvm/bin /usr/local/opt/llvm/bin
        REQUIRED)
    # The compile lines are GCC lines; ask clang not to fail on GCC-only warning and optimisation options.
    set(EUC_CLANG_TIDY_COMMAND
        "${EUC_CLANG_TIDY_EXE}"
        "--extra-arg=-Wno-unknown-warning-option"
        "--extra-arg=-Wno-ignored-optimization-argument"
        "--extra-arg=-Wno-unused-command-line-argument")
    if(CMAKE_CROSSCOMPILING)
        # clang cannot tell the target from the path of arm-none-eabi-gcc, and needs the toolchain's newlib
        # headers through the GCC sysroot.
        execute_process(
            COMMAND "${CMAKE_C_COMPILER}" -print-sysroot
            OUTPUT_VARIABLE gcc_sysroot
            OUTPUT_STRIP_TRAILING_WHITESPACE
            COMMAND_ERROR_IS_FATAL ANY)
        list(APPEND EUC_CLANG_TIDY_COMMAND
            "--extra-arg=--target=arm-none-eabi"
            "--extra-arg=--sysroot=${gcc_sysroot}")
    endif()
endif()

# euc_target_defaults(<target>)
# Applies the firmware warning set and code generation rules to one of our own targets.
function(euc_target_defaults target)
    # -ffp-contract=off: never fuse a*b+c into one FMA. Host and target must round every float operation the
    # same way, or SIL results drift from the image running in the emulator.
    target_compile_options(${target} PRIVATE ${EUC_C_WARNINGS} -ffp-contract=off)
    if(CMAKE_CROSSCOMPILING)
        # One section per function and object lets --gc-sections drop everything nothing references.
        target_compile_options(${target} PRIVATE
            -ffreestanding
            -ffunction-sections
            -fdata-sections
            -fno-common
            -g)
    endif()
    if(EUC_CLANG_TIDY)
        set_property(TARGET ${target} PROPERTY C_CLANG_TIDY ${EUC_CLANG_TIDY_COMMAND})
    endif()
endfunction()
