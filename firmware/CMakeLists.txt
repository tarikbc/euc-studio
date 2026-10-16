# EUC Studio firmware. One CMake tree builds two worlds:
#   - target images for the controller MCU (arm-none-eabi cross build, presets target-*), and
#   - host code: unit tests and the software-in-the-loop (SIL) library the simulator loads (presets host-*).
# Layer rules and commands: README.md.
cmake_minimum_required(VERSION 3.25)

project(euc_firmware
    VERSION 0.1.0
    DESCRIPTION "EUC Studio controller firmware"
    LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(EUC_BOARD "begode-extreme" CACHE STRING "Board the target image is built for (directory under boards/)")
set(EUC_PROFILE "compat" CACHE STRING
    "Target image profile: compat (stock behaviour) or clean (known defects fixed)")
set_property(CACHE EUC_PROFILE PROPERTY STRINGS compat clean)
option(EUC_DEV_BUILD "Set the dev flag in the .euc_info block (unsigned image)" ON)
option(EUC_SANITIZERS "Host builds: compile with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(EUC_CLANG_TIDY "Run clang-tidy on every firmware source while it compiles" OFF)

find_package(Python3 3.11 REQUIRED COMPONENTS Interpreter)

include(cmake/compile_options.cmake)
include(cmake/dependencies.cmake)
include(cmake/analysis.cmake)

if(CMAKE_CROSSCOMPILING)
    if(NOT EUC_PROFILE MATCHES "^(compat|clean)$")
        message(FATAL_ERROR "EUC_PROFILE must be compat or clean, not '${EUC_PROFILE}'")
    endif()
    string(TOUPPER "${EUC_PROFILE}" profile_upper)
    add_library(euc_profile INTERFACE)
    target_compile_definitions(euc_profile INTERFACE EUC_PROFILE_${profile_upper}=1)
endif()

add_subdirectory(core)

if(CMAKE_CROSSCOMPILING)
    include(cmake/build_info.cmake)
    include(cmake/image.cmake)

    add_subdirectory(ports/stm32f1)
    add_subdirectory(boards/${EUC_BOARD})
    add_subdirectory(apps/${EUC_BOARD})
else()
    enable_testing()
    add_subdirectory(ports/sim)
    add_subdirectory(tests)
endif()
