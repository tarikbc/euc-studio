# Style and static analysis targets. They work from any preset's build directory:
#   format        rewrite every firmware source with clang-format
#   format-check  fail if any firmware source is not clang-format clean
#   cppcheck      cppcheck over the sources this build compiles, with the MISRA C:2012 addon on firmware code
#   quirks-doc    regenerate docs/quirks.md from config/quirks.def (CI runs the --check form)
#   probes        target presets: regenerate profiles/begode-extreme/probes.toml (the image build checks it)

file(GLOB_RECURSE EUC_FORMAT_SOURCES CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/apps/*.[ch]"
    "${PROJECT_SOURCE_DIR}/boards/*.[ch]"
    "${PROJECT_SOURCE_DIR}/core/*.[ch]"
    "${PROJECT_SOURCE_DIR}/ports/*.[ch]"
    "${PROJECT_SOURCE_DIR}/tests/*.[ch]")

find_program(EUC_CLANG_FORMAT_EXE clang-format)
if(EUC_CLANG_FORMAT_EXE)
    add_custom_target(format
        COMMAND "${EUC_CLANG_FORMAT_EXE}" -i ${EUC_FORMAT_SOURCES}
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        VERBATIM)
    add_custom_target(format-check
        COMMAND "${EUC_CLANG_FORMAT_EXE}" --dry-run --Werror ${EUC_FORMAT_SOURCES}
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        VERBATIM)
endif()

find_program(EUC_CPPCHECK_EXE cppcheck)
if(EUC_CPPCHECK_EXE)
    option(EUC_CPPCHECK_MISRA "Run the cppcheck MISRA C:2012 addon on firmware sources (not on tests)" ON)
    set(cppcheck_common
        "--project=${CMAKE_BINARY_DIR}/compile_commands.json"
        "--suppress=*:${CMAKE_BINARY_DIR}/*"
        "--suppressions-list=${PROJECT_SOURCE_DIR}/tools/cppcheck-suppressions.txt"
        --enable=warning,style,performance,portability
        --std=c11
        --inline-suppr
        --error-exitcode=1
        --quiet)
    if(CMAKE_CROSSCOMPILING)
        # The compile database leaves out the headers the cross compiler finds on its own (newlib, CMSIS is
        # -isystem). Without them cppcheck guesses about memcpy, __DSB and friends; their own findings are
        # suppressed.
        foreach(dir IN LISTS CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES)
            list(APPEND cppcheck_common "-I${dir}" "--suppress=*:${dir}/*")
        endforeach()
        list(APPEND cppcheck_common "-I${cmsis6_SOURCE_DIR}/CMSIS/Core/Include")
    endif()
    set(cppcheck_misra "")
    if(EUC_CPPCHECK_MISRA)
        set(cppcheck_misra "--addon=misra")
    endif()
    # Firmware sources get MISRA; test code (Unity macros, host builds only) gets the general checks.
    set(cppcheck_tests_command "")
    if(NOT CMAKE_CROSSCOMPILING)
        set(cppcheck_tests_command
            COMMAND "${EUC_CPPCHECK_EXE}" ${cppcheck_common}
                "--cppcheck-build-dir=${CMAKE_BINARY_DIR}/cppcheck/tests"
                "--file-filter=${PROJECT_SOURCE_DIR}/tests/*")
    endif()
    add_custom_target(cppcheck
        COMMAND "${EUC_CPPCHECK_EXE}" ${cppcheck_common} ${cppcheck_misra}
            "--cppcheck-build-dir=${CMAKE_BINARY_DIR}/cppcheck/firmware"
            "--file-filter=${PROJECT_SOURCE_DIR}/apps/*"
            "--file-filter=${PROJECT_SOURCE_DIR}/core/*"
            "--file-filter=${PROJECT_SOURCE_DIR}/ports/*"
        ${cppcheck_tests_command}
        COMMAND_EXPAND_LISTS
        VERBATIM)
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/cppcheck/firmware" "${CMAKE_BINARY_DIR}/cppcheck/tests")
endif()

add_custom_target(quirks-doc
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/gen_quirks_doc.py"
    VERBATIM)

if(CMAKE_CROSSCOMPILING)
    add_custom_target(probes
        COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/gen_probes.py" --cc "${CMAKE_C_COMPILER}"
        VERBATIM)
endif()
