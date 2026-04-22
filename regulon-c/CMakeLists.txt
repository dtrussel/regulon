cmake_minimum_required(VERSION 3.21)

project(regulon
    VERSION     1.0.0
    DESCRIPTION "Regulon Control Systems Library — C11 Implementation"
    LANGUAGES   C
)

# ---------------------------------------------------------------------------
# Language standard
# ---------------------------------------------------------------------------
set(CMAKE_C_STANDARD          11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS        OFF)

# ---------------------------------------------------------------------------
# Build options (RON_USE_DOUBLE, RON_BUILD_TESTS, RON_ENABLE_ASSERT)
# ---------------------------------------------------------------------------
include(cmake/ron_options.cmake)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(regulon STATIC
    src/ron_filter.c
    src/ron_pid_config.c
    src/ron_pid_core.c
    src/ron_pid_fault.c
    src/ron_pid_api.c
)

target_include_directories(regulon
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Strict warning flags — every warning is an error (RON-IS-001 §3.4)
target_compile_options(regulon PRIVATE
    $<$<C_COMPILER_ID:GNU>:
        -Wall -Wextra -Wpedantic -Werror
        -Wconversion -Wshadow -Wundef
        -fno-common -fstack-usage
    >
    $<$<C_COMPILER_ID:Clang>:
        -Wall -Wextra -Wpedantic -Werror
        -Wconversion -Wshadow -Wundef
    >
    $<$<C_COMPILER_ID:MSVC>:
        /W4 /WX
    >
)

# ---------------------------------------------------------------------------
# Compile-time options
# ---------------------------------------------------------------------------
if(RON_USE_DOUBLE)
    target_compile_definitions(regulon PUBLIC RON_USE_DOUBLE=1)
endif()

if(RON_ENABLE_ASSERT)
    target_compile_definitions(regulon PRIVATE
        "RON_ASSERT(cond)=do { if(!(cond)) { __builtin_trap(); } } while(0)"
    )
endif()

# ---------------------------------------------------------------------------
# Tests (host only — skip when cross-compiling)
# ---------------------------------------------------------------------------
if(RON_BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_subdirectory(test)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS regulon ARCHIVE DESTINATION lib)
install(DIRECTORY include/ron DESTINATION include)
