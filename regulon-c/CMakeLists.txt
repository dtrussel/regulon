cmake_minimum_required(VERSION 3.21)

project(regulon
    VERSION     0.1.0
    DESCRIPTION "Regulon Control Systems Library — C11 Implementation"
    LANGUAGES   C
)

# ---------------------------------------------------------------------------
# Language standard
# ---------------------------------------------------------------------------
set(CMAKE_C_STANDARD          11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS        OFF)

# GNUInstallDirs must be included before it is referenced by
# target_include_directories() below, not just in the Install section.
include(GNUInstallDirs)

# ---------------------------------------------------------------------------
# Build options (precision, tests, examples, per-module selection)
# ---------------------------------------------------------------------------
include(cmake/ron_options.cmake)

# ---------------------------------------------------------------------------
# Module dependency resolution
#
# State-space depends on the Kalman filter (ron_statespace.c calls ron_kf_*),
# so enabling state-space forces the Kalman module on.  LQR requires both
# Kalman and state-space (shares the observer and matrix helper).  LQG
# requires LQR (and therefore transitively Kalman + state-space).  Kalman,
# state-space, observer, LQR, and LQG all require the internal matrix helper.
# ---------------------------------------------------------------------------
if(RON_ENABLE_LQG AND NOT RON_ENABLE_LQR)
    message(STATUS "regulon: RON_ENABLE_LQG forces RON_ENABLE_LQR ON")
    set(RON_ENABLE_LQR ON CACHE BOOL "" FORCE)
endif()
if(RON_ENABLE_LQR AND NOT RON_ENABLE_STATESPACE)
    message(STATUS "regulon: RON_ENABLE_LQR forces RON_ENABLE_STATESPACE ON")
    set(RON_ENABLE_STATESPACE ON CACHE BOOL "" FORCE)
endif()
if(RON_ENABLE_STATESPACE AND NOT RON_ENABLE_KALMAN)
    message(STATUS "regulon: RON_ENABLE_STATESPACE forces RON_ENABLE_KALMAN ON")
    set(RON_ENABLE_KALMAN ON CACHE BOOL "" FORCE)
endif()

# ---------------------------------------------------------------------------
# Generated module-availability header (ron/ron_modules.h)
# ---------------------------------------------------------------------------
foreach(_mod FILTER GAIN_SCHED TRAJECTORY CASCADE KALMAN STATESPACE LQR LQG AUTOTUNE HEALTH METRICS)
    if(RON_ENABLE_${_mod})
        set(RON_HAVE_${_mod} 1)
    else()
        set(RON_HAVE_${_mod} 0)
    endif()
endforeach()

set(RON_GENERATED_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/include)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ron_modules.h.in
    ${RON_GENERATED_INCLUDE_DIR}/ron/ron_modules.h
    @ONLY
)

# ---------------------------------------------------------------------------
# Library source list (mandatory baseline + selected modules)
# ---------------------------------------------------------------------------
# Mandatory baseline: PID core + integrated feed-forward path.
set(RON_LIB_SOURCES
    src/ron_pid_api.c
    src/ron_pid_config.c
    src/ron_pid_core.c
    src/ron_pid_fault.c
    src/ron_feedforward.c
)

if(RON_ENABLE_FILTER)
    list(APPEND RON_LIB_SOURCES src/ron_filter.c)
endif()
if(RON_ENABLE_GAIN_SCHED)
    list(APPEND RON_LIB_SOURCES src/ron_gain_sched.c)
endif()
if(RON_ENABLE_TRAJECTORY)
    list(APPEND RON_LIB_SOURCES src/ron_trajectory_trap.c src/ron_trajectory_scurve.c)
endif()
if(RON_ENABLE_CASCADE)
    list(APPEND RON_LIB_SOURCES src/ron_cascade.c)
endif()
if(RON_ENABLE_AUTOTUNE)
    list(APPEND RON_LIB_SOURCES src/ron_autotune.c)
endif()
if(RON_ENABLE_HEALTH)
    list(APPEND RON_LIB_SOURCES src/ron_health.c)
endif()
if(RON_ENABLE_METRICS)
    list(APPEND RON_LIB_SOURCES src/ron_metrics.c)
endif()
# Matrix / state-estimation modules (share the internal ron_matrix helper).
if(RON_ENABLE_KALMAN OR RON_ENABLE_STATESPACE OR RON_ENABLE_LQR OR RON_ENABLE_LQG)
    list(APPEND RON_LIB_SOURCES src/ron_matrix.c)
endif()
if(RON_ENABLE_KALMAN)
    list(APPEND RON_LIB_SOURCES src/ron_kalman.c)
endif()
if(RON_ENABLE_STATESPACE)
    list(APPEND RON_LIB_SOURCES src/ron_statespace.c src/ron_observer.c)
endif()
if(RON_ENABLE_LQR)
    list(APPEND RON_LIB_SOURCES src/ron_lqr.c)
endif()
if(RON_ENABLE_LQG)
    list(APPEND RON_LIB_SOURCES src/ron_lqg.c)
endif()

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(regulon STATIC ${RON_LIB_SOURCES})

target_include_directories(regulon
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${RON_GENERATED_INCLUDE_DIR}>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_library(regulon::regulon ALIAS regulon)

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
# Examples (host only — never cross-compiled; not part of any production gate)
# ---------------------------------------------------------------------------
if(RON_BUILD_EXAMPLES AND NOT CMAKE_CROSSCOMPILING)
    add_subdirectory(examples)
endif()

# ---------------------------------------------------------------------------
# Benchmarks (host only — never cross-compiled; informational, not a gate)
# ---------------------------------------------------------------------------
if(RON_BUILD_BENCHMARKS AND NOT CMAKE_CROSSCOMPILING)
    add_subdirectory(bench)
endif()

# ---------------------------------------------------------------------------
# Install: static library, headers, and a relocatable CMake package
# (regulonConfig.cmake / regulonConfigVersion.cmake / regulonTargets.cmake)
# so downstream consumers can `find_package(regulon)` and link
# `regulon::regulon`, plus a pkg-config .pc file for non-CMake consumers.
# ---------------------------------------------------------------------------
include(CMakePackageConfigHelpers)

install(TARGETS regulon
    EXPORT      regulonTargets
    ARCHIVE     DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ron DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${RON_GENERATED_INCLUDE_DIR}/ron/ron_modules.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ron
)

install(EXPORT regulonTargets
    FILE        regulonTargets.cmake
    NAMESPACE   regulon::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/regulon
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/regulonConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/regulonConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/regulon
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/regulonConfigVersion.cmake
    VERSION       ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/regulonConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/regulonConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/regulon
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/regulon.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/regulon.pc
    @ONLY
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/regulon.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)
