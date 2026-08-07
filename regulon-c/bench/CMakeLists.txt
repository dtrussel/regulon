# =============================================================================
# bench/CMakeLists.txt — Host timing benchmarks for the Regulon C11 library
#
# Benchmarks are host-only, informational programs (they use <stdio.h> and
# <time.h>) that report per-call timing for each module's step function
# against the RON-PR-003 design target (>= 10 kHz sample rate, i.e. a
# generous 100 microsecond per-step budget on the benchmarking host). They
# are NOT a substitute for target-specific WCET analysis (RON-PR-001/003
# explicitly defer the certified budget to target-specific integration) and
# are not part of any production verification gate.
#
# RON-IS-001 §8.1 | RON-PR-001, RON-PR-003
# SPDX-License-Identifier: MIT
# =============================================================================

add_executable(bench_step bench_step.c)
target_link_libraries(bench_step PRIVATE
    regulon
    $<$<AND:$<NOT:$<C_COMPILER_ID:MSVC>>,$<NOT:$<PLATFORM_ID:Windows>>>:m>
)
target_compile_options(bench_step PRIVATE
    $<$<C_COMPILER_ID:GNU>:  -Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow>
    $<$<C_COMPILER_ID:Clang>:-Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow>
    $<$<C_COMPILER_ID:MSVC>: /W4 /WX>
)
