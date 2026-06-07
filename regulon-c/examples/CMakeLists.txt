# =============================================================================
# examples/CMakeLists.txt — Host example programs for the Regulon C11 library
#
# Examples are host-only documentation programs (they use <stdio.h>) and are
# NOT part of any production verification gate.  They are built only when
# RON_BUILD_EXAMPLES=ON and never cross-compiled.  Each links the full library
# through the aggregate ron/ron.h header.
#
# RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================

macro(ron_add_example example_name)
    add_executable(${example_name} ${ARGN})
    target_link_libraries(${example_name} PRIVATE
        regulon
        $<$<AND:$<NOT:$<C_COMPILER_ID:MSVC>>,$<NOT:$<PLATFORM_ID:Windows>>>:m>
    )
    target_compile_options(${example_name} PRIVATE
        $<$<C_COMPILER_ID:GNU>:  -Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow>
        $<$<C_COMPILER_ID:Clang>:-Wall -Wextra -Wpedantic -Werror -Wconversion -Wshadow>
        $<$<C_COMPILER_ID:MSVC>: /W4 /WX>
    )
endmacro()

ron_add_example(pid_quickstart      pid_quickstart.c)
ron_add_example(cascade_control_loop cascade_control_loop.c)
