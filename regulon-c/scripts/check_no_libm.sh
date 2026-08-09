#!/usr/bin/env bash
# =============================================================================
# check_no_libm.sh — assert a built archive references no math-library symbols.
#
# Usage: check_no_libm.sh <nm-tool> <archive.a>
#
# RON-DC-002 requires the library to depend on nothing beyond <stdint.h>,
# <stdbool.h> and <float.h>, and permits <math.h> only for functions that are
# bounded and WCET-analysable. The biquad coefficient helpers compute sin/cos
# from a fixed-length polynomial instead, so a correct build pulls in no libm
# symbol at all.
#
# That is a link-time property, and it is easy to lose silently: one #include
# in one source is enough. This makes it a build gate.
#
# The cross-compile jobs already install no libc for the target, so a libm
# *call* would fail to compile there. This catches the subtler case of a
# reference that resolves against the host or a sysroot that happens to exist.
#
# RON-IS-001 §8.1 | RON-DC-002 | RON-TC-QUAL-019
# SPDX-License-Identifier: MIT
# =============================================================================
set -euo pipefail

if [ "$#" -ne 2 ]; then
    echo "usage: $0 <nm-tool> <archive.a>" >&2
    exit 2
fi

nm_tool="$1"
archive="$2"

if ! command -v "$nm_tool" >/dev/null 2>&1; then
    echo "error: nm tool '$nm_tool' not found on PATH" >&2
    exit 2
fi
if [ ! -f "$archive" ]; then
    echo "error: archive '$archive' does not exist" >&2
    exit 2
fi

# Every double- and float-suffixed entry point in <math.h> §7.12. Listed
# explicitly rather than pattern-matched: a pattern loose enough to catch
# `sin` also catches `ron_sin_...`, and one tight enough to avoid that misses
# the next function someone reaches for.
libm_symbols='
acos acosf acosh acoshf asin asinf asinh asinhf atan atan2 atan2f atanf
atanh atanhf cbrt cbrtf ceil ceilf copysign copysignf cos cosf cosh coshf
erf erfc erfcf erff exp exp2 exp2f expf expm1 expm1f fabs fabsf fdim fdimf
floor floorf fma fmaf fmax fmaxf fmin fminf fmod fmodf frexp frexpf hypot
hypotf ilogb ilogbf ldexp ldexpf lgamma lgammaf llrint llrintf llround
llroundf log log10 log10f log1p log1pf log2 log2f logb logbf logf lrint
lrintf lround lroundf modf modff nan nanf nearbyint nearbyintf nextafter
nextafterf pow powf remainder remainderf remquo remquof rint rintf round
roundf scalbln scalblnf scalbn scalbnf sin sinf sinh sinhf sqrt sqrtf tan
tanf tanh tanhf tgamma tgammaf trunc truncf
'

undefined=$("$nm_tool" --undefined-only "$archive" 2>/dev/null |
            awk '{ print $NF }' | sort -u)

found=""
for sym in $libm_symbols; do
    if printf '%s\n' "$undefined" | grep -qx "$sym"; then
        found="$found $sym"
    fi
done

if [ -n "$found" ]; then
    echo "::error::$archive references libm symbols, violating RON-DC-002:$found"
    echo ""
    echo "The library must compute what it needs with bounded arithmetic."
    echo "See regulon-c/src/ron_filter.c for the sin/cos it uses instead."
    exit 1
fi

echo "OK: $archive references no libm symbols (RON-DC-002)"
