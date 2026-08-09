#!/usr/bin/env bash
# =============================================================================
# check_stack_usage.sh — fail if any function's stack frame exceeds a budget.
#
# The library is used from interrupt handlers and from RTOS threads whose
# stacks are sized in single kilobytes, so a frame that quietly grows is a
# real defect rather than a style issue.  It is also an easy one to introduce:
# every scratch matrix in the estimator and optimal-control modules is sized
# at RON_MAT_MAX_DIM regardless of the dimensions actually configured, so one
# extra local costs the square of that bound in every call.
#
# The build already emits -fstack-usage data (see regulon-c/CMakeLists.txt);
# this reads it back.
#
# Usage: check_stack_usage.sh <build-dir> [budget-bytes]
#
# RON-PR-022, RON-SR-003
# SPDX-License-Identifier: MIT
# =============================================================================
set -euo pipefail

BUILD_DIR="${1:?usage: check_stack_usage.sh <build-dir> [budget-bytes]}"
BUDGET="${2:-768}"

mapfile -t SU_FILES < <(find "$BUILD_DIR" -name '*.su' 2>/dev/null)
if [ "${#SU_FILES[@]}" -eq 0 ]; then
    echo "error: no .su files under '$BUILD_DIR' — build with GCC first." >&2
    exit 2
fi

# Each .su line is: <file>:<line>:<col>:<function>\t<bytes>\t<qualifier>
worst=0
worst_fn=""
violations=0

while IFS=$'\t' read -r location bytes _qualifier; do
    [ -n "${bytes:-}" ] || continue
    fn="${location##*:}"
    if [ "$bytes" -gt "$worst" ]; then
        worst="$bytes"
        worst_fn="$fn"
    fi
    if [ "$bytes" -gt "$BUDGET" ]; then
        echo "::error::${fn} uses ${bytes} B of stack, over the ${BUDGET} B budget"
        violations=$((violations + 1))
    fi
done < <(cat "${SU_FILES[@]}")

echo "largest stack frame: ${worst} B (${worst_fn}); budget ${BUDGET} B"

if [ "$violations" -gt 0 ]; then
    echo "error: ${violations} function(s) over the stack budget." >&2
    exit 1
fi

echo "Stack usage within budget."
