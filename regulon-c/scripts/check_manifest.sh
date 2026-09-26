#!/usr/bin/env bash
# =============================================================================
# check_manifest.sh — Guard the CI source manifests against drift.
#
# The CI gates (format, cppcheck/MISRA, complexity, coverage, CBMC) read their
# file lists from scripts/lib_sources.txt and scripts/format_files.txt instead
# of hard-coding paths in every job.  This script fails if those manifests have
# drifted from what is actually on disk, so a newly added production source can
# never silently escape a gate.
#
# Run from the repository root:  bash regulon-c/scripts/check_manifest.sh
#
# RON-IS-001 §8.1 | RON-TP-001
# SPDX-License-Identifier: MIT
# =============================================================================
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$here"

lib_manifest="regulon-c/scripts/lib_sources.txt"
fmt_manifest="regulon-c/scripts/format_files.txt"
status=0

# Strip blank lines / comments and sort a manifest.
read_manifest() { grep -vE '^\s*(#.*)?$' "$1" | sort; }

# 1. lib_sources.txt must be exactly the set of production .c files in src/.
disk_sources="$(find regulon-c/src -name '*.c' | sort)"
manifest_sources="$(read_manifest "$lib_manifest")"
if ! diff <(printf '%s\n' "$disk_sources") <(printf '%s\n' "$manifest_sources") >/dev/null; then
    echo "ERROR: lib_sources.txt is out of sync with regulon-c/src/*.c:"
    diff <(printf '%s\n' "$disk_sources") <(printf '%s\n' "$manifest_sources") || true
    status=1
fi

# 2. Every production source and header (src/*.c, src/*.h, include/ron/*.h)
#    must appear in the format manifest.
fmt_set="$(read_manifest "$fmt_manifest")"
for f in $(find regulon-c/src -name '*.c' -o -name '*.h' | sort) \
         $(find regulon-c/include/ron -name '*.h' | sort); do
    if ! grep -qxF "$f" <<<"$fmt_set"; then
        echo "ERROR: $f is missing from format_files.txt"
        status=1
    fi
done

# 3. Every manifest entry must exist on disk.
for f in $manifest_sources $fmt_set; do
    if [ ! -f "$f" ]; then
        echo "ERROR: manifest lists a missing file: $f"
        status=1
    fi
done

# 4. The version in ron_platform.h must match project(VERSION) in CMakeLists.
cmake_version="$(sed -nE 's/^[[:space:]]*VERSION[[:space:]]+([0-9]+\.[0-9]+\.[0-9]+).*/\1/p' \
    regulon-c/CMakeLists.txt | head -n 1)"
header_version="$(for part in MAJOR MINOR PATCH; do
    sed -nE "s/^#define RON_VERSION_${part} ([0-9]+)U$/\1/p" regulon-c/include/ron/ron_platform.h
done | paste -sd. -)"
if [ "$cmake_version" != "$header_version" ]; then
    echo "ERROR: CMakeLists.txt version $cmake_version != ron_platform.h RON_VERSION_* $header_version"
    status=1
fi

if [ "$status" -eq 0 ]; then
    echo "Manifests are in sync."
fi
exit "$status"
