#!/usr/bin/env bash
# =============================================================================
# doxygen_filter.sh — Doxygen INPUT_FILTER for the Regulon C11 headers.
#
# Every header's file-level and struct/function documentation blocks open
# with a bare "/*" line (see AGENTS.md's mandatory @file/@brief/@doc/@req
# header convention), not the "/**" Doxygen requires to treat a comment as
# documentation. This filter rewrites only that exact opening-line pattern
# to "/**" so Doxygen picks up the existing prose without requiring every
# header to be rewritten. It never touches single-line comments (e.g. the
# "/* Satisfies: ... | Test: ... */" traceability annotations above each
# function), since those don't match the bare "/*" line pattern.
#
# Doxygen invokes this as: doxygen_filter.sh <path-to-file>, and expects the
# filtered content on stdout; it never modifies the file on disk.
#
# RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================
set -euo pipefail

sed 's|^/\*$|/**|' "$1"
