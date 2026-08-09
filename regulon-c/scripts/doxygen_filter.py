#!/usr/bin/env python3
# =============================================================================
# doxygen_filter.py — Doxygen INPUT_FILTER for the Regulon C11 headers.
#
# The headers use this repository's own annotation conventions (AGENTS.md)
# rather than stock Doxygen markup.  Doxygen cannot read them as-is, so this
# filter rewrites them on the fly into equivalent Doxygen constructs.  It
# writes the transformed source to stdout and never modifies the file on disk.
#
# Three transformations are applied:
#
#   1. Documentation blocks open with a bare "/*" line, not the "/**" that
#      Doxygen requires.  Such lines become "/**".  Single-line comments and
#      banner comments (e.g. "/* ====...") are deliberately left alone.
#
#   2. The custom file-header tags "@module", "@doc" and "@req" are not
#      Doxygen commands and would otherwise be reported as unknown.  They are
#      rewritten into "@par" sections so their content is both rendered and
#      warning-free.  Continuation lines of a wrapped "@req" list are folded
#      into the same section.
#
#   3. The single-line traceability annotations that sit directly above each
#      declaration,
#
#          /* Satisfies: RON-FR-730 | Test: RON-TC-LQR-001 */
#
#      become "@par Satisfies" / "@par Tests" sections so the requirement and
#      test IDs reach the rendered API reference, where the Sphinx side
#      turns them into links back into the specifications.  When the
#      declaration already has a documentation block above it, the annotation
#      is merged into that block rather than emitted as a second one, since
#      two adjacent blocks would leave Doxygen to pick only one.
#
# Doxygen invokes this as: doxygen_filter.py <path-to-file>
#
# RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================
"""Rewrite Regulon header annotations into Doxygen markup on stdout."""

from __future__ import annotations

import re
import sys

# "/* Satisfies: <ids> | Test: <ids> */" — the annotation above a declaration.
# The "Test:" half is optional; a few annotations carry only requirement IDs.
_TRACE_RE = re.compile(
    r"^/\*\s*Satisfies:\s*(?P<req>.*?)"
    r"(?:\s*\|\s*Tests?:\s*(?P<test>.*?))?\s*\*/\s*$"
)

# A custom file-header tag, e.g. " * @req      RON-FR-730, RON-FR-731,".
_TAG_RE = re.compile(r"^(?P<prefix>\s*\*\s*)@(?P<tag>module|doc|req)\s+(?P<body>.*?)\s*$")

# A free-text "Satisfies:" paragraph written inside an existing documentation
# block, e.g. " * Satisfies: RON-FR-001 - RON-FR-007, RON-FR-020,".  These
# predate the single-line annotations and are often the more detailed of the
# two, so they are promoted to the same structured section rather than dropped.
_INBLOCK_SATISFIES_RE = re.compile(r"^(?P<prefix>\s*\*\s*)Satisfies:\s*(?P<body>.*?)\s*$")

# A continuation line inside a wrapped tag body: " *           RON-FR-735, ...".
# It must not itself start a new tag and must not be the block terminator.
_CONT_RE = re.compile(r"^(?P<prefix>\s*\*\s+)(?P<body>[^@/\s].*?)\s*$")

# Human-readable section titles for the custom tags.
_TAG_TITLES = {
    "module": "Module",
    "doc": "Specification",
    "req": "Requirements",
}


def _emit_par(out: list[str], prefix: str, title: str, body: str) -> None:
    """Append a Doxygen "@par <title>" section carrying *body*."""
    out.append(f"{prefix}@par {title}\n")
    out.append(f"{prefix}{body}\n")


def _merge_trace_into_block(out: list[str], req: str, test: str, skip_req: bool) -> bool:
    """Fold a traceability annotation into the doc block just emitted.

    Returns True when a preceding block was found and extended.  The block is
    identified by its closing "*/" as the last non-blank line emitted so far;
    that terminator is removed, the new sections are appended, and the
    terminator is restored.

    When *skip_req* is set the block already carries a "Satisfies" section
    promoted from its own free-text paragraph.  That version is usually the
    more detailed of the two, so it is kept and only the test IDs are added.
    """
    idx = len(out) - 1
    while idx >= 0 and out[idx].strip() == "":
        idx -= 1
    if idx < 0 or out[idx].strip() != "*/":
        return False

    prefix = out[idx][: out[idx].index("*/")] + "* "
    terminator = out.pop(idx)
    tail = out[idx:]
    del out[idx:]

    if not skip_req:
        out.append(f"{prefix.rstrip()}\n")
        _emit_par(out, prefix, "Satisfies", req)
    if test:
        _emit_par(out, prefix, "Tests", test)
    out.append(terminator)
    out.extend(tail)
    return True


def transform(lines: list[str]) -> list[str]:
    """Apply all three rewrites to *lines* and return the new source."""
    out: list[str] = []
    pending_tag_prefix = ""
    in_tag_body = False
    in_doc_block = False
    # Whether the block currently open, or the one that closed most recently,
    # already produced a "Satisfies" section of its own.
    block_has_satisfies = False
    closed_block_has_satisfies = False

    for line in lines:
        stripped = line.rstrip("\n")

        # 3. Traceability annotation above a declaration.
        trace = _TRACE_RE.match(stripped)
        if trace:
            in_tag_body = False
            req = trace.group("req").strip()
            test = (trace.group("test") or "").strip()
            if _merge_trace_into_block(out, req, test, closed_block_has_satisfies):
                closed_block_has_satisfies = False
                continue
            out.append("/**\n")
            _emit_par(out, " * ", "Satisfies", req)
            if test:
                _emit_par(out, " * ", "Tests", test)
            out.append(" */\n")
            continue

        # 2c. A free-text "Satisfies:" paragraph inside an open doc block.
        if in_doc_block:
            inblock = _INBLOCK_SATISFIES_RE.match(stripped)
            if inblock:
                prefix = inblock.group("prefix")
                _emit_par(out, prefix, "Satisfies", inblock.group("body"))
                pending_tag_prefix = prefix
                in_tag_body = True
                block_has_satisfies = True
                continue

        # 2a. A custom file-header tag opens a wrapped body.
        tag = _TAG_RE.match(stripped)
        if tag:
            prefix = tag.group("prefix")
            _emit_par(out, prefix, _TAG_TITLES[tag.group("tag")], tag.group("body"))
            pending_tag_prefix = prefix
            in_tag_body = True
            continue

        # 2b. Continuation of the tag body started above.
        if in_tag_body:
            cont = _CONT_RE.match(stripped)
            if cont and not stripped.lstrip().startswith("*/"):
                out.append(f"{pending_tag_prefix}{cont.group('body')}\n")
                continue
            in_tag_body = False

        # 1. A bare "/*" opens a documentation block.
        if stripped == "/*":
            out.append("/**\n")
            in_doc_block = True
            block_has_satisfies = False
            continue

        # Track the extent of an explicit documentation block so that rule 2c
        # only fires inside one.  A "/**< ... */" member comment opens and
        # closes on the same line and so leaves the flag clear.
        if stripped.lstrip().startswith("/**"):
            in_doc_block = "*/" not in stripped
            block_has_satisfies = False
        elif in_doc_block and "*/" in stripped:
            in_doc_block = False
            closed_block_has_satisfies = block_has_satisfies

        out.append(line)

    return out


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <file>", file=sys.stderr)
        return 2
    with open(sys.argv[1], encoding="utf-8") as handle:
        lines = handle.readlines()
    sys.stdout.writelines(transform(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
