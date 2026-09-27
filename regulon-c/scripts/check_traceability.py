#!/usr/bin/env python3
# =============================================================================
# check_traceability.py — Guard requirement <-> test <-> code traceability.
#
# AGENTS.md requires every test ID to exist in TP_ControlLib.rst before it is
# used, and every requirement to be verified by a test. This script enforces
# both mechanically so neither can drift silently:
#
#   1. Every RON-TC-* ID referenced by the C sources, headers or tests, or by
#      the Rust crate, is defined in TP_ControlLib.rst.
#   2. Every requirement ID cited by the C sources, headers or tests, or by the
#      Rust crate, is defined in SRS_ControlLib.rst.
#   3. Every SRS requirement (FR/SR/PR/QR/DC) is referenced by TP_ControlLib.rst.
#
# Functional requirements that no C source cites are reported but do not fail
# the check: some are verified by analysis or by the Rust track only.
#
# IDs are "defined" in TP by a section title that opens with the ID, or by a
# list-table row whose first cell is the ID. Ranges written as
# "RON-TC-CASC-001 – CASC-007" or "RON-FR-750 – FR-759" are expanded wherever
# they appear.
#
# Run from anywhere:  python3 regulon-c/scripts/check_traceability.py
#
# RON-IS-001 §8.1 | RON-TP-001
# SPDX-License-Identifier: MIT
# =============================================================================
"""Fail if requirement, test-plan and code IDs are out of sync."""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRS = ROOT / "docs" / "specs" / "SRS_ControlLib.rst"
TP = ROOT / "docs" / "specs" / "TP_ControlLib.rst"
CODE_DIRS = [ROOT / "regulon-c" / d for d in ("src", "include", "test")]
RUST_DIRS = [ROOT / "regulon-rs"]

REQ_KINDS = ("FR", "SR", "PR", "QR", "DC")
REQ = r"RON-(?:%s)-\d{3}" % "|".join(REQ_KINDS)
TC = r"RON-TC-[A-Z]+-\d{3}(?:-FV)?"
# "<ID> – <ID>", "<ID> - KIND-NNN" or "<ID>..<ID>" (en dash, hyphen or dots).
RANGE_TAIL = r"\s*(?:[–-]|\.\.)\s*(?:RON-)?(?:TC-)?(?:[A-Z]+-)?(\d{3})(?!\d)"


def explicit(text: str, id_pattern: str) -> set[str]:
    """IDs written out in text; range endpoints count, range interiors do not."""
    return set(re.findall(id_pattern, text)) | {
        m.group(1) for m in re.finditer(r"(?:[–-]|\.\.)\s*(%s)" % id_pattern, text)
    }


def expand(text: str, id_pattern: str) -> set[str]:
    """Return every ID matching id_pattern in text, with ranges expanded."""
    ids = explicit(text, id_pattern)
    for m in re.finditer("(%s)%s" % (id_pattern, RANGE_TAIL), text):
        first, last = m.group(1), int(m.group(2))
        head, num = re.match(r"(.*-)(\d{3})", first.removesuffix("-FV")).groups()
        suffix = "-FV" if first.endswith("-FV") else ""
        if last - int(num) > 200:  # not a range, just adjacent IDs
            continue
        for n in range(int(num), last + 1):
            ids.add(f"{head}{n:03d}{suffix}")
    return ids


def defined_in(text: str, id_pattern: str) -> set[str]:
    """IDs defined by a section title or the first cell of a table row."""
    ids: set[str] = set()
    title = re.compile(r"^(%s)\s+[—-]" % id_pattern, re.M)
    row = re.compile(r"^\s*\*\s*-\s*\**(%s(?:%s)?)" % (id_pattern, RANGE_TAIL), re.M)
    ids |= set(title.findall(text))
    for m in row.finditer(text):
        ids |= expand(m.group(1), id_pattern)
    return ids


def main() -> int:
    srs = SRS.read_text(encoding="utf-8")
    tp = TP.read_text(encoding="utf-8")
    code_files = sorted(
        p for d in CODE_DIRS for p in d.rglob("*") if p.suffix in (".c", ".h")
    )
    rust_files = sorted(
        p
        for d in RUST_DIRS
        for p in d.rglob("*.rs")
        if "target" not in p.relative_to(d).parts
    )
    code = "\n".join(p.read_text(encoding="utf-8") for p in code_files)
    rust = "\n".join(p.read_text(encoding="utf-8") for p in rust_files)
    # Test functions carry their ID in the name: test_ron_tc_pid_015[_x] in
    # Unity, ron_tc_pid_015[_x] in cargo test.
    code_tc = explicit(code + rust, TC) | {
        "RON-TC-%s-%s" % (m.group(1).upper(), m.group(2))
        for m in re.finditer(r"\b(?:test_)?ron_tc_([a-z]+)_(\d{3})", code + rust)
    }

    req_defined = defined_in(srs, REQ)
    tc_defined = defined_in(tp, TC)
    # FV variants are also defined by the formal-verification tables, whose
    # first cell may be the harness file rather than the ID.
    tc_defined |= {i for i in expand(tp, TC) if i.endswith("-FV")}

    failures = {
        "Test IDs used in code but not defined in TP_ControlLib.rst":
            code_tc - tc_defined,
        "Requirement IDs cited in code but not defined in SRS_ControlLib.rst":
            explicit(code + rust, REQ) - req_defined,
        "SRS requirements not referenced by TP_ControlLib.rst":
            req_defined - expand(tp, REQ),
    }
    report = {
        "Functional requirements not cited by any C source (info)":
            {i for i in req_defined if i.startswith("RON-FR-")} - expand(code, REQ),
    }

    status = 0
    for title, ids in failures.items():
        if ids:
            status = 1
            print(f"FAIL: {title}:\n  " + " ".join(sorted(ids)))
    for title, ids in report.items():
        if ids:
            print(f"note: {title}:\n  " + " ".join(sorted(ids)))
    if status == 0:
        print(
            f"traceability OK: {len(req_defined)} requirements, "
            f"{len(tc_defined)} test IDs, {len(code_files)} C files, "
            f"{len(rust_files)} Rust files"
        )
    return status


if __name__ == "__main__":
    sys.exit(main())
