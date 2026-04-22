# AGENTS.md - Regulon Control Systems Library

## Identity
Library: **Regulon** | Abbreviation: `ron`.
Two first-class implementations: **C11** (`regulon-c/`) and **Rust Edition 2021** (`regulon-rs/`).

Implementation-specific guidance is intentionally split:
- C11 guidance: `regulon-c/AGENTS.md`
- Rust guidance: `regulon-rs/AGENTS.md`

## Layout
```
docs/specs/SRS_ControlLib.rst   <- RON-SRS-001  requirements (ground truth)
docs/specs/SADS_ControlLib.rst  <- RON-SADS-001 architecture & pseudocode
docs/specs/IS_ControlLib.rst    <- RON-IS-001   API headers, naming, build
docs/specs/TP_ControlLib.rst    <- RON-TP-001   test cases & traceability
regulon-c/                      <- C11 implementation
regulon-rs/                     <- Rust Edition 2021 implementation
```

## Before Writing Code
1. Find the requirement IDs: `rg -n "RON-FR-0XX" docs/specs/SRS_ControlLib.rst`
2. Read the SADS pseudocode for the affected module.
3. Find the test IDs: `rg "RON-FR-0XX" docs/specs/TP_ControlLib.rst`
4. Write or update the test/proof first, then the implementation.
5. Every test ID **must already exist** in `TP_ControlLib.rst` before you write it.
6. Keep implementation scope aligned with the active iteration plan and do not start future modules opportunistically.

## Test IDs
Format: `RON-TC-<MODULE>-<NNN>` (e.g. `RON-TC-PID-015`). Formal variants: `RON-TC-PID-015-FV`.
Record all new or changed test/proof claims in `docs/specs/TP_ControlLib.rst`.

## Shared Prohibitions
`malloc/free/Box/Vec` | recursion | VLAs | `goto/setjmp` | `panic!/unwrap()` in library code | global mutable state | magic numbers | unbounded loops

## PR Checklist
- [ ] Every new function has a `Satisfies:` + `Test:` annotation in the implementation-specific style.
- [ ] Every new test ID exists in `TP_ControlLib.rst`.
- [ ] Relevant implementation-specific lint, format, test, coverage, and formal gates pass.
- [ ] `CHANGELOG.rst` updated.
