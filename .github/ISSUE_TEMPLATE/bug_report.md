---
name: Bug report
about: Something in the library doesn't behave as specified
title: "[bug] "
labels: bug
---

**Implementation affected**
- [ ] C11 (`regulon-c/`)
- [ ] Rust (`regulon-rs/`)

**Module / file**
e.g. `ron_lqr` / `regulon-c/src/ron_lqr.c`

**Requirement / test ID (if known)**
e.g. `RON-FR-733`, `RON-TC-LQR-003` — see `docs/specs/SRS_ControlLib.rst` /
`TP_ControlLib.rst`.

**Expected behavior**
What the spec (or your reasonable expectation) says should happen.

**Actual behavior**
What actually happens.

**Minimal reproduction**
A minimal C/Rust snippet, failing unit test, or CBMC harness that
reproduces the issue. Include build flags (e.g. `RON_USE_DOUBLE`,
compiler, sanitizers) if relevant.

**Environment**
- Compiler/toolchain and version:
- OS/target:
- Commit or tag:

**Additional context**
Anything else that would help (stack trace, sanitizer output, CBMC
counterexample, etc).
