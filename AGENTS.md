# AGENTS.md — Regulon Control Systems Library

## Identity
Library: **Regulon** | Abbreviation: `ron` | C prefix: `ron_` / `RON_` | Rust: `ron::` module namespace.  
Two first-class implementations: **C11** (`regulon-c/`) and **Rust Edition 2021** (`regulon-rs/`).

## Layout
```
docs/specs/SRS_ControlLib.rst   ← RON-SRS-001  requirements (ground truth)
docs/specs/SADS_ControlLib.rst  ← RON-SADS-001 architecture & pseudocode
docs/specs/IS_ControlLib.rst    ← RON-IS-001   API headers, naming, build
docs/specs/TP_ControlLib.rst    ← RON-TP-001   test cases & traceability
regulon-c/include/ron/          ← public C headers  (ron_*.h)
regulon-c/src/                  ← C sources         (ron_*.c)
regulon-c/test/unit/            ← Unity tests       (test_ron_*.c)
regulon-c/test/formal/          ← CBMC harnesses    (*_proof.c)
regulon-rs/regulon/src/         ← Rust library crate (#![no_std])
regulon-rs/regulon-sys/src/     ← C-ABI wrapper     (extern "C")
```

## Before writing code
1. Find the requirement IDs: `grep -n "RON-FR-0XX" docs/specs/SRS_ControlLib.rst`
2. Read the SADS pseudocode for the affected module.
3. Find the test IDs: `grep "RON-FR-0XX" docs/specs/TP_ControlLib.rst`
4. Write the test function first, then the implementation.
5. Every test ID **must already exist** in `TP_ControlLib.rst` before you write it.

## C11 rules
- **File header** (mandatory on every `.c`/`.h`): `@file`, `@brief`, `@doc RON-IS-001`, `@req RON-FR-xxx`, `SPDX-License-Identifier: MIT`
- **Every function** must have `/* Satisfies: RON-FR-xxx | Test: RON-TC-xxx-NNN */` above it.
- **Naming**: public `ron_<module>_<verb>`, internal `static <module>_<verb>`, types `ron_<noun>_t`, macros `RON_<SCREAMING>`.
- **Permitted headers**: `<stdint.h>`, `<stdbool.h>`, `<float.h>` only. `<math.h>` only in coefficient helpers.
- **Error pattern**: null-check → init-check → fault-latch → input validation → computation (in that order, every public function).
- **Coding standard**: MISRA C:2023. Run `cppcheck --addon=misra.py --error-exitcode=1 regulon-c/src/`.

## Rust rules
- **File header** (mandatory on every `.rs`): `//!` module doc with `**Satisfies:** RON-FR-xxx`, `**Tests:** RON-TC-xxx`, `SPDX-License-Identifier: MIT`.
- **Every function/impl**: `/// **Satisfies:** RON-FR-xxx`.  Every `unsafe` block: `// SAFETY: <reason>`.
- **Lint gates** in every module: `#![deny(clippy::all, clippy::pedantic, missing_docs)]`.
- **No `std::`** in `regulon/src/` — the crate is `#![no_std]`. Use `libm` feature for math.
- **No `unwrap()`/`panic!`** in library code — return `Result<_, RonError>`.
- **Const generics** for all matrix dimensions (Kalman, state-space, observer). Never runtime-sized slices.
- **Coding standard**: MISRA Rust:2024. Run `cargo clippy -- -D warnings -D clippy::pedantic`.

## Test IDs
Format: `RON-TC-<MODULE>-<NNN>` (e.g. `RON-TC-PID-015`). Formal variants: `RON-TC-PID-015-FV`.  
C function name: `test_ron_tc_pid_015`. Rust function name: `ron_tc_pid_015`. Kani: `#[kani::proof] fn ron_tc_pid_015_fv()`.

## Build
```bash
# C — host (tests + sanitisers)
cmake -B regulon-c/build -S regulon-c -DRON_BUILD_TESTS=ON -DCMAKE_C_FLAGS="-fsanitize=address,undefined"
cmake --build regulon-c/build && cd regulon-c/build && ctest --output-junit results.xml

# Rust — host tests
cd regulon-rs && cargo test --workspace

# Cross-compile C (ARM example)
cmake -B regulon-c/build/arm -S regulon-c -DCMAKE_TOOLCHAIN_FILE=regulon-c/cmake/toolchains/arm-none-eabi.cmake

# Cross-compile Rust
cargo build --target thumbv7em-none-eabihf
```

## Analysis & coverage
```bash
# C: MISRA, complexity, formatting
cppcheck --addon=misra.py --error-exitcode=1 regulon-c/src/
lizard regulon-c/src/ -C 10 --warnings-only
clang-format --dry-run --Werror regulon-c/src/*.c

# Rust: lint, format, audit
cargo clippy -- -D warnings -D clippy::pedantic && cargo fmt --check && cargo audit

# Coverage (must reach 100% statement + branch)
# C:    cmake with --coverage, then lcov + genhtml
# Rust: cargo llvm-cov nextest --workspace --mcdc

# Formal verification
cbmc --unwind 32 --bounds-check regulon-c/test/formal/<harness>.c regulon-c/src/*.c -I regulon-c/include
cargo kani --workspace
```

## Absolute prohibitions
`malloc/free/Box/Vec` · recursion · VLAs · `goto/setjmp` · `panic!/unwrap()` in lib · `std::` in `regulon/src/` · global mutable state · magic numbers · `int`/`long` without width · unbounded loops · implicit numeric casts (C)

## PR checklist
- [ ] Every new function: `Satisfies:` + `Test:` annotation.  
- [ ] Every new test ID exists in `TP_ControlLib.rst`.  
- [ ] `cppcheck --addon=misra.py` / `cargo clippy -D pedantic` exit 0.  
- [ ] 100% statement + branch coverage for changed files.  
- [ ] `CHANGELOG.rst` updated.
