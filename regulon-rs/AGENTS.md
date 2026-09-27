# AGENTS.md - Regulon Rust Implementation

This file applies to `regulon-rs/`. Repository-wide requirements and traceability rules remain in `../AGENTS.md`.

## Layout
```
regulon/src/       <- Rust library crate (#![no_std]); one directory per module
                      with mod.rs, tests.rs and, where proofs exist, proofs.rs
```

The crate is for Rust-native users only. There is no C-ABI wrapper crate;
C and C++ firmware uses the C11 track in `../regulon-c/`.

## Rust Rules
- **File header** (mandatory on every `.rs`): `//!` module doc with `**Satisfies:** RON-FR-xxx`, `**Tests:** RON-TC-xxx`, `SPDX-License-Identifier: MIT`.
- **Every function/impl**: `/// **Satisfies:** RON-FR-xxx`.
- **No `unsafe`** anywhere in the crate.
- **Lint gates** in every module: `#![deny(clippy::all, clippy::pedantic, missing_docs)]`.
- **No `std::`** in `regulon/src/` - the crate is `#![no_std]`. Use `libm` feature for math.
- **No `unwrap()`/`panic!`** in library code - return `Result<_, RonError>`.
- **Const generics** for all matrix dimensions (Kalman, state-space, observer). Never runtime-sized slices.
- **Coding standard**: MISRA Rust:2024. Run `cargo clippy -- -D warnings -D clippy::pedantic`.

## Rust Test IDs
Rust unit/property function name: `ron_tc_pid_015`.
Kani proof name: `#[kani::proof] fn ron_tc_pid_015_fv()`.

## Build
```bash
cd regulon-rs
cargo test --workspace

# Cross-compile Rust
cargo build --target thumbv7em-none-eabihf
```

## Analysis & Coverage
```bash
cd regulon-rs
cargo clippy -- -D warnings -D clippy::pedantic
cargo fmt --check
cargo audit

# Coverage / MC/DC where available
cargo llvm-cov nextest --workspace --mcdc
```

## Formal Verification
```bash
cd regulon-rs
cargo kani --workspace
```

## CI (`.github/workflows/ci_rust.yml`)
Every job can be reproduced locally from `regulon-rs/`:
`cargo fmt --all --check`; `cargo clippy --workspace [--features double_precision] -- -D warnings -D clippy::all -D clippy::pedantic`;
`cargo test --workspace [--features double_precision]` (stable, plus beta single precision);
`cargo build --workspace --release --target thumbv7em-none-eabihf`; `cargo kani --workspace`;
`cargo audit`; `cargo llvm-cov --workspace --lcov`; and from the repo root
`python3 regulon-c/scripts/check_traceability.py`. CI sets `RUSTFLAGS=-D warnings`.

## Rust-Specific Prohibitions
`Box/Vec` | recursion | `panic!/unwrap()` in library code | `std::` in `regulon/src/` | global mutable state | magic numbers | unbounded loops
