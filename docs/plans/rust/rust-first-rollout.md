# Rust-First Rollout Status

Date: 2026-04-11

## Plan Summary

`regulon-rs` is the primary Rust product track. It is driven by the shared specifications, but it is not coupled to the C implementation shape or C ABI. The delivery plan is iteration-based:

1. Iteration 1: workspace, platform/error foundations, Rust-native PID API, PID implementation, safety tests, host and embedded build validation.
2. Iteration 2: close PID verification gaps on the local host, add coverage/audit evidence where possible, and start the first reusable filter slice.
3. Iteration 3: start the feed-forward sprint with the lowest-risk PID extension first, then follow with gain scheduling once the PID/feed-forward surface is stable.
4. Iteration 4: continue with the remaining gain-scheduling scope, then cascade/trajectory, then estimator/state-space modules, then autotune/health/metrics.

## Current Progress

### Iteration 1

Status: implemented

Implemented:
- `regulon-rs` Cargo workspace and `regulon` crate.
- `platform`, `error`, and `pid` modules.
- Rust-native `Pid`/`PidConfig` API with encapsulated state.
- PID features: parallel and ideal-form config support, Euler and trapezoidal integration, derivative on error/measurement, optional normalization, saturation, rate limiting, anti-windup, bumpless manual/automatic switching, fault latching, state snapshots, and runtime config updates.
- Traceable Rust unit tests for the implemented PID/safety slice.
- Host tests, `cargo fmt`, `cargo clippy`, and embedded cross-build on `thumbv7em-none-eabihf`.

Remaining after Iteration 1:
- `cargo audit` evidence.
- Local coverage evidence.
- Local Kani execution is not available on this Windows host. Official Kani documentation currently targets Linux environments, so formal-proof execution remains an environment constraint rather than an unresolved code defect.

### Iteration 2

Status: implemented locally, with one environment-limited verification gap

Implemented:
- `cargo audit` runs successfully on the workspace.
- `cargo-llvm-cov` and `llvm-tools-preview` are installed locally and the coverage command runs successfully.
- First reusable filter slice is implemented:
  - `RON-FR-100` to `RON-FR-103`
  - `RON-FR-110` to `RON-FR-111`
  - `RON-FR-130` to `RON-FR-131`
- Traceable Rust tests were added for the LP1 filter and standalone asymmetric rate limiter.
- Host test count increased to 37 passing tests.

Measured evidence:
- `cargo fmt --all`: passes
- `cargo test --workspace`: passes
- `cargo clippy --workspace -- -D warnings -D clippy::all -D clippy::pedantic`: passes
- `cargo audit`: passes
- `cargo llvm-cov --workspace --summary-only`: runs successfully
- Current local coverage snapshot:
  - Regions: 84.51%
  - Lines: 88.79%

Remaining gap:
- Local Kani execution is still unavailable on this Windows host. Formal-proof execution remains pending on a supported Linux environment.

## Definition Of Done By Iteration

### Iteration 1 done
- PID baseline compiles on host and embedded target.
- PID baseline passes Rust unit tests.
- Lint gate passes with pedantic warnings denied.

### Iteration 2 target
- Coverage command runs locally or the local blocker is documented precisely.
- Filter slice compiles, is exported publicly, and has traceable tests.
- Changelog reflects the new Rust rollout and filter work.

### Iteration 3 target
- Start the feed-forward sprint with a bounded first story rather than all FF modes at once.
- Deliver static-gain feed-forward inside the Rust PID API, with disabled-path parity maintained.
- Extend PID diagnostics so the feed-forward contribution is inspectable by the caller.
- Add traceable tests for the delivered feed-forward slice and rerun the existing quality gates.

## Current Sprint

### Iteration 3

Status: in progress

Sprint goal:
- Extend the PID module with the first feed-forward increment while preserving the current PID baseline and quality gates.

Sprint stories:
1. Story FF-01: static-gain feed-forward (`RON-FR-200`, `RON-FR-201a`, `RON-FR-203`, `RON-FR-204`, `RON-FR-205`).
2. Story FF-02: velocity/acceleration/external feed-forward modes (`RON-FR-201b-d`, `RON-FR-202`) after FF-01 is stable.
3. Story GS-01: gain-scheduling table and hard/interpolated updates once the PID + feed-forward config surface is stable.

Story FF-01 current status:
- Implemented in `regulon-rs` as the first Iteration 3 delivery slice.
- Rust-native PID config now carries a bounded feed-forward configuration with disabled and static-gain modes.
- PID diagnostics now expose the last feed-forward contribution and a `FEED_FORWARD_ACTIVE` status bit.
- Traceable tests added for `RON-TC-FF-002` and `RON-TC-FF-008`.

Iteration 3 evidence so far:
- `cargo fmt --all`: passes
- `cargo test --workspace`: passes with 39 tests
- `cargo clippy --workspace -- -D warnings -D clippy::all -D clippy::pedantic`: passes
- `cargo build --workspace --target thumbv7em-none-eabihf`: passes
- `cargo audit`: passes
- `cargo llvm-cov --workspace --summary-only`: passes
- Updated local coverage snapshot:
  - Regions: 84.69%
  - Lines: 88.70%

Remaining Iteration 3 scope:
- Story FF-02 remains open.
- Story GS-01 remains open.
- Coverage is still below the spec target and needs additional branch/test closure work in later slices.

## Notes

- The anti-windup recovery test in the Rust PID suite uses a simple plant surrogate to measure recovery improvement versus no anti-windup. This keeps the test aligned with the intended behavioral contrast while remaining deterministic on the host.
- Formal verification remains part of the plan, but local execution of Kani is blocked by host-platform support rather than missing code hooks.
- Iteration 3 is intentionally starting with static-gain feed-forward only. The higher-order feed-forward modes and gain scheduling remain planned work, not implied parity commitments in the current code.
