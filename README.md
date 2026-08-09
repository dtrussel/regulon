# Regulon

[![CI - C11 Library](https://github.com/dtrussel/regulon/actions/workflows/ci_c.yml/badge.svg)](https://github.com/dtrussel/regulon/actions/workflows/ci_c.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Regulon** (`ron`) is a spec-driven, embedded-safe control-systems library:
PID, filters, feed-forward, gain scheduling, trajectory generation, cascade
control, Kalman filtering, state-space/observer control, LQR/LQG optimal
control, relay auto-tuning, health monitoring, and runtime performance
metrics — all with no dynamic allocation, no recursion, and formally
verified safety properties where it matters.

The library is developed from a single set of requirements
(`docs/specs/SRS_ControlLib.rst`) down through architecture
(`SADS_ControlLib.rst`), API (`IS_ControlLib.rst`), and test specification
(`TP_ControlLib.rst`) — every public function traces back to a requirement
ID and forward to a test ID.

## Implementations

| | Status | Location |
|---|---|---|
| **C11** | Complete — all 14 modules implemented, tested, and release-hardened | [`regulon-c/`](regulon-c/) |
| **Rust** (Edition 2021) | Early stage — PID and signal-conditioning filters ported so far | [`regulon-rs/`](regulon-rs/) |

The rest of this README covers the C11 implementation, which is the
primary, production-ready track. See `docs/plans/rust/rust-first-rollout.md`
for the Rust port's status and plan.

## Modules (C11)

| Module | Header | Summary |
|---|---|---|
| PID | `ron_pid.h` | Parallel-form PID with 2-DOF setpoint weighting, back-calculation/clamping anti-windup, output saturation and rate limiting, input/output normalisation, and a latched fault register. |
| Filters | `ron_filter.h` | First-order low-pass, moving-average FIR, biquad IIR (cascaded sections, coefficient hot-swap), and a rate-of-change limiter. |
| Feed-forward | `ron_feedforward.h` | Static/velocity/acceleration/external feed-forward terms integrated into the PID output path. |
| Gain scheduling | `ron_gain_sched.h` | Hard-switch or interpolated gain scheduling over a breakpoint table, applied atomically to a PID instance. |
| Trajectory | `ron_trajectory.h` | Trapezoidal and jerk-limited S-curve motion profile generators. |
| Cascade | `ron_cascade.h` | Outer/inner PID composition with coordinated saturation and anti-windup propagation. |
| Kalman filter | `ron_kalman.h` | Discrete linear Kalman filter — predict/update, Cholesky innovation solve, Joseph-form update, steady-state mode. |
| State-space / observer | `ron_statespace.h`, `ron_observer.h` | Discrete state-feedback control with integral augmentation and a Luenberger observer; external, Luenberger, or Kalman state-estimate sources. |
| LQR | `ron_lqr.h` | Discrete-time MIMO Linear Quadratic Regulator with an in-library DARE solver (iterative value recursion), pre-computed or DARE gain modes. |
| LQG | `ron_lqg.h` | LQR combined with an embedded Kalman estimator via the separation principle — independent dual-DARE initialisation. |
| Auto-tune | `ron_autotune.h` | Relay-feedback (Åström–Hägglund) excitation, zero-crossing Ku/Tu estimation, Ziegler-Nichols/Tyreus-Luyben/conservative tuning rules. |
| Health monitor | `ron_health.h` | Passive control-loop supervisor: output-stuck, diverging, oscillating, sensor-dropout, and setpoint-unreachable detection. |
| Metrics | `ron_metrics.h` | Passive runtime performance accumulator: IAE/ISE/ITAE, peak overshoot, rise time, settling time. |
| Aggregate | `ron.h` | Single include for the whole library, gated by generated `RON_HAVE_<MODULE>` macros. |

Every module is independently selectable at configure time via
`RON_ENABLE_<MODULE>` CMake options (all default `ON`); dependencies
between modules (e.g. LQR forcing on state-space and Kalman) are resolved
automatically.

## Design constraints

No `malloc`/`free`, no recursion, no VLAs, no `goto`/`setjmp`, no global
mutable state, no unbounded loops. All storage is caller-owned (typically a
file-scope `static`). Single-precision `float` by default; `RON_USE_DOUBLE`
selects `double` at compile time. Safe to call from an ISR: every `_step()`
function has bounded, allocation-free execution.

## Quickstart

```bash
# Host build + tests
cmake -B build -S regulon-c -DRON_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure

# Try the examples (pid_quickstart, cascade_control_loop, kalman_estimation,
# statespace_observer, autotune_relay, trajectory_motion, lqr_lqg_control)
cmake -B build -S regulon-c -DRON_BUILD_TESTS=OFF -DRON_BUILD_EXAMPLES=ON
cmake --build build
./build/examples/pid_quickstart
```

```c
#include "ron/ron.h"

static ron_pid_instance_t pid;

void init(void) {
    ron_pid_config_t cfg = {
        .Kp = 2.0F, .Ki = 5.0F, .Kd = 0.0F,
        .b = 1.0F, .c = 1.0F,
        .u_min = -10.0F, .u_max = 10.0F,
        .I_min = -100.0F, .I_max = 100.0F,
        .aw_mode = RON_AW_BACK_CALC, .T_aw = 0.05F,
    };
    (void)ron_pid_init(&pid, &cfg);
}

void control_isr(ron_float_t setpoint, ron_float_t measurement, ron_float_t dt) {
    ron_float_t u;
    ron_status_t status;
    (void)ron_pid_step(&pid, setpoint, measurement, dt, &u, &status);
    actuator_set(u);
}
```

### Using an installed package

```bash
cmake -B build -S regulon-c -DCMAKE_INSTALL_PREFIX=/opt/regulon
cmake --build build
cmake --install build
```

CMake consumers:

```cmake
find_package(regulon REQUIRED)
target_link_libraries(my_target PRIVATE regulon::regulon)
```

Non-CMake consumers (pkg-config):

```bash
gcc app.c $(pkg-config --cflags --libs regulon) -o app
```

### Using it from Zephyr

Regulon ships as a [Zephyr module](https://docs.zephyrproject.org/latest/develop/modules.html).
Add it to your west manifest, then enable it with one Kconfig option:

```cfg
CONFIG_REGULON=y
```

Each optional module has a `CONFIG_REGULON_<MODULE>` option mirroring the
CMake ones, with dependencies resolved through Kconfig `select`. A runnable
control-loop sample lives in `zephyr/samples/pid_loop/`:

```bash
cd zephyr/samples/pid_loop
west build -b native_sim/native/64 . -- -DZEPHYR_EXTRA_MODULES=$(pwd)/../../..
./build/zephyr/zephyr.exe
```

See the [Zephyr integration guide](docs/guides/zephyr.rst) for precision and
FPU selection, thread/ISR ownership, and sample-period handling.

### Cross-compiling

Toolchain files are provided for ARM Cortex-M (`gcc`/`clang`) and RISC-V
(`rv32imc`) under `regulon-c/cmake/toolchains/`:

```bash
cmake -B build_arm -S regulon-c \
      -DCMAKE_TOOLCHAIN_FILE=regulon-c/cmake/toolchains/arm-none-eabi.cmake \
      -DCMAKE_C_FLAGS="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb" \
      -DRON_BUILD_TESTS=OFF
cmake --build build_arm
```

## Verification

Every push and pull request runs the full [`ci_c.yml`](.github/workflows/ci_c.yml)
matrix: GCC + ASan/UBSan, GCC double-precision, Clang, `clang-format`,
`cppcheck`/MISRA C:2023, complexity (`lizard -C 10`), 100% statement and
branch coverage (LLVM `llvm-cov`), CBMC formal proofs (bounded-memory /
output-saturation properties), ARM and RISC-V cross-compile smoke builds, a
source-manifest drift check, a minimal-subset build, the example programs,
a timing benchmark, a package-install smoke test, and a documentation
build with warnings promoted to errors.

## Documentation

The documentation site is built with Sphinx and Breathe, and covers the API
reference, the specifications, and the usage guides together. Requirement and
test IDs on each API entry link back into the specs.

```bash
python3 -m venv .venv
.venv/bin/pip install -r docs/requirements.txt
.venv/bin/sphinx-build -b html docs docs/_build/html
# open docs/_build/html/index.html
```

Doxygen runs automatically as Breathe's XML backend, so `sphinx-build` is the
whole build — but it does need `doxygen` and `graphviz` on `PATH`. Set
`REGULON_SKIP_DOXYGEN=1` to reuse existing XML when iterating on prose.

Every push builds the site with warnings promoted to errors; publishing to
GitHub Pages is the manually-triggered
[`docs_c.yml`](.github/workflows/docs_c.yml) workflow.

## Repository layout

```
docs/                <- Sphinx documentation site (conf.py, guides/, api/)
docs/specs/          <- SRS/SADS/IS/TP — requirements, architecture, API, and test specs (ground truth)
docs/plans/          <- Per-phase implementation plans and closure evidence
docs/deviations/     <- MISRA C:2023 deviation records
regulon-c/           <- C11 implementation
zephyr/              <- Zephyr module manifest, Kconfig, build glue, and sample
regulon-rs/          <- Rust Edition 2021 implementation (in progress)
```

See [`AGENTS.md`](AGENTS.md) and [`regulon-c/AGENTS.md`](regulon-c/AGENTS.md)
for the spec-first development workflow, coding rules, and the required
evidence for any change.

## License

MIT — see [`LICENSE`](LICENSE).
