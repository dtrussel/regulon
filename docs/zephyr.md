<!-- SPDX-License-Identifier: MIT -->
# Using Regulon as a Zephyr Module

Regulon ships as a [Zephyr](https://www.zephyrproject.org/) module. The library
performs no dynamic allocation and is `<math.h>`-free, so it links against
Zephyr's **minimal libc** (no libm) and works on bare-metal MCU targets.

The module is defined by:

- `zephyr/module.yml` — module manifest.
- `regulon-c/zephyr/Kconfig` — `CONFIG_REGULON*` options.
- `regulon-c/zephyr/CMakeLists.txt` — `zephyr_library()` build glue.

## Enabling it

Add to your application's `prj.conf`:

```conf
CONFIG_REGULON=y
```

then include the aggregate header (or any individual module header) in your code:

```c
#include <ron/ron.h>
```

## Adding the module to your workspace

You can consume Regulon either way:

### T2 — west manifest (recommended for projects already using west)

Add Regulon as a project in your application's `west.yml`:

```yaml
manifest:
  remotes:
    - name: dtrussel
      url-base: https://github.com/dtrussel
  projects:
    - name: regulon
      remote: dtrussel
      revision: main
      path: modules/regulon
```

then `west update`. Zephyr discovers the module via its `zephyr/module.yml`.

### T3 — ZEPHYR_EXTRA_MODULES (quick local integration)

Point Zephyr at a local checkout without touching your manifest:

```sh
west build -b <board> app -- -DZEPHYR_EXTRA_MODULES=/path/to/regulon
```

(or set `ZEPHYR_EXTRA_MODULES` in the environment / your app `CMakeLists.txt`).

## Kconfig options

| Option | Default | Effect |
| --- | --- | --- |
| `CONFIG_REGULON` | n | Enable the library (PID + feed-forward baseline) |
| `CONFIG_REGULON_USE_DOUBLE` | n | Use `double` instead of `float` for `ron_float_t` |
| `CONFIG_REGULON_ENABLE_ASSERT` | n | Compile internal `RON_ASSERT` checks (trap on violation) |
| `CONFIG_REGULON_FILTER` | y | Signal-conditioning filters |
| `CONFIG_REGULON_GAIN_SCHED` | y | Gain scheduling |
| `CONFIG_REGULON_TRAJECTORY` | y | Trapezoidal / S-curve trajectory generators |
| `CONFIG_REGULON_CASCADE` | y | Cascade controller |
| `CONFIG_REGULON_KALMAN` | y | Discrete Kalman filter |
| `CONFIG_REGULON_STATESPACE` | y | State-space controller + observer (selects `KALMAN`) |
| `CONFIG_REGULON_AUTOTUNE` | y | Relay-feedback auto-tuner |
| `CONFIG_REGULON_HEALTH` | y | Control-loop health monitor |
| `CONFIG_REGULON_METRICS` | y | Runtime performance metrics |

Disable the modules you don't need to shrink the build; the PID core and its
integrated feed-forward path are always present.

## Sample

`samples/zephyr/regulon_pid` drives a first-order plant to a setpoint with a
Regulon PID. Build and run it on the simulator:

```sh
west build -b native_sim samples/zephyr/regulon_pid -- -DZEPHYR_EXTRA_MODULES=$PWD
west build -t run
```

It is also exercised in CI via twister on `native_sim` and `qemu_cortex_m3`
(including a `CONFIG_MINIMAL_LIBC=y` build that proves the no-libm property).
