.. ============================================================
.. CHANGELOG — Regulon Control Systems Library
.. ============================================================

CHANGELOG
=========

All notable changes to the **Regulon Control Systems Library** are documented
here following `Keep a Changelog <https://keepachangelog.com/en/1.0.0/>`_
conventions.  Version numbers follow `Semantic Versioning <https://semver.org/>`_.

------------------------------------------------------------------------

`0.1.0`_ - 2026-08-07
======================

First tagged release. Covers the complete C11 implementation (``regulon-c/``,
``project(regulon VERSION 0.1.0 ...)``) — every roadmap phase from the PID
baseline through Phase 12 (LQR/LQG optimal control) is implemented, tested,
and traceable — plus the release-engineering work below that makes the
repository installable and usable outside its own build tree. The Rust
implementation (``regulon-rs/``) remains early-stage (PID and filters only)
and is not part of this release's completeness claim; see
``docs/plans/rust/rust-first-rollout.md`` for its separate status.

Highlights of this release, in the detailed entries below:

- **C11 library complete**: all 14 modules (PID, filters, feed-forward,
  gain scheduling, trajectory generators, cascade control, Kalman filter,
  state-space controller + Luenberger observer, LQR, LQG, relay
  auto-tuner, health monitor, runtime metrics, aggregate header) are
  implemented with 100% statement/branch coverage, CBMC formal proofs
  where the test plan calls for them, and full MISRA C:2023 traceability.
- **Installable**: a ``find_package(regulon)``-consumable CMake package
  and a ``pkg-config`` file, in addition to the existing in-tree
  ``add_subdirectory`` build.
- **Portable**: ARM Cortex-M and RISC-V (``rv32imc``) cross-compile smoke
  builds, both exercised in CI.
- **Documented**: a top-level ``README.md``, a Sphinx + Breathe
  documentation site carrying the API reference, the specifications and
  the usage guides together with the two cross-linked,
  ``CONTRIBUTING.md``/``SECURITY.md``, and issue/PR templates
  — none of which existed before this release.
- **Measured**: a host timing benchmark against the ``RON-PR-003`` 10 kHz
  design-target budget, and a refreshed MISRA deviations record covering
  the full active source set instead of just the original PID slice.

.. _0.1.0: https://github.com/dtrussel/regulon/releases/tag/v0.1.0

------------------------------------------------------------------------

0.1.0 — Contributor and Security Hygiene Docs
---------------------------------------------

Added
~~~~~
- ``CONTRIBUTING.md``: summarises the spec-first workflow already
  described in ``AGENTS.md``/``regulon-c/AGENTS.md`` for external
  contributors, plus a pre-PR checklist.
- ``SECURITY.md``: vulnerability-reporting process via GitHub private
  security advisories (no personal contact info published).
- ``.github/ISSUE_TEMPLATE/bug_report.md``,
  ``.github/ISSUE_TEMPLATE/feature_request.md``,
  ``.github/ISSUE_TEMPLATE/config.yml`` (disables blank issues, links to
  the security-advisory flow instead of public issues for vulnerabilities).
- ``.github/pull_request_template.md``.

------------------------------------------------------------------------

0.1.0 — Zephyr RTOS Module
--------------------------

Added
~~~~~
- ``zephyr/module.yml``: Zephyr module manifest, making the repository
  consumable through a west manifest with no vendoring or hand-written
  build glue.
- ``zephyr/Kconfig``: ``CONFIG_REGULON`` plus a ``CONFIG_REGULON_<MODULE>``
  option per optional module, mirroring the ``RON_ENABLE_<MODULE>`` CMake
  options. The dependency chain the standalone build resolves imperatively
  is expressed with Kconfig ``select``, so requesting LQG pulls in LQR,
  state-space and the Kalman filter without the user knowing the chain.
  ``CONFIG_REGULON_DOUBLE_PRECISION`` and ``CONFIG_REGULON_ASSERT`` cover
  the two global options.
- ``zephyr/CMakeLists.txt``: builds a ``zephyr_library`` from the same
  sources as the standalone build and generates ``ron/ron_modules.h`` from
  the same template, so ``<ron/ron.h>`` includes exactly the headers whose
  implementations were compiled regardless of which build system produced
  the library. ``RON_USE_DOUBLE`` is applied application-wide, since
  ``ron_float_t`` is part of the ABI.
- ``zephyr/samples/pid_loop/``: a complete Zephyr application running a PID
  loop in a periodic thread against a simulated first-order plant, building
  and running without hardware.
- ``docs/guides/zephyr.rst``: integration guide covering the west manifest,
  the Kconfig options, precision and FPU selection, thread/ISR ownership of
  controller instances, deriving ``dt`` from the scheduler period, fault
  handling, and troubleshooting.

Changed
~~~~~~~
- ``docs/specs/SRS_ControlLib.rst`` (1.2.0 -> 1.2.1): clarified that the
  RTOS scope exclusion concerns kernel coupling — the library still
  contains no task wrappers, synchronisation primitives, or kernel calls —
  and not build-system packaging for an RTOS ecosystem, which is the same
  category as the CMake package and pkg-config file already shipped. No
  requirements added or changed.

Verification evidence
~~~~~~~~~~~~~~~~~~~~~~
Built and run against Zephyr 4.1.1 on ``native_sim/native/64`` with the
host toolchain:

- Full configuration (``CONFIG_REGULON=y``) builds and runs; the sample
  loop converges from 0 to 0.994 against a setpoint of 1.0, showing the
  overshoot expected of a PI controller on a first-order lag.
- Minimal configuration (every optional module ``=n``) links only the five
  mandatory baseline objects, and the generated ``ron_modules.h`` reports
  ``RON_HAVE_*`` as 0 for the excluded modules.
- ``CONFIG_REGULON_LQG=y`` alone resolves ``REGULON_LQR``,
  ``REGULON_STATESPACE`` and ``REGULON_KALMAN`` to ``y`` and links
  ``ron_lqg.c``, ``ron_lqr.c``, ``ron_statespace.c``, ``ron_observer.c``,
  ``ron_kalman.c`` and ``ron_matrix.c``, confirming the ``select`` chain.

Continuous integration
~~~~~~~~~~~~~~~~~~~~~~
- ``zephyr/tests/control/``: a ztest suite checking that each module behaves
  correctly once cross-compiled and executed on an MCU, rather than merely
  linking. The host suite already covers behaviour exhaustively; this covers
  what differs on target - floating point (including targets with no FPU),
  ABI, alignment, and the Kconfig-selected source set with its generated
  ``ron_modules.h``. Tests for optional modules compile only when their
  Kconfig option selected them, which doubles as a check that the
  ``RON_HAVE_*`` macros agree with what was built.
- ``.github/workflows/zephyr_nightly.yml``: daily (and manually
  triggerable) job building the module against a pinned Zephyr release.
  Zephyr is a large dependency and the glue changes rarely, so gating every
  push on it would cost more than it catches; what the job actually guards
  is drift on the Zephyr side, which a daily cadence catches soon enough.

  It asserts behaviour rather than just exit status: the sample must run
  and converge to within 10% of its setpoint, the minimum-footprint
  configuration must link exactly the five baseline objects, the complete
  configuration must link one object per source in
  ``scripts/lib_sources.txt``, and ``CONFIG_REGULON_LQG=y`` alone must
  resolve the LQR/state-space/Kalman chain and link the shared matrix
  helper and observer.

  Beyond the host checks it covers real Cortex-M targets. The behavioural
  suite is **executed under QEMU** on ``qemu_cortex_m3`` (ARMv7-M, soft
  float - no FPU at all) and ``mps2/an521`` (Cortex-M33, ARMv8-M, with
  FPU), and cross-compiled for ``mps2/an386`` (Cortex-M4F, hard float),
  ``mps2/an500`` (Cortex-M7) and ``nrf52840dk/nrf52840`` (Cortex-M4F with a
  vendor HAL). Cortex-M4F and M7 have no in-tree QEMU target, so those are
  compile and link checks; they still cover hard-float ABI selection, FPU
  code generation and vendor HAL integration. Flash and RAM footprints are
  written to the job summary.

  Cost is kept down rather than assumed: the ``native_sim`` job needs no
  SDK at all, the others install only the minimal Zephyr SDK bundle with
  the single ``arm-zephyr-eabi`` toolchain, and Zephyr is fetched as a
  shallow clone with ``manifest.project-filter`` reduced to ``cmsis`` and
  ``hal_nordic`` - roughly 700 MB rather than several gigabytes. Both the
  tree and the SDK are cached.

Documented
~~~~~~~~~~
- ``docs/guides/zephyr.rst`` gained a stack sizing section. Running on
  target surfaced that the matrix modules need far more stack than
  Zephyr's default thread provides: worst single frames are ~2.4 kB for
  the DARE solver, ~1.9 kB for the Kalman covariance update and ~1.3 kB
  for LQG, so a call chain reaches roughly 4 kB against a default of about
  one. Overflowing it does not fault cleanly on Cortex-M without stack
  protection - it corrupts and hangs, which is exactly how it presented -
  so the section gives measured per-module figures and recommends enabling
  stack protection during bring-up. Everything below the estimators (PID,
  filters, trajectory, cascade, autotune, health, metrics) stays under
  512 B and needs no attention.

Fixed
~~~~~
- ``ron_pid_core.c``: initialised ``u_final``, silencing a GCC
  ``-Wmaybe-uninitialized`` false positive that appears at ``-O2`` and
  ``-Os`` (the optimisation level Zephyr builds at). The variable was
  already written on every path where the caller reads it — GCC cannot see
  that the helper writing it returns a non-``RON_FAULT_NONE`` fault
  whenever it does not — so this changes no behaviour, and it satisfies
  MISRA C:2023 Rule 9.1 explicitly rather than by inference. Statement and
  branch coverage of the file remain 100%.

------------------------------------------------------------------------

0.1.0 — Documentation Site (Sphinx + Breathe)
---------------------------------------------

Added
~~~~~
- ``docs/conf.py``, ``docs/index.rst``, ``docs/requirements.txt``: a Sphinx
  site using the Furo theme, MyST, ``sphinx-copybutton``, ``sphinx-design``
  and Graphviz. It unifies three bodies of material that previously lived
  apart — the C11 API reference, the SRS/SADS/IS/TP specification set, and
  hand-written guides. ``conf.py`` runs Doxygen itself and reads the
  version from ``regulon-c/CMakeLists.txt``, so ``sphinx-build`` is the
  whole build and the version is not duplicated.
- ``docs/api/``: per-module reference pages rendered by Breathe from the
  Doxygen XML, plus an ``index`` page documenting the conventions the API
  shares across modules (caller-owned instances, init/step/reset,
  returned faults, explicit ``dt``, compile-time dimension bounds).
- ``docs/guides/``: quickstart, installation and integration, module
  selection, cross-compiling, and a verification/compliance page that
  states plainly what each CI gate proves — and what it does not.
- ``docs/_ext/regulon_trace.py``: a Sphinx extension that records where
  each requirement and test ID is defined (a section whose title opens
  with the ID, or a table row whose first cell is the ID) and rewrites
  bare IDs on the API pages into links to those anchors. 513 links across
  the 15 module pages, all resolving.
- ``regulon-c/scripts/doxygen_filter.py``: replaces the previous shell
  filter. Besides opening documentation blocks it converts the
  repository's ``@module``/``@doc``/``@req`` tags and the per-declaration
  ``/* Satisfies: ... | Test: ... */`` annotations into Doxygen sections,
  merging an annotation into the block above it when one exists rather
  than emitting a second block Doxygen would have to choose between.
- ``.github/workflows/ci_c.yml``: new ``docs-build`` job (job 16) running
  ``sphinx-build -W`` on every push and pull request, with ``docs/**``
  added to the path filters. A best-effort ``linkcheck`` pass runs
  alongside it without gating.

Changed
~~~~~~~
- ``regulon-c/Doxyfile``: Doxygen is now an XML backend for Breathe rather
  than an HTML generator (``GENERATE_HTML=NO``, ``GENERATE_XML=YES``,
  ``OPTIMIZE_OUTPUT_FOR_C=YES``, dot graphs off). ``RON_STATIC_ASSERT`` is
  expanded away via ``PREDEFINED`` so build-time budget assertions stop
  being exported as declarations. With the previous 17 unknown-tag
  warnings eliminated by the new filter, ``WARN_AS_ERROR=FAIL_ON_WARNINGS``
  is now on.
- ``.github/workflows/docs_c.yml``: builds and publishes the Sphinx site.
  Still ``workflow_dispatch``-only, so an unconfigured Pages setup cannot
  break the required ``ci_c.yml`` checks.
- Nine public headers gained ``@brief``/``@param``/``@retval``
  documentation for the 66 functions that had none: ``ron_filter.h`` (22),
  ``ron_lqr.h`` (8), ``ron_trajectory.h`` (8), ``ron_statespace.h`` (7),
  ``ron_lqg.h`` (6), ``ron_kalman.h`` (5), ``ron_observer.h`` (4),
  ``ron_feedforward.h`` (4), ``ron_gain_sched.h`` (2). Return values were
  read from the implementations, so the documented fault codes are the
  ones each function actually returns. Comments only; no declaration or
  behaviour changes.

Fixed
~~~~~
- Rendering the specifications for the first time surfaced latent defects
  in documents that had never been built: four section underlines shorter
  than their titles (``IS_ControlLib.rst``), a ``list-table`` row with a
  stray fourth cell (``SADS_ControlLib.rst``, DD-11) and another with a
  stray third (``TP_ControlLib.rst``), a sub-case list docutils parsed as
  a malformed enumerated list, and a ``figure`` directive pointing at an
  image that was never committed — now a Graphviz context diagram that
  renders from source.
- ``ron_scurve_step``'s ``jrk`` parameter was undocumented; caught by
  enabling ``WARN_AS_ERROR``.
- ``CONTRIBUTING.md`` used repository-relative links that resolved on
  GitHub but not in the rendered site; they are now absolute.

Verification evidence
~~~~~~~~~~~~~~~~~~~~~~
- ``doxygen Doxyfile`` (Doxygen 1.9.8) with ``WARN_AS_ERROR=FAIL_ON_WARNINGS``:
  exits 0, zero warnings.
- ``sphinx-build -W --keep-going -b html docs docs/_build/html`` (Sphinx
  9.0.4, Breathe 4.36.0): exits 0, zero warnings, from a clean virtualenv
  built only from ``docs/requirements.txt``.
- All 176 distinct traceability anchors referenced from the API pages
  resolve to an existing ``id`` in the target document.
- ``clang-format --dry-run --Werror`` over ``format_files.txt``,
  ``check_manifest.sh``, and ``ctest`` (17/17) all pass after the header
  documentation pass.

------------------------------------------------------------------------

0.1.0 — CMake Package Export and pkg-config
-------------------------------------------

Previously ``install(TARGETS regulon ARCHIVE DESTINATION lib)`` copied the
static library and headers but produced no ``find_package``-consumable
package; downstream consumers had no supported way to link the library
outside the in-tree ``add_subdirectory`` build.

Added
~~~~~
- ``regulon-c/cmake/regulonConfig.cmake.in``,
  ``regulon-c/cmake/regulon.pc.in``: templates for the generated CMake
  package config and pkg-config file.
- Installed CMake package (``lib/cmake/regulon/``):
  ``regulonConfig.cmake``, ``regulonConfigVersion.cmake``
  (``SameMajorVersion`` compatibility), and ``regulonTargets.cmake``
  exporting the ``regulon::regulon`` imported target. Consumers use
  ``find_package(regulon REQUIRED)`` +
  ``target_link_libraries(t PRIVATE regulon::regulon)``.
- Installed pkg-config file (``lib/pkgconfig/regulon.pc``) for non-CMake
  consumers (``pkg-config --cflags --libs regulon``).
- ``regulon-c/scripts/package_smoke/``: a standalone consumer-project
  fixture (own ``CMakeLists.txt`` + ``main.c``) that links against an
  installed package via ``find_package``, used by the new
  ``package-install-smoke`` CI job and available for local package
  testing.
- ``.github/workflows/ci_c.yml``: new ``package-install-smoke`` job
  builds and installs the library to a throwaway prefix, then configures/
  builds/runs the consumer fixture against it via both
  ``CMAKE_PREFIX_PATH`` and ``pkg-config``.

Changed
~~~~~~~
- ``regulon-c/CMakeLists.txt``: added a ``regulon::regulon`` ALIAS target
  (so in-tree ``add_subdirectory`` consumers and installed
  ``find_package`` consumers use the same target name); moved
  ``include(GNUInstallDirs)`` to the top of the file (it must run before
  ``CMAKE_INSTALL_INCLUDEDIR`` is referenced by
  ``target_include_directories()`` — it was previously only included
  further down in the Install section, which silently produced an empty
  ``$<INSTALL_INTERFACE:...>`` and no ``INTERFACE_INCLUDE_DIRECTORIES``
  on the exported target); switched the install destinations to the
  standard ``GNUInstallDirs`` variables instead of hardcoded ``lib``/
  ``include``.
- ``.gitignore``: added ``regulon-c/scripts/package_smoke/build/``.

Verification evidence
~~~~~~~~~~~~~~~~~~~~~~
- Local install to a throwaway prefix + a standalone consumer project
  configured with ``-DCMAKE_PREFIX_PATH=<prefix>``: ``find_package(regulon)``
  resolves, ``regulon::regulon`` links, and the consumer program runs and
  calls ``ron_pid_init`` successfully.
- The same consumer source compiled directly with
  ``$(pkg-config --cflags --libs regulon)`` (``PKG_CONFIG_PATH`` pointed at
  the installed prefix): builds and runs successfully.
- Full in-tree host build/test suite (19/19) unaffected by the
  ``CMakeLists.txt`` reordering.

------------------------------------------------------------------------

0.1.0 — Host Timing Benchmark (RON-PR-001, RON-PR-003)
------------------------------------------------------

Added
~~~~~
- ``regulon-c/bench/bench_step.c`` and ``regulon-c/bench/CMakeLists.txt``:
  a new host-only, ``RON_BUILD_BENCHMARKS``-gated timing benchmark
  reporting average/worst-observed per-call wall-clock time for the
  PID, Kalman, state-space, LQR, and LQG step functions against the
  RON-PR-003 design target (>= 10 kHz sample rate, i.e. a 100
  microsecond/step budget). This is informational evidence, not a
  certified WCET analysis — RON-PR-001/RON-PR-003 explicitly defer the
  certified budget to target-specific static or measurement-based
  timing analysis during integration — but it turns the previously
  doc-only performance claims into an actual, repeatable measurement
  and catches gross regressions (e.g. an accidental unbounded loop).
- ``.github/workflows/ci_c.yml``: new ``benchmark`` job builds and runs
  it on every push/PR (fails only on a >100 ms/step gross regression,
  so ordinary CI-runner jitter around the 10 kHz target doesn't break
  the build).
- ``regulon-c/cmake/ron_options.cmake``: new ``RON_BUILD_BENCHMARKS``
  option (default ``OFF``, host-only, mirrors ``RON_BUILD_EXAMPLES``).

Verification evidence
~~~~~~~~~~~~~~~~~~~~~~
- GCC and Clang default-flag builds: both build and run cleanly.
- Representative host measurements (informational, not a target
  budget): PID ~120 ns/step, state-space ~150 ns/step, LQR ~280
  ns/step, LQG ~3.0 us/step, Kalman ~6.3 us/step — all comfortably
  within the 100 us/step design-target budget on this host.

------------------------------------------------------------------------

0.1.0 — CI: RISC-V Cross-Compile Smoke Build
--------------------------------------------

Added
~~~~~
- ``.github/workflows/ci_c.yml``: new ``cross-riscv`` job cross-compiling
  the full library for ``rv32imc``/``ilp32`` with the
  ``gcc-riscv64-unknown-elf`` multilib toolchain (the Ubuntu/Debian
  package installs a ``riscv64-unknown-elf-gcc`` binary that targets
  rv32 or rv64 depending on ``-march``/``-mabi``, not a separate
  ``riscv32-unknown-elf-gcc``).

Changed
~~~~~~~
- ``regulon-c/cmake/toolchains/riscv32-unknown-elf.cmake``: corrected the
  compiler/binutils binary names to the ``riscv64-unknown-elf-*`` triple
  actually installed by the apt package, and added the same
  Newlib/picolibc-detection-with-declaration-only-fallback pattern already
  used by ``arm-none-eabi.cmake`` (``RON_RISCV_GCC_LIBC_INCLUDE`` /
  ``RON_RISCV_GCC_ALLOW_HEADER_SHIM``). Previously this toolchain file was
  present in the repository but never exercised by CI or verified to
  actually work.
- ``regulon-c/cmake/freestanding/riscv32-unknown-elf/include/math.h``: new
  declaration-only fallback header (mirrors the existing ARMv7 one), used
  only when no real libc headers are configured.

Verification evidence
~~~~~~~~~~~~~~~~~~~~~~
- ``riscv64-unknown-elf-gcc`` 13.2.0 with real ``picolibc`` headers
  (``/usr/lib/picolibc/riscv64-unknown-elf/include``): the full library,
  including ``ron_lqr.c``/``ron_lqg.c``, cross-compiles cleanly for
  ``-march=rv32imc -mabi=ilp32``.
- Declaration-only header-shim fallback path (``RON_RISCV_GCC_ALLOW_HEADER_SHIM``,
  no libc installed): also builds cleanly, matching the ARM toolchain's
  existing fallback behaviour.

------------------------------------------------------------------------

0.1.0 — C11 Phase 12: LQR and LQG Implementation
------------------------------------------------

Implements the LQR and LQG modules specified in the prior phase, closing
the last open module in the C11 roadmap.

Added
~~~~~
- ``regulon-c/src/ron_lqr.c``: LQR implementation — the shared discrete
  algebraic Riccati equation (DARE) solver (``ron_lqr_dare_solve``,
  iterative value recursion per SADS DD-19), pre-computed and DARE gain
  modes, three-source state-estimate dispatch (external / Luenberger /
  Kalman), optional per-input integral augmentation, and per-input
  saturation and rate limiting.

- ``regulon-c/src/ron_lqg.c``: LQG implementation — dual-DARE
  initialisation (LQR gain via the shared ``ron_lqr_dare_solve``; Kalman
  gain via ``ron_kf_init`` from the noise covariances, independently per
  the separation principle), predict/update delegation to the embedded
  ``ron_kf_t``, and the combined control step with the same per-input
  output-limiting semantics as LQR.

- ``regulon-c/src/ron_lqr_internal.h``: private header sharing
  ``ron_lqr_dare_solve`` between ``ron_lqr.c`` and ``ron_lqg.c``.

- ``regulon-c/test/unit/test_ron_lqr.c``: Unity suite ``RON-TC-LQR-001``
  – ``RON-TC-LQR-009`` plus defensive/validation coverage, all at 100%
  statement and branch coverage on ``ron_lqr.c``.

- ``regulon-c/test/unit/test_ron_lqg.c``: Unity suite ``RON-TC-LQG-001``
  – ``RON-TC-LQG-009`` plus defensive/validation coverage, all at 100%
  statement and branch coverage on ``ron_lqg.c``.

- ``regulon-c/test/formal/lqr_saturation_proof.c``: CBMC harness
  (``RON-TC-LQR-010-FV``) proving output saturation bounds and no heap
  allocation for a bounded finite ``ron_lqr_step``. CBMC verification:
  SUCCESSFUL.

- ``regulon-c/test/formal/lqg_no_heap_proof.c``: CBMC harness
  (``RON-TC-LQG-010-FV``) proving output saturation bounds, no heap
  allocation, and a finite Kalman state estimate for a bounded finite
  predict/update/step cycle. CBMC verification: SUCCESSFUL.

Changed
~~~~~~~
- ``regulon-c/cmake/ron_options.cmake``: ``RON_ENABLE_LQR`` and
  ``RON_ENABLE_LQG`` now default ``ON``, matching every other implemented
  module.

- ``regulon-c/test/CMakeLists.txt``: registers ``test_ron_lqr`` and
  ``test_ron_lqg`` (guarded by their respective ``RON_ENABLE_*`` options).

- ``regulon-c/scripts/lib_sources.txt`` / ``scripts/format_files.txt``:
  add ``ron_lqr.c``, ``ron_lqg.c``, and ``ron_lqr_internal.h`` to the CI
  format / cppcheck / MISRA / complexity / coverage / CBMC source
  manifests.

Design notes
~~~~~~~~~~~~
- ``ron_lqg``'s configuration validation deliberately checks only ``A``
  and ``B`` up front (consumed directly by the shared DARE solver); the
  Kalman-only fields (``H``, ``Q_noise``, ``R_noise``, ``x0``, ``P0``,
  ``K_f_inf``) are validated by the embedded ``ron_kf_init`` call itself,
  mirroring the delegation pattern already used by
  ``ron_statespace``/``ron_observer`` for their embedded estimators
  instead of duplicating the checks.

Verification evidence
~~~~~~~~~~~~~~~~~~~~~~
- ``ctest`` (GCC): 19/19 suites pass, including the new ``test_ron_lqr``
  and ``test_ron_lqg``.
- Double-precision (``RON_USE_DOUBLE=ON``), standalone Clang, and GCC
  ``-fsanitize=address,undefined -fno-sanitize-recover=all``: all 19
  suites pass on each configuration.
- ``clang-format --dry-run --Werror`` over the format manifest: clean.
- ``cppcheck --addon=misra.py`` over the active library source set: no
  findings in ``ron_lqr.c`` / ``ron_lqg.c`` (pre-existing findings in
  ``ron_autotune.c``/``.h`` are unrelated to this phase).
- ``lizard -C 10``: no function exceeds CCN 10.
- GCC ``--coverage`` (gcov -b): 100% line and 100% branch coverage (both
  directions taken) on both ``ron_lqr.c`` and ``ron_lqg.c``.
- CBMC (``--unwind 65 --unwinding-assertions --bounds-check
  --pointer-check``): both new harnesses report VERIFICATION SUCCESSFUL.
- ARM Cortex-M cross-compile (``arm-none-eabi-gcc`` with real Newlib
  headers): full library including ``ron_lqr.c``/``ron_lqg.c`` builds
  cleanly.
- ``bash regulon-c/scripts/check_manifest.sh``: manifests in sync.

------------------------------------------------------------------------

0.1.0 — Spec Phase: LQR and LQG Controller Support
--------------------------------------------------

Specification-layer additions for the Linear Quadratic Regulator (LQR) and
Linear Quadratic Gaussian (LQG) controller modules.  No C implementation
files are added in this phase; those follow in the next implementation phase.

Added
~~~~~
- ``docs/specs/SRS_ControlLib.rst`` v1.2.0: new requirements
  ``RON-FR-730`` – ``RON-FR-739`` (LQR) and ``RON-FR-750`` – ``RON-FR-759``
  (LQG).  Updated traceability matrix in Appendix C.

- ``docs/specs/SADS_ControlLib.rst`` v1.2.0: new module design sections
  for ``ron_lqr`` (DARE solver pseudocode, control-law pseudocode, data
  structures) and ``ron_lqg`` (separation-principle design, init/predict/
  update/step pseudocode).  Updated module dependency diagram.  Added
  design decisions DD-19 (iterative DARE over Schur decomposition) and
  DD-20 (LQG forces Kalman-only estimator).

- ``docs/specs/IS_ControlLib.rst`` v1.2.0: compile-time constants
  ``RON_LQR_MAX_STATES`` and ``RON_LQR_MAX_INPUTS``; complete C API
  specifications for ``ron_lqr.h`` and ``ron_lqg.h``; CMake options
  ``RON_ENABLE_LQR`` and ``RON_ENABLE_LQG`` (default OFF until
  implementation phase); traceability rows for FR-730–739 and FR-750–759.

- ``docs/specs/TP_ControlLib.rst`` v1.1.0: test catalog
  ``RON-TC-LQR-001`` – ``RON-TC-LQR-010`` and ``RON-TC-LQG-001`` –
  ``RON-TC-LQG-010`` (including formal harness entries
  ``RON-TC-LQR-010-FV`` / ``RON-TC-LQG-010-FV``).  Updated requirement
  coverage table and test execution order.  Added open items OI-TP-07–09.

- ``regulon-c/include/ron/ron_lqr.h``: public C header (types, enums,
  ``ron_lqr_config_t``, ``ron_lqr_state_t``, ``ron_lqr_t``, full API with
  traceability annotations).

- ``regulon-c/include/ron/ron_lqg.h``: public C header (types, enums,
  ``ron_lqg_config_t``, ``ron_lqg_t``, full API with traceability
  annotations).

- ``regulon-c/include/ron/ron.h``: ``RON_HAVE_LQR`` / ``RON_HAVE_LQG``
  conditional includes.

- ``regulon-c/cmake/ron_options.cmake``: ``RON_ENABLE_LQR`` and
  ``RON_ENABLE_LQG`` options (default OFF).

- ``regulon-c/cmake/ron_modules.h.in``: ``RON_HAVE_LQR`` and
  ``RON_HAVE_LQG`` template entries.

- ``regulon-c/CMakeLists.txt``: LQR/LQG dependency-resolution rules
  (LQG→LQR→STATESPACE→KALMAN), module-variable loop, and conditional
  ``src/ron_lqr.c`` / ``src/ron_lqg.c`` source-list entries.

------------------------------------------------------------------------

0.1.0 — C11 Phase 11 Full-Library Integration And Release Hardening
-------------------------------------------------------------------

This entry completes the C11 implementation: all eleven roadmap phases
(PID, filters, feed-forward, gain scheduling, trajectory, cascade, Kalman,
state-space/observer, auto-tune, health, metrics) are now integrated,
release-hardened, and traceable end to end.

Added
~~~~~
- ``regulon-c/include/ron/ron.h``: aggregate convenience header that
  transitively includes every public module header in dependency order,
  guarded by ``RON_HAVE_<MODULE>`` macros from the generated
  ``ron/ron_modules.h``.

- Per-module CMake selection (``RON_ENABLE_FILTER`` … ``RON_ENABLE_METRICS``,
  default ON) generating ``ron/ron_modules.h`` via ``cmake/ron_modules.h.in``.
  The PID core and integrated feed-forward path are the mandatory baseline;
  ``RON_ENABLE_STATESPACE`` forces ``RON_ENABLE_KALMAN``.

- ``regulon-c/test/integration/test_ron_integration.c``: full-library
  integration suite ``RON-TC-INT-001`` … ``RON-TC-INT-005`` (aggregate-header
  include-topology check, trajectory→cascade→health→metrics loop with a
  determinism check, estimator-in-the-loop, auto-tune deploy, and
  multi-instance isolation).

- Host example programs behind ``RON_BUILD_EXAMPLES``:
  ``regulon-c/examples/pid_quickstart.c`` and
  ``regulon-c/examples/cascade_control_loop.c``.

- Centralized CI source manifests ``regulon-c/scripts/lib_sources.txt`` and
  ``regulon-c/scripts/format_files.txt`` with a drift guard
  ``regulon-c/scripts/check_manifest.sh``; new CI jobs ``manifest-check``,
  ``build-subset`` (PID-only smoke build), and ``build-examples``
  (GCC + Clang under ``-Werror``).

- ``docs/plans/c/c11-phase-11-integration.md``: Phase 11 living plan / closure
  record. CBMC harness inventory added to ``docs/specs/TP_ControlLib.rst``.

Changed
~~~~~~~
- ``.github/workflows/ci_c.yml`` and ``regulon-c/scripts/verify_pid.ps1`` now
  read the shared manifests instead of five parallel hard-coded source lists,
  removing the per-file maintenance hazard.

- ``docs/specs/IS_ControlLib.rst`` and ``docs/specs/SADS_ControlLib.rst``
  document the aggregate header, the generated ``ron_modules.h``, and the
  per-module build options. ``TP_ControlLib.rst`` adds the ``INT`` module and
  both traceability matrices for the integration suite.

Fixed
~~~~~
- Closed CI gate gaps surfaced by the manifest drift check:
  ``ron_matrix_internal.h`` was missing from the clang-format list, and
  ``verify_pid.ps1`` omitted ``ron_matrix.c`` / ``ron_metrics.c`` /
  ``ron_observer.c`` / ``ron_statespace.c`` from its CBMC source set.

------------------------------------------------------------------------

0.1.0 — Rust-First PID Kickoff
------------------------------

Added
~~~~~
- ``regulon-rs/Cargo.toml`` and ``regulon-rs/.cargo/config.toml``: Rust
  workspace scaffold and cross-target build configuration for the new
  Rust-first implementation track.

- ``regulon-rs/regulon/Cargo.toml`` and ``regulon-rs/regulon/src/``:
  ``#![no_std]`` Rust crate with ``platform``, ``error``, and ``pid``
  modules implementing the project kickoff PID baseline.

- Rust PID public API centered on ``Pid`` and ``PidConfig`` with runtime
  state encapsulation, configuration validation, manual/automatic mode
  switching, normalization, saturation, rate limiting, anti-windup, fault
  latching, and state snapshots.

- Traceable Rust unit tests and Kani proof scaffolding for the PID and
  safety baseline using ``RON-TC-PID-*`` and ``RON-TC-SAFE-*`` identifiers.

- ``docs/plans/rust/rust-first-rollout.md``: rollout plan/status record for
  the Rust-first implementation, iteration sequence, and current completion
  state.

- ``regulon-rs/regulon/src/filter/``: first reusable filter slice with a
  first-order low-pass filter and standalone asymmetric rate limiter plus
  traceable Rust tests.

- Static-gain feed-forward support in the Rust PID module, including a
  bounded feed-forward configuration surface, feed-forward diagnostics in
  PID state/status, and traceable Rust tests for ``RON-TC-FF-002`` and
  ``RON-TC-FF-008``.

------------------------------------------------------------------------

0.1.0 C11 PID Vertical Slice
----------------------------

Added
~~~~~
- ``regulon-c/include/ron/ron_metrics.h`` /
  ``regulon-c/src/ron_metrics.c``: added the complete C11 Phase 10 runtime
  performance metrics slice (replacing the prior stub).  The passive
  accumulator attaches to any controller and quantifies closed-loop quality
  each step from ``(r, y, dt)`` (RON-FR-950), computing the error integrals
  IAE, ISE, ITAE and the step-response transients peak overshoot, rise time,
  and settling time (RON-FR-951).  It supports cumulative and windowed
  accumulation (RON-FR-952), is enable/disable at runtime and disabled by
  default with a zero-overhead disabled path (RON-FR-953), and auto-restarts
  the transient metrics on a detected setpoint step (RON-FR-954).  The
  authoritative metric set is the SRS/IS/SADS list (no RMSE / steady-state
  error).  Verified by ``RON-TC-MET-001`` through ``RON-TC-MET-007`` Unity
  tests and the ``metrics_no_heap_proof.c`` CBMC no-heap / monotone-integral
  harness (``RON-TC-MET-001-FV``), with 100% line and branch coverage on
  ``ron_metrics.c``.

- ``regulon-c/include/ron/ron_health.h`` /
  ``regulon-c/src/ron_health.c``: added the complete C11 Phase 9 control-loop
  health monitor slice (replacing the prior stub).  The passive observer
  attaches to any controller and evaluates loop health each step from
  ``(r, y, u, dt)`` (RON-FR-900), reporting output-stuck, diverging,
  oscillating, sensor-dropout, and setpoint-unreachable conditions through a
  latched bitmask (RON-FR-901) with independently configurable per-condition
  thresholds (RON-FR-902).  It never modifies the controller (RON-FR-903,
  SADS DD-16), fires an optional callback on each condition's first activation
  (RON-FR-904), and latches every condition until ``ron_health_clear()``
  (RON-FR-905).  Output-stuck is detected as an unchanged output for
  ``t_sat_max`` (the only reading consistent with the IS configuration, which
  exposes no saturation limits); two opaque state fields (``u_prev``,
  ``prev_valid``) extend the IS-enumerated set for that comparator and the
  first-step guard.  Verified by ``RON-TC-HLTH-001`` through
  ``RON-TC-HLTH-010`` Unity tests and the ``health_no_heap_proof.c`` CBMC
  no-heap / monotonic-latch harness, with 100% line and branch coverage on
  ``ron_health.c``.

- ``regulon-c/include/ron/ron_autotune.h`` /
  ``regulon-c/src/ron_autotune.c``: added the complete C11 Phase 8 relay-
  feedback PID auto-tuner slice (replacing the prior stub).  The module
  excites a plant with a hysteresis relay in place of the PID output
  (RON-FR-800) with configurable amplitude / hysteresis / minimum cycles /
  timeout (RON-FR-801), estimates ``Ku = 4d/(pi*A)`` and ``Tu`` by zero-
  crossing counting (RON-FR-802), supports the Ziegler-Nichols, Tyreus-
  Luyben, some-overshoot, and no-overshoot tuning rules (RON-FR-803), commits
  the tuned gains to the target PID only on an explicit
  ``ron_autotune_apply`` (RON-FR-804), exposes raw ``Ku`` / ``Tu``
  (RON-FR-805), keeps the relay output within ``[u_bias - d, u_bias + d]``
  (RON-FR-806), and restores the PID on caller abort or timeout (RON-FR-807).
  The module is standalone scalar math (no ``ron_matrix`` dependency) and
  touches the PID only through the existing atomic gain / mode APIs.

- ``regulon-c/test/unit/test_ron_autotune.c``: added traceable Unity tests
  ``RON-TC-AT-001`` through ``RON-TC-AT-008`` covering closed-loop relay
  excitation of a first-order plant, configuration validation, ``Ku`` / ``Tu``
  estimation within 10% of known reference values, all four tuning rules,
  apply-only gain commit, raw ``Ku`` / ``Tu`` exposure, relay-output bounds
  with step guards, and abort / timeout restore.  ``ron_autotune.c`` holds
  100% line and branch coverage.

- ``regulon-c/test/formal/autotune_relay_bound_proof.c``: added the
  ``RON-TC-AT-007-FV`` CBMC harness proving the relay output stays within
  ``[u_bias - d, u_bias + d]`` and that the auto-tuner path performs no heap
  allocation.

- ``docs/plans/c/c11-phase-8-autotune.md``: added and closed the living
  Phase 8 implementation plan with verification evidence and design choices.

- ``regulon-c/include/ron/ron_statespace.h`` /
  ``regulon-c/src/ron_statespace.c`` and
  ``regulon-c/include/ron/ron_observer.h`` /
  ``regulon-c/src/ron_observer.c``: added the complete C11 Phase 7
  state-space controller and Luenberger observer slice.  The
  state-feedback controller computes ``u = -K x_hat + Kr r`` (RON-FR-700)
  with external-vector, embedded-Luenberger, and embedded-Kalman estimate
  sources (RON-FR-701), optional integral augmentation on the regulated
  output (RON-FR-702), PID-equivalent output saturation / rate limiting and
  fault detection (RON-FR-703), and runtime-updatable ``K`` / ``Kr``
  (RON-FR-704).  The observer implements
  ``x_hat(k+1) = A x_hat + B u + L (y - C x_hat)`` (RON-FR-720) with
  caller-supplied ``A``, ``B``, ``C``, ``L`` (RON-FR-721), a full-state
  getter (RON-FR-722), and compile-time-bounded storage (RON-FR-723).

- ``regulon-c/src/ron_matrix.c`` and
  ``regulon-c/src/ron_matrix_internal.h``: added an internal, non-public
  shared bounded matrix / vector helper (uniform ``RON_MAT_MAX_DIM``
  stride) factored out of ``ron_kalman.c`` and now used by the Kalman,
  state-space, and observer modules.  ``ron_kalman.c`` was refactored onto
  it as a pure extraction with unchanged public API, numerics, and tests.

- ``regulon-c/test/unit/test_ron_observer.c`` and
  ``regulon-c/test/unit/test_ron_statespace.c``: added traceable Unity
  tests ``RON-TC-SS-001`` through ``RON-TC-SS-009`` covering reference
  state feedback, all three estimate sources with embedded-estimator
  advance and cross-source rejection, integral accumulation / clamp /
  reset, saturation and bidirectional rate limiting matching the PID
  pipeline, runtime gain update, the Luenberger step / convergence /
  parameterisation, the observer state getter, and the full
  configuration-validation, defensive, maximum-dimension, and
  numeric-overflow surfaces.  All four affected sources retain 100%
  line and branch coverage.

- ``regulon-c/test/formal/statespace_sat_proof.c``: added the
  ``RON-TC-SS-004-FV`` CBMC harness proving the state-space output stays
  within ``[u_min, u_max]`` and that the state-space path performs no heap
  allocation.

- ``docs/specs/IS_ControlLib.rst``: added the ``ron_observer.h`` and
  ``ron_statespace.h`` API blocks.  ``docs/specs/TP_ControlLib.rst``: added
  detailed descriptions for ``RON-TC-SS-001`` through ``RON-TC-SS-009`` and
  ``RON-TC-SS-004-FV``.  ``regulon-c/include/ron/ron_platform.h``: added the
  ``RON_SS_MAX_*`` minimum-bound static asserts and the ``RON_MAT_MAX_DIM``
  definition with its coverage asserts.

- ``docs/plans/c/c11-phase-7-statespace-observer.md``: added and closed the
  living Phase 7 implementation plan with verification evidence and design
  choices.

- ``regulon-c/include/ron/ron_kalman.h`` and
  ``regulon-c/src/ron_kalman.c``: added the complete C11 Phase 6 discrete
  linear Kalman filter slice with caller-owned fixed-maximum matrix/vector
  storage (RON-FR-601, RON-FR-607); predict and update cycle (RON-FR-602);
  scalar division for ``m == 1`` and an in-place Cholesky-factor / solve
  for ``m > 1`` innovation inversion (RON-FR-603); optional Joseph-form
  covariance update (RON-FR-604); measurement-dropout no-op update
  (RON-FR-605); optional steady-state fixed-gain mode using ``K_inf``
  (RON-FR-606); and bounded-iteration internal ``sqrt`` so the production
  source remains free of ``<math.h>``.

- ``regulon-c/test/unit/test_ron_kalman.c``: added 8 traceable Unity tests
  (``RON-TC-KF-001`` through ``RON-TC-KF-008``) covering scalar
  convergence, parameter/dimension validation, hand-checked predict /
  update reference cases, the diagonal and non-diagonal Cholesky paths,
  the non-positive-definite ``S`` rejection path, the ``m == 1``
  degenerate-``S`` guard, Joseph-form parity with the standard update,
  measurement dropout, steady-state fixed-gain mode, maximum-dimension
  storage, and all defensive null / uninitialised / non-finite input
  paths, including the ``RON_FAULT_OUTPUT_NAN`` numeric-overflow detection
  in both ``ron_kf_predict`` and ``ron_kf_update``.

- ``regulon-c/test/formal/kalman_no_heap_proof.c``: added the
  ``RON-TC-KF-008-FV`` CBMC harness asserting that the Kalman lifecycle
  (init / predict / update / get_state / reset) does not call
  ``malloc``, ``calloc``, ``realloc``, or ``free``.

- ``docs/plans/c/c11-phase-6-kalman-filter.md``: added and closed the
  living Phase 6 implementation plan with final verification evidence,
  residual tool gaps, and design choices (uniform stride scratch
  matrices, non-``const`` ``kf_mat_t`` parameters to avoid the C11
  pedantic restriction on ``T(*)[N] → const T(*)[N]`` conversions,
  conservative reuse of ``RON_FAULT_CONFIG_INVALID`` for non-positive-
  definite innovation covariances, and reuse of ``RON_FAULT_OUTPUT_NAN``
  for non-finite predict/update outputs).

- ``docs/specs/TP_ControlLib.rst``: filled in detailed test descriptions
  for ``RON-TC-KF-002`` through ``RON-TC-KF-005``, ``RON-TC-KF-007``,
  ``RON-TC-KF-008``, and ``RON-TC-KF-008-FV``.

- ``regulon-c/include/ron/ron_cascade.h`` and
  ``regulon-c/src/ron_cascade.c``: added the complete C11 Phase 5 cascade
  (master/slave) PID controller slice.  The outer loop's clamped output
  automatically drives the inner setpoint (RON-FR-401, RON-FR-402);
  cross-loop back-calculation anti-windup propagation restrains the outer
  integral when the inner loop saturates (RON-FR-403); coordinated
  MANUAL↔AUTO mode transitions with bumpless transfer preserve bumpless
  hand-off ordering (RON-FR-404); fault-clear and reset operate on both
  loops atomically (RON-FR-405); and a unified 32-bit status word encodes
  both loop statuses with extraction macros (RON-FR-406).

- ``regulon-c/test/unit/test_ron_cascade.c``: added 12 traceable Unity
  tests (RON-TC-CASC-001 through RON-TC-CASC-012) achieving 100% line and
  branch coverage on ``ron_cascade.c``.

- ``regulon-c/include/ron/ron_trajectory.h``,
  ``regulon-c/src/ron_trajectory_trap.c``, and
  ``regulon-c/src/ron_trajectory_scurve.c``: added the complete C11 Phase 4
  trajectory-generator slice with standalone trapezoidal and bounded
  seven-phase jerk-limited S-curve profiles.

- ``regulon-c/test/unit/test_ron_trajectory.c``: added traceable Unity tests
  for ``RON-TC-TRAJ-001`` through ``RON-TC-TRAJ-008`` covering convergence,
  short moves, reverse moves, retargeting, hold/resume, validation, completion,
  and bounded kinematic outputs.

- ``docs/plans/c/c11-phase-4-trajectory-generators.md``: added and closed the
  living Phase 4 implementation plan with final verification evidence,
  residual tool gaps, and design choices.

- ``regulon-c/include/ron/ron_gain_sched.h`` and
  ``regulon-c/src/ron_gain_sched.c``: added the complete C11 Phase 3 gain
  scheduling slice with bounded table validation, hard-switch scheduling,
  linear interpolation, and optional integral reset on successful hard
  switches.

- ``regulon-c/test/unit/test_ron_gain_sched.c``: added traceable Unity tests
  for ``RON-TC-GS-001`` through ``RON-TC-GS-008``.

- ``regulon-c/include/ron/ron_feedforward.h`` and
  ``regulon-c/src/ron_feedforward.c``: added the complete C11 Phase 2
  feed-forward PID extension with static, velocity, acceleration, and
  external feed-forward modes.

- ``regulon-c/test/unit/test_ron_feedforward.c``: added traceable Unity tests
  for ``RON-TC-FF-001`` through ``RON-TC-FF-009``.

- ``regulon-c/include/ron/ron_filter.h`` and ``regulon-c/src/ron_filter.c``:
  added the complete C11 signal-conditioning filter slice with LP1,
  moving-average FIR, cascaded biquad IIR, coefficient helpers, notch
  hot-swap, and asymmetric rate-limiter APIs.

- ``regulon-c/test/unit/test_ron_filter.c``: added traceable Unity tests for
  ``RON-TC-FILT-001`` through ``RON-TC-FILT-017``.

- ``regulon-c/test/formal/filter_*_proof.c``: added focused CBMC harnesses
  for filter null-pointer handling, bounded array access, bounded execution,
  and no-heap evidence.

- ``regulon-c/cmake/toolchains/armv7-none-eabi-clang.cmake``: added an
  LLVM/Clang bare-metal ARMv7 cross-compile toolchain and CI/local verification
  wiring for ``cross-arm-clang``, preferring Newlib target headers when
  installed.

- ``regulon-c/src/ron_pid_api.c``, ``regulon-c/src/ron_pid_core.c``,
  ``regulon-c/src/ron_pid_fault.c``, and ``regulon-c/src/ron_pid_internal.h``:
  implemented the active C11 PID slice behind the frozen public API.

- ``regulon-c/test/unit/test_ron_pid_core.c``,
  ``regulon-c/test/unit/test_ron_pid_api.c``, and
  ``regulon-c/test/unit/test_ron_pid_common.h``: added traceable PID and
  safety unit suites for the kickoff sprints.

- ``regulon-c/test/formal/*.c``: added the first PID-focused CBMC harness set
  for saturation, back-calculation, integral clamp, multi-instance
  independence, and null-pointer validation.

- ``docs/plans/c/c11-rollout.md``: C-track rollout/status note with local
  evidence and remaining verification gaps.

- ``regulon-c/scripts/verify_pid.ps1``: repo-owned Windows verification
  entrypoint for the active PID slice, including tool probing, MSVC builds,
  double-precision regression, formatting, cppcheck, and complexity checks.

- Additional C CBMC harnesses for PID verification closure, covering API
  validation, denominator guards, no-heap/no-recursion/no-blocking claims,
  array-bounds smoke coverage, NaN/Inf fault handling, integral overflow,
  bounded step execution, and no global-state dependency.

Changed
~~~~~~~
- ``regulon-c/include/ron/ron_pid.h`` and ``regulon-c/src/ron_pid_api.c``:
  added the public ``ron_pid_set_config()`` atomic runtime update API and
  routed the existing PID runtime setters through that full-config path.

- ``regulon-c/CMakeLists.txt`` and ``regulon-c/test/CMakeLists.txt``: enabled
  the Phase 1 filter, Phase 2 feed-forward, Phase 3 gain-scheduling, and
  Phase 4 trajectory source/unit suites in the active C11 build without
  enabling later placeholder modules.

- ``regulon-c/scripts/verify_pid.ps1`` and ``.github/workflows/ci_c.yml``:
  extended active-source verification from PID-only to PID plus filters and
  feed-forward plus gain scheduling plus trajectory generators for format,
  static analysis, complexity, coverage, cross-compile, and CBMC source lists.

- ``docs/specs/IS_ControlLib.rst`` and ``docs/specs/TP_ControlLib.rst``:
  reconciled the trajectory API/state model and completed detailed test-plan
  descriptions for ``RON-TC-TRAJ-003`` and ``RON-TC-TRAJ-005`` through
  ``RON-TC-TRAJ-008``.

- ``docs/specs/IS_ControlLib.rst`` and ``docs/specs/TP_ControlLib.rst``:
  reconciled the filter API with ``get_state`` operations, shared
  fault/status types, band-pass coefficient generation, and detailed
  ``RON-TC-FILT-*`` test scenarios.

- ``docs/plans/c/c11-rollout.md`` and ``docs/plans/c11-roadmap.md``:
  recorded Phase 0 PID closure acceptance, Phase 1 filter completion, and
  Phase 2 feed-forward completion; they now also record Phase 3 gain
  scheduling completion and Phase 4 trajectory generator completion.

- ``regulon-c/CMakeLists.txt``: keeps the active C build limited to accepted
  slices, now PID plus filters plus feed-forward plus gain scheduling plus
  trajectory generators, so later placeholder modules do not pollute host
  builds and quality evidence.

- ``regulon-c/test/CMakeLists.txt``: fixed the Windows/MSVC host-test build by
  making ``m`` linkage conditional and enabling the PID core/API suites.

- ``docs/specs/SRS_ControlLib.rst``, ``docs/specs/IS_ControlLib.rst``, and
  ``docs/specs/TP_ControlLib.rst``: reconciled the PID API summary and updated
  stale ``c/`` path references to ``regulon-c/`` where the C11 track is
  described concretely.

- ``regulon-c/src/ron_pid_config.c`` and ``regulon-c/src/ron_pid_core.c``:
  refactored the active PID validation and step pipeline to keep cyclomatic
  complexity within the ``RON-QR-011`` limit.

- ``regulon-c/test/unit/test_ron_pid_api.c``: tightened the PID verification
  slice with open-loop anti-windup recovery contrast, expanded null-pointer
  checks, fault-register coverage, safe-output clamping, and deterministic
  reproducibility testing.

- ``regulon-c/scripts/verify_pid.ps1``: added Clang/LLVM coverage
  enforcement, dynamic CBMC harness discovery, CBMC unwinding assertions,
  and cleaner skip reporting for unavailable Clang, coverage, cross, and
  formal tools.

- ``.github/workflows/ci_c.yml``: now enforces 100% statement and branch
  coverage with Clang/LLVM coverage, uploads raw/rendered coverage plus CBMC
  artifacts, and adds the ARM Cortex-M cross-compile smoke build for the
  active PID slice.

- ``docs/specs/TP_ControlLib.rst`` and ``docs/plans/c/c11-rollout.md``:
  updated the C verification-closure claims, evidence, and remaining local
  tool gaps.

- ``docs/deviations/MISRA_C_deviations.rst``: recorded approved deviations and
  observations for the active PID verification closure.

------------------------------------------------------------------------

0.1.0 — Sprint 1 (C11 Platform + Type Headers)
----------------------------------------------

Added
~~~~~
- ``regulon-c/include/ron/ron_platform.h``: Platform portability layer —
  ``ron_float_t``, ``RON_FLOAT_*`` macros, ``RON_STATIC_ASSERT``,
  ``RON_ASSERT``, ``RON_ISNAN``/``RON_ISINF``/``RON_ISFINITE``,
  ``ron_clamp()``, ``ron_fabs()``, dimension constants for all modules.
  Satisfies: RON-PR-010, RON-PR-011, RON-DC-001/002.

- ``regulon-c/include/ron/ron_pid_types.h``: All public enumerations
  (``ron_aw_mode_t``, ``ron_deriv_mode_t``, ``ron_op_mode_t``,
  ``ron_safe_policy_t``, ``ron_integ_method_t``), fault/status bitmask
  types, ``ron_pid_config_t``, ``ron_pid_state_t``, ``ron_pid_instance_t``.
  Compile-time size assertions included.
  Satisfies: RON-FR-001–071 (declarations), RON-SR-010–013, RON-PR-021.

- ``regulon-c/include/ron/ron_pid.h``: Complete public API declarations
  (lifecycle, runtime, configuration update, state inspection, fault management).
  Satisfies: RON-FR-050–071, RON-SR-001/002/012.

- ``regulon-c/test/unit/test_ron_pid_types.c``: Unity test suite covering
  RON-TC-PERF-005, RON-TC-PERF-006, RON-TC-QUAL-005, RON-TC-QUAL-007.

------------------------------------------------------------------------

[Sprint 0] 2026-04-11 — C11 Build Scaffold
-------------------------------------------

Added
~~~~~
- ``regulon-c/CMakeLists.txt``: Main CMake 3.21+ build file for the C11
  implementation track.  Builds the ``regulon`` static library.
  Satisfies: RON-QR-004, RON-DC-005.

- ``regulon-c/cmake/ron_options.cmake``: Build options
  ``RON_USE_DOUBLE``, ``RON_BUILD_TESTS``, ``RON_ENABLE_ASSERT``.

- ``regulon-c/cmake/toolchains/arm-none-eabi.cmake``: ARM Cortex-M
  cross-compile toolchain file.

- ``regulon-c/cmake/toolchains/riscv32-unknown-elf.cmake``: RISC-V 32-bit
  cross-compile toolchain file.

- ``regulon-c/cmake/toolchains/host-x86_64.cmake``: Host x86-64 toolchain
  reference file.

- ``regulon-c/test/CMakeLists.txt``: Test sub-directory build.  Registers
  Unity-based test executables with CTest.

- ``regulon-c/test/framework/unity/``: Vendored Unity v2.6.0 (MIT) — the
  lightweight C unit test framework used for all ``UT`` and ``IT`` test
  levels.

- ``.clang-format``: Project-wide code formatting configuration.
  Satisfies: RON-SR-030.

- ``.github/workflows/ci_c.yml``: GitHub Actions CI pipeline — build,
  test, static analysis (cppcheck), coverage (lcov), and artefact upload.
  Satisfies: RON-QR-004, RON-TC-QUAL-008.

- ``docs/deviations/MISRA_C_deviations.rst``: MISRA C:2023 deviation
  records (initially empty — no deviations at baseline).

- ``CHANGELOG.rst``: This file.

------------------------------------------------------------------------

Specification Baseline — 2026-04-11
-------------------------------------

Added
~~~~~
- ``docs/specs/SRS_ControlLib.rst`` v1.1.0 — Software Requirements
  Specification (RON-SRS-001).
- ``docs/specs/SADS_ControlLib.rst`` v1.1.0 — Software Architecture and
  Design Specification (RON-SADS-001).
- ``docs/specs/IS_ControlLib.rst`` v1.1.0 — Implementation Specification
  (RON-IS-001).
- ``docs/specs/TP_ControlLib.rst`` v1.0.0 — Test Plan and Specification
  (RON-TP-001).
- ``AGENTS.md`` — Developer guidelines and coding standards.
- ``LICENSE`` — MIT License.
