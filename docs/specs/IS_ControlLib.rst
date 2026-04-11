.. ============================================================
.. Implementation Specification
.. Regulon — PID Controller Module
.. ============================================================

.. meta::
   :description: Implementation Specification for the Regulon, PID Controller Module.
   :keywords: PID, control systems, embedded, C, implementation, MISRA, API

########################################################################
Implementation Specification
########################################################################

**Document Title:** Implementation Specification — Regulon, PID Controller Module

**Document ID:** RON-IS-001

**Version:** 1.0.0

**Status:** Draft

**Date:** 2025-04-10

.. contents:: Table of Contents
   :depth: 3
   :local:
   :backlinks: none

.. sectnum::

------------------------------------------------------------------------

Revision History
================

.. list-table::
   :header-rows: 1
   :widths: 10 10 60 20

   * - Version
     - Date
     - Description
     - Author
   * - 0.1
     - 2025-03-20
     - Initial draft
     - TBD
   * - 1.0.0
     - 2025-04-10
     - First baseline release
     - TBD
   * - 1.1.0
     - 2025-04-10
     - Added: new module headers (ron_filter.h, ron_feedforward.h,
       ron_gain_sched.h, ron_cascade.h, ron_trajectory.h, ron_kalman.h,
       ron_statespace.h, ron_observer.h, ron_autotune.h, ron_health.h,
       ron_metrics.h). Updated directory layout, CMakeLists.txt, compile-time
       constants, and traceability table.
     - TBD

------------------------------------------------------------------------

Introduction
============

Purpose
-------

This document is the **Implementation Specification (IS)** for the Regulon Control Systems Library PID Controller Module. It refines the language-agnostic architecture established in RON-SADS-001 into concrete, binding implementation decisions:

- The programming language and dialect selection.
- The applicable coding standard.
- The complete C-language public API (header-level declarations).
- The internal file and directory structure.
- The build system specification.
- Platform abstraction and portability conventions.
- Integration guidelines for embedded targets.

This document, together with RON-SRS-001 and RON-SADS-001, forms the complete specification baseline. Developers **shall** implement the library in strict conformance with all three documents.

Scope
-----

This document covers the Regulon PID module only. It does not cover test code, documentation tooling, or future library modules, although conventions established here shall be followed by all future modules.

Parent Documents
----------------

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - Document ID
     - Title
   * - RON-SRS-001 v1.0.0
     - Software Requirements Specification — PID Controller Module
   * - RON-SADS-001 v1.0.0
     - Software Architecture and Design Specification — PID Controller Module
   * - RON-TP-001
     - Test Plan (companion document, produced separately)

References
----------

.. list-table::
   :header-rows: 1
   :widths: 10 90

   * - ID
     - Reference
   * - [REF-01]
     - ISO/IEC 9899:2011 (C11), *Programming Languages — C*. ISO.
   * - [REF-02]
     - ISO/IEC 9899:1999 (C99), *Programming Languages — C*. ISO.
   * - [REF-03]
     - MISRA C:2023, *Guidelines for the Use of the C Language in Critical Systems*. MISRA.
   * - [REF-04]
     - MISRA Rust:2024, *Guidelines for the Use of the Rust Language in Safety-Related Systems*. MISRA.
   * - [REF-05]
     - IEC 61508:2010, *Functional Safety of E/E/PE Safety-related Systems*. IEC.
   * - [REF-06]
     - IEEE 754:2019, *Standard for Floating-Point Arithmetic*. IEEE.
   * - [REF-07]
     - CMake documentation, version 3.21+. https://cmake.org/documentation/
   * - [REF-08]
     - The Rust Reference, Edition 2021. https://doc.rust-lang.org/reference/
   * - [REF-09]
     - Ferrocene Language Specification. https://spec.ferrocene.dev/
   * - [REF-10]
     - Kani Rust Verifier documentation. https://model-checking.github.io/kani/
   * - [REF-11]
     - CERT C Coding Standard (2016 edition). SEI/Carnegie Mellon.
   * - [REF-12]
     - *The Embedded Rust Book*. https://docs.rust-embedded.org/book/

------------------------------------------------------------------------

Language Selection
==================

Overview
--------

Regulon targets **two first-class implementations** of equal standing: one in
**C11** and one in **Rust (Edition 2021)**. Both implementations shall be
algorithmically identical, satisfy all requirements in RON-SRS-001, and follow
the architecture defined in RON-SADS-001. This section documents the
comparative evaluation that led to that decision, and the specific rules each
implementation track must follow.

The two implementations are maintained as sibling crates/libraries within the
same repository under ``c/`` and ``rust/`` subdirectories respectively. They
share the same documentation, test specifications, and requirement traceability.

Comparative Evaluation
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 28 36 36

   * - Factor
     - C11
     - Rust (Edition 2021)
   * - **Toolchain availability**
     - GCC and Clang support every MCU family (ARM, RISC-V, AVR, PIC, MSP430,
       MIPS, etc.) with decades of embedded maturity.
     - ``rustc`` (via LLVM) supports ARM Cortex-M, RISC-V, x86; growing MCU
       coverage. Some niche families (AVR, PIC) have limited or unstable
       support.
   * - **Safety-critical standards**
     - MISRA C:2023 is the de-facto standard, referenced by IEC 61508,
       ISO 26262, IEC 62304, DO-178C, EN 50128. Widely accepted by
       certification bodies.
     - MISRA Rust:2024 published. Ferrocene toolchain qualified to IEC 61508
       SIL 4 / ISO 26262 ASIL D (commercial). ``rustc`` itself is not yet
       independently qualified by a notified body, though the gap is closing
       rapidly.
   * - **Memory safety**
     - Requires discipline + MISRA rules + static analysis to prevent UB,
       null dereferences, buffer overflows. Safety is enforced by convention.
     - Ownership and borrow checker eliminate entire classes of memory bugs
       *at compile time*, structurally. ``unsafe`` blocks are explicit,
       localisable, and auditable.
   * - **Undefined behaviour**
     - Large surface area of UB (signed overflow, strict aliasing, unsequenced
       side effects). MISRA C:2023 restricts but does not eliminate all UB.
     - No UB outside ``unsafe``. Safe Rust provides a formally defined
       semantics (under active formalisation via RustBelt / a-mir-formality).
   * - **Type-level dimensionality**
     - Matrix dimensions require compile-time macros (``RON_KF_MAX_STATES``
       etc.) and fixed-size arrays. Template-like generics require C macros
       or code generation.
     - Const generics allow matrix dimensions, float precision, and buffer
       sizes to be true type parameters — checked at compile time, zero
       runtime cost, no macro gymnastics.
   * - **Embedded ecosystem**
     - Mature: HALs, RTOS, linker scripts, startup code all well-established.
     - ``#![no_std]``, ``embedded-hal``, RTIC, Embassy are production-ready.
       Growing but younger ecosystem.
   * - **Static analysis tooling (free)**
     - ``cppcheck``, Clang Static Analyzer (``scan-build``), ``clang-tidy``,
       CBMC (formal verification). Good coverage; some MISRA rules require
       manual review.
     - ``cargo clippy`` (built-in), ``cargo audit``, Kani verifier (AWS,
       Apache 2.0), Miri interpreter. Kani provides bounded model checking
       rivalling commercial tools.
   * - **Coverage tooling (free)**
     - ``gcov``/``lcov``/``gcovr`` (GCC), ``llvm-cov`` (Clang). Mature.
     - ``cargo-tarpaulin`` (MIT), ``cargo-llvm-cov``. MC/DC via ``llvm-cov``
       with ``--mcdc`` flag (LLVM 18+).
   * - **WCET analysis**
     - Measurement-based profiling + OTAWA (open-source research tool,
       IRIT/ONERA). No fully free production-grade static WCET tool exists
       for C; measurement + bounding is the practical free approach.
     - Same situation: LLVM IR-based analysis possible with OTAWA's LLVM
       frontend; measurement-based profiling via ``cargo-embed`` / ITM.
   * - **Cross-language interoperability**
     - C ABI is the universal lingua franca. Callable from any language.
     - Rust can export a C-compatible ABI (``extern "C"``, ``#[repr(C)]``),
       making the Rust library callable from C firmware with minimal overhead.
   * - **Learning curve / team familiarity**
     - Universally known in embedded. Very large talent pool.
     - Steeper learning curve. Smaller (but rapidly growing) embedded talent
       pool. Ownership model requires adjustment.

Decision
--------

Both tracks proceed in parallel. The **C11 implementation** is the **primary
interoperability target**: it produces a plain C static library usable from
any host language and on any MCU toolchain without restriction. The **Rust
implementation** is the **preferred new-development target** for projects
where the Rust toolchain is available: it offers stronger compile-time safety
guarantees, more expressive generic types for the matrix-heavy modules
(Kalman, state-space), and superior free static analysis via Kani.

Neither implementation is considered secondary or experimental. Both shall
maintain full feature parity and requirement traceability.

------------------------------------------------------------------------

C11 Implementation Track
=========================

Language Standard
-----------------

The C implementation **shall** conform to **ISO/IEC 9899:2011 (C11)**, using
only the subset compatible with C99 in the portable core. The sole C11-specific
feature is ``_Static_assert``; a C99-compatible fallback macro
``RON_STATIC_ASSERT`` is provided in ``ron_platform.h``.

Coding Standard (C Track)
--------------------------

All C source files **shall** conform to **MISRA C:2023** [REF-03], satisfying
RON-SR-030.

- All **Mandatory** rules: no exceptions.
- All **Required** rules: deviations require a formal Deviation Record (see
  below).
- **Advisory** guidelines: shall be satisfied where practical; non-conformances
  reviewed and recorded.

Deviation records are maintained in ``c/docs/deviations/MISRA_deviations.rst``.
Each record shall contain: rule number and text, file/line range, justification,
compensating measures, and reviewer sign-off.

Supplementary C Coding Rules
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Rule
     - Description
   * - No magic numbers
     - All numeric literals (other than ``0``, ``1``, ``-1`` in trivial
       contexts) shall be named ``#define`` constants or ``enum`` values.
   * - Exact-width integer types
     - Use ``<stdint.h>`` types (``uint8_t``, ``int32_t``, etc.) exclusively;
       never plain ``int``, ``long``.
   * - ``const``-correctness
     - All pointer parameters not modified by the callee shall be ``const``.
   * - Explicit casts
     - All conversions between types of differing width or signedness shall
       be written as explicit casts.
   * - Include guards
     - Every header: ``#ifndef``/``#define``/``#endif`` guard with a unique
       name derived from the file path.
   * - File header block
     - Every ``.c``/``.h`` begins with a standard comment block: file name,
       document ID, brief description, SPDX licence identifier.

Prohibited C Features
~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Feature
     - Reason
   * - ``malloc``, ``calloc``, ``realloc``, ``free``
     - Non-deterministic; violates RON-SR-003.
   * - Recursion
     - Unbounded stack; violates RON-SR-004.
   * - Variable-length arrays (VLAs)
     - Non-deterministic stack usage.
   * - ``setjmp``/``longjmp``
     - Bypasses structured control flow.
   * - ``goto``
     - Prohibited by MISRA C:2023 Rule 15.1.
   * - ``<stdio.h>``, ``<stdlib.h>`` (allocation functions)
     - Unavailable bare-metal; not required.
   * - ``_Thread_local``
     - Not universally available; concurrency is caller-managed.
   * - Implicit function declarations
     - All functions must be declared before use.

C Toolchain and Free Tools
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Tool / Category
     - Tool and Notes
   * - **Compiler**
     - GCC (``arm-none-eabi-gcc``, ``riscv32-unknown-elf-gcc``, etc.) or
       Clang/LLVM. Both are free and open-source. Flags: ``-Wall -Wextra
       -Wpedantic -Werror -Wconversion -Wshadow`` as minimum.
   * - **Static analysis**
     - ``cppcheck`` (GPL) — general C/C++ static analysis with a MISRA C
       addon (``cppcheck --addon=misra.py``). Covers a significant subset of
       MISRA C:2023 required rules automatically.
       ``clang-tidy`` + Clang Static Analyzer (``scan-build``, Apache 2.0) —
       detects UB, null dereferences, unintialised memory, integer overflows.
       Together these two free tools provide broad coverage; manual code
       review compensates for rules not mechanically checkable.
   * - **Formal verification**
     - **CBMC** (BSD-4-Clause, open source) — C Bounded Model Checker.
       Used to formally verify loop-free bounded functions (which all
       Regulon functions are, by design). Particularly valuable for verifying
       saturation, anti-windup, and fault-detection logic.
   * - **Coverage**
     - ``gcov``/``lcov``/``gcovr`` (GPL) for GCC builds. ``llvm-cov`` for
       Clang builds. Both support branch coverage; ``llvm-cov`` supports
       MC/DC via ``--mcdc`` (LLVM 18+), satisfying RON-QR-023.
   * - **Memory safety (host)**
     - AddressSanitizer (ASan) + UBSan (built into GCC/Clang, free) for
       host-native test builds. Valgrind (GPL) as supplementary check.
   * - **WCET**
     - **Measurement-based**: instrument with cycle counters via
       SysTick/DWT (ARM), RISC-V cycle CSR, or logic analyser. Document
       measured WCET as the accepted bound per RON-PR-003.
       **OTAWA** (LGPL, open-source research tool, IRIT/ONERA) for
       static WCET analysis where the target is supported. Acceptable as
       a supplementary analysis tool; measurement remains the primary method.
   * - **Testing framework**
     - **Unity** (MIT) — lightweight C unit test framework, suitable for
       host and on-target execution. **CMocka** (Apache 2.0) as alternative.
   * - **Build system**
     - CMake 3.21+ (BSD licence).
   * - **Documentation**
     - Doxygen (GPL) for API reference. Sphinx + ``breathe`` for integration
       with ``.rst`` documentation (both free).
   * - **Formatting**
     - ``clang-format`` (Apache 2.0). A ``.clang-format`` configuration file
       shall be committed to the repository root.

------------------------------------------------------------------------

Rust Implementation Track
==========================

Language Standard
-----------------

The Rust implementation **shall** target **Rust Edition 2021** [REF-08],
compiled with the stable toolchain (``rustc`` stable channel). Nightly
features **shall not** be used in library code; they may be used in
test/benchmark harnesses only, with explicit documentation.

The library crate shall be ``#![no_std]`` with an optional ``std`` feature
flag that enables ``std``-dependent functionality (e.g., richer ``Error``
trait impls) for host-based testing and tooling, while remaining bare-metal
compatible by default.

Crate and Module Structure
--------------------------

.. code-block:: none

   rust/
   ├── Cargo.toml                      -- workspace root
   ├── regulon/                        -- main library crate
   │   ├── Cargo.toml
   │   └── src/
   │       ├── lib.rs                  -- crate root; re-exports all public API
   │       ├── platform.rs             -- float type alias, math helpers, assert macros
   │       ├── error.rs                -- RonError enum (mirrors ron_fault_t)
   │       ├── pid/
   │       │   ├── mod.rs
   │       │   ├── config.rs
   │       │   ├── core.rs
   │       │   └── types.rs
   │       ├── filter/
   │       │   ├── mod.rs
   │       │   ├── lp1.rs
   │       │   ├── moving_avg.rs
   │       │   ├── biquad.rs
   │       │   └── rate_limiter.rs
   │       ├── feedforward.rs
   │       ├── gain_sched.rs
   │       ├── cascade.rs
   │       ├── trajectory/
   │       │   ├── mod.rs
   │       │   ├── trapezoidal.rs
   │       │   └── scurve.rs
   │       ├── kalman.rs
   │       ├── statespace.rs
   │       ├── observer.rs
   │       ├── autotune.rs
   │       ├── health.rs
   │       └── metrics.rs
   └── regulon-sys/                    -- optional C-ABI wrapper crate
       ├── Cargo.toml
       └── src/
           └── lib.rs                  -- #[no_mangle] extern "C" functions

Rust Naming Conventions
-----------------------

Rust idioms are used rather than mechanically mapping from C. The ``ron``
abbreviation is used as the crate/module prefix in ``use`` statements and
in the C-ABI wrapper.

.. list-table::
   :header-rows: 1
   :widths: 28 36 36

   * - Entity
     - Rust convention
     - Example
   * - Types (structs, enums)
     - ``PascalCase`` in module namespace
     - ``ron::pid::PidInstance``, ``ron::kalman::KalmanFilter``
   * - Traits
     - ``PascalCase``
     - ``ron::Controller``, ``ron::Filter``
   * - Functions / methods
     - ``snake_case``
     - ``pid.step()``, ``ron::pid::init()``
   * - Constants
     - ``SCREAMING_SNAKE_CASE``
     - ``ron::platform::FLOAT_MAX``
   * - Modules
     - ``snake_case``
     - ``ron::gain_sched``, ``ron::trajectory``
   * - C-ABI exported symbols
     - ``ron_<module>_<verb>`` (matching C track)
     - ``ron_pid_step``, ``ron_kf_update``

Key Rust Design Patterns
------------------------

**Const generics for matrix dimensions:**

The Kalman filter, state-space controller, and observer shall use const
generics to express matrix dimensions as type-level parameters, eliminating
the compile-time-constant macros required in the C track:

.. code-block:: rust

   pub struct KalmanFilter<const N: usize, const M: usize, const P: usize> {
       config: KalmanConfig<N, M, P>,
       state:  KalmanState<N, M>,
   }

   pub struct KalmanConfig<const N: usize, const M: usize, const P: usize> {
       pub A:  [[RonFloat; N]; N],
       pub B:  [[RonFloat; N]; P],
       pub H:  [[RonFloat; M]; N],
       pub Q:  [[RonFloat; N]; N],
       pub R:  [[RonFloat; M]; M],
       // ...
   }

This is zero-cost: the compiler generates a monomorphised implementation
for each concrete ``(N, M, P)`` combination used by the application.

**Typestate for mode transitions:**

The operating mode (automatic vs. manual) and initialisation state shall be
encoded in the type system using the typestate pattern where it adds
significant safety value:

.. code-block:: rust

   pub struct Pid<State> { config: PidConfig, state: PidState, _s: PhantomData<State> }
   pub struct Automatic;
   pub struct Manual;

   impl Pid<Automatic> {
       pub fn step(&mut self, r: RonFloat, y: RonFloat, dt: RonFloat)
           -> Result<(RonFloat, PidStatus), RonError> { ... }
       pub fn to_manual(self, manual_out: RonFloat) -> Pid<Manual> { ... }
   }
   impl Pid<Manual> {
       pub fn to_automatic(self) -> Pid<Automatic> { ... }  // bumpless transfer
   }

**No ``unsafe`` in library code (goal):**

All Regulon library code shall be written in safe Rust. The sole permissible
use of ``unsafe`` is in the C-ABI wrapper crate (``regulon-sys``) for
``extern "C"`` function bodies that must dereference raw pointers from C
callers. Each such block shall be individually justified with a safety
comment.

**``#[no_std]`` compatibility:**

The library crate shall not depend on ``std``. Permitted dependencies:

- ``core`` and ``alloc`` (no-alloc mode: ``alloc`` excluded by default).
- ``libm`` (MIT, provides ``no_std``-compatible math functions: ``fabsf``,
  ``sqrtf``, ``cosf``, etc.) — only used where the host target lacks
  hardware FP intrinsics.
- ``num-traits`` (MIT/Apache 2.0, ``no_std`` compatible) — for generic
  float arithmetic in const-generic code.

Coding Standard (Rust Track)
-----------------------------

The Rust implementation **shall** conform to **MISRA Rust:2024** [REF-04] to
the extent that rules are applicable to the ``no_std`` embedded domain.

In addition, the following rules are mandated:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Rule
     - Description
   * - All ``#[allow(...)]`` suppressions documented
     - Every Clippy or compiler lint suppression shall have a comment
       explaining why it is safe to suppress.
   * - No ``unwrap()``/``expect()`` in library code
     - Panics are not acceptable in embedded. All error paths shall return
       ``Result<_, RonError>`` or a safe default. ``unwrap()`` is permitted
       only in test code.
   * - No ``panic!`` in library code
     - Use ``Result`` returns. For truly unrecoverable conditions, trigger
       the platform fault handler via a registered callback (mirroring the
       C track's ``RON_ASSERT`` / ``fault_cb`` model).
   * - Explicit integer widths
     - Use ``u8``, ``u16``, ``u32``, ``i32``, ``f32``/``f64`` etc.
       explicitly; avoid ``usize`` for control-system quantities.
   * - ``#[repr(C)]`` on ABI-boundary types
     - Any type exposed through the C-ABI wrapper shall be annotated
       ``#[repr(C)]`` to guarantee layout compatibility.
   * - File-level documentation
     - Every ``.rs`` file shall begin with a ``//!`` module doc comment
       including: purpose, document ID (RON-IS-001), requirement IDs
       satisfied, and SPDX identifier.

Rust Toolchain and Free Tools
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Tool / Category
     - Tool and Notes
   * - **Compiler**
     - ``rustc`` stable (free, open-source, MIT/Apache 2.0). Cross-compile
       targets installed via ``rustup target add``. For certified builds,
       **Ferrocene** (Ferrous Systems, commercial but evaluation available)
       provides a qualified toolchain; the open-source ``rustc`` is used
       for development.
   * - **Static analysis / linting**
     - **``cargo clippy``** (built-in, free) — Rust's official linter;
       run with ``-D warnings -D clippy::all -D clippy::pedantic`` to
       enforce strictness.
       **``cargo audit``** (free) — checks dependencies for known
       security advisories.
   * - **Formal verification**
     - **Kani** (AWS, Apache 2.0, free) — bounded model checker for Rust.
       Exceptionally well-suited to Regulon: all step functions are bounded
       O(1) loops, making Kani proofs tractable. Kani harnesses shall be
       written alongside unit tests for all safety-critical functions.
   * - **Interpreter / UB detection**
     - **Miri** (built-in with nightly, free) — Rust's Mid-level IR
       interpreter detects UB, memory errors, and unsafe-block violations.
       Run on the test suite in CI with ``cargo +nightly miri test``.
   * - **Coverage**
     - **``cargo-llvm-cov``** (Apache 2.0, free) — source-based coverage
       via LLVM instrumentation. Supports branch coverage and MC/DC
       (``--mcdc``, LLVM 18+), satisfying RON-QR-023.
   * - **WCET**
     - Same approach as C track: measurement-based profiling via
       ``cargo-embed`` / defmt / ITM. OTAWA with LLVM IR frontend as
       supplementary static analysis.
   * - **Testing**
     - ``cargo test`` (built-in). **``proptest``** (MIT) for property-based
       testing (particularly valuable for verifying saturation invariants
       and filter stability across random parameter spaces).
   * - **Build system**
     - Cargo (built-in, free). Cross-compilation via ``.cargo/config.toml``
       and ``rustup target add``.
   * - **Documentation**
     - ``rustdoc`` (built-in, free). API docs generated with ``cargo doc``.
       Integrated into Sphinx via ``exhale`` or linked from the master
       ``.rst`` documentation.
   * - **Formatting**
     - ``rustfmt`` (built-in, free). A ``rustfmt.toml`` configuration file
       shall be committed to the repository root.

------------------------------------------------------------------------

Repository and File Structure
==============================

Directory Layout
----------------

The repository contains both implementation tracks under sibling top-level
directories. Shared artefacts (documentation, requirements, test
specifications) live at the root.

.. code-block:: none

   regulon/
   ├── docs/
   │   ├── SRS_ControlLib.rst          -- RON-SRS-001
   │   ├── SADS_ControlLib.rst         -- RON-SADS-001
   │   ├── IS_ControlLib.rst           -- RON-IS-001 (this document)
   │   ├── TP_ControlLib.rst           -- RON-TP-001
   │   └── deviations/
   │       ├── MISRA_C_deviations.rst
   │       └── MISRA_Rust_deviations.rst
   ├── c/                              -- C11 implementation track
   │   ├── include/
   │   │   └── ron/
   │   │       ├── ron_platform.h
   │   │       ├── ron_pid_types.h
   │   │       ├── ron_pid.h
   │   │       ├── ron_filter.h
   │   │       ├── ron_feedforward.h
   │   │       ├── ron_gain_sched.h
   │   │       ├── ron_cascade.h
   │   │       ├── ron_trajectory.h
   │   │       ├── ron_kalman.h
   │   │       ├── ron_statespace.h
   │   │       ├── ron_observer.h
   │   │       ├── ron_autotune.h
   │   │       ├── ron_health.h
   │   │       └── ron_metrics.h
   │   ├── src/
   │   │   ├── ron_pid_config.c
   │   │   ├── ron_pid_core.c
   │   │   ├── ron_pid_fault.c
   │   │   ├── ron_pid_api.c
   │   │   ├── ron_filter.c
   │   │   ├── ron_feedforward.c
   │   │   ├── ron_gain_sched.c
   │   │   ├── ron_cascade.c
   │   │   ├── ron_trajectory_trap.c
   │   │   ├── ron_trajectory_scurve.c
   │   │   ├── ron_kalman.c
   │   │   ├── ron_statespace.c
   │   │   ├── ron_observer.c
   │   │   ├── ron_autotune.c
   │   │   ├── ron_health.c
   │   │   └── ron_metrics.c
   │   ├── test/
   │   │   ├── unit/          (test_ron_pid_*.c, test_ron_filter_*.c, ...)
   │   │   └── framework/unity/
   │   ├── cmake/
   │   │   ├── toolchains/
   │   │   │   ├── arm-none-eabi.cmake
   │   │   │   ├── riscv32-unknown-elf.cmake
   │   │   │   └── host-x86_64.cmake
   │   │   └── ron_options.cmake
   │   └── CMakeLists.txt
   ├── rust/                           -- Rust Edition 2021 track
   │   ├── Cargo.toml                  -- workspace
   │   ├── regulon/                    -- #![no_std] library crate
   │   │   ├── Cargo.toml
   │   │   └── src/
   │   │       ├── lib.rs
   │   │       ├── platform.rs
   │   │       ├── error.rs
   │   │       ├── pid/
   │   │       ├── filter/
   │   │       ├── feedforward.rs
   │   │       ├── gain_sched.rs
   │   │       ├── cascade.rs
   │   │       ├── trajectory/
   │   │       ├── kalman.rs
   │   │       ├── statespace.rs
   │   │       ├── observer.rs
   │   │       ├── autotune.rs
   │   │       ├── health.rs
   │   │       └── metrics.rs
   │   ├── regulon-sys/                -- C-ABI wrapper crate
   │   │   ├── Cargo.toml
   │   │   └── src/lib.rs
   │   └── .cargo/config.toml
   ├── .github/workflows/
   │   ├── ci_c.yml
   │   └── ci_rust.yml
   ├── CHANGELOG.rst
   ├── LICENSE
   └── README.rst

Header Inclusion Model (C Track)
---------------------------------

Consumers of the C library **shall** include only the relevant public header(s)
under ``c/include/ron/``. For PID-only use:

.. code-block:: c

   #include "ron/ron_pid.h"

The headers ``ron_platform.h`` and ``ron_pid_types.h`` are transitively included.
Internal translation units may include internal headers directly. No internal
header (e.g., ``ron_pid_core_internal.h``) is installed or part of the public
API surface.

------------------------------------------------------------------------

Platform Abstraction Header: ``ron_platform.h`` (C Track)
==========================================================

This header is the single point of configuration for all platform-dependent
behaviour in the C implementation. It **shall** be the first header included
in every ``.c`` file.

.. code-block:: c

   /*
    * @file    ron_platform.h
    * @brief   Platform portability layer for the Regulon library.
    * @doc     RON-IS-001
    * @version 1.0.0
    * SPDX-License-Identifier: MIT
    */

   #ifndef RON_PLATFORM_H
   #define RON_PLATFORM_H

   #include <stdint.h>
   #include <stdbool.h>
   #include <float.h>

   /* ------------------------------------------------------------------ */
   /* Floating-point precision selection                                  */
   /* Define RON_USE_DOUBLE=1 at compile time to select double precision */
   /* ------------------------------------------------------------------ */
   #if defined(RON_USE_DOUBLE) && (RON_USE_DOUBLE == 1)
       typedef double ron_float_t;
       #define RON_FLOAT_MAX  DBL_MAX
       #define RON_FLOAT_MIN  (-DBL_MAX)
       #define RON_FLOAT_EPSILON  DBL_EPSILON
       #define RON_FLOAT_C(x) (x)        /* double literal suffix */
   #else
       typedef float ron_float_t;
       #define RON_FLOAT_MAX  FLT_MAX
       #define RON_FLOAT_MIN  (-FLT_MAX)
       #define RON_FLOAT_EPSILON  FLT_EPSILON
       #define RON_FLOAT_C(x) (x##F)     /* float literal suffix */
   #endif

   /* ------------------------------------------------------------------ */
   /* Compile-time assertion                                              */
   /* C11: use _Static_assert. Fallback for C99 toolchains.              */
   /* ------------------------------------------------------------------ */
   #if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
       #define RON_STATIC_ASSERT(cond, msg)  _Static_assert(cond, msg)
   #else
       /* C99 fallback: typedef trick causes a compile error on failure  */
       #define RON_STATIC_ASSERT(cond, msg) \
           typedef char ron_static_assert_##__LINE__[(cond) ? 1 : -1]
   #endif

   /* ------------------------------------------------------------------ */
   /* Runtime assertion                                                   */
   /* Override RON_ASSERT by defining it before including this header.  */
   /* Default: no-op (for production builds). A debug build should map   */
   /* this to a platform fault handler or breakpoint.                    */
   /* ------------------------------------------------------------------ */
   #ifndef RON_ASSERT
       #define RON_ASSERT(cond)  ((void)(cond))
   #endif

   /* ------------------------------------------------------------------ */
   /* Floating-point classification helpers (portable, no <math.h>)      */
   /* IEEE 754 mandates these properties; valid for float and double.     */
   /* ------------------------------------------------------------------ */

   /**
    * @brief Returns non-zero if x is a NaN (Not-a-Number).
    * Relies on the IEEE 754 property that NaN != NaN.
    */
   #define RON_ISNAN(x)   ((x) != (x))

   /**
    * @brief Returns non-zero if x is positive or negative infinity.
    * Relies on the IEEE 754 property that only Inf values exceed FLOAT_MAX.
    */
   #define RON_ISINF(x)   (((x) > RON_FLOAT_MAX) || ((x) < RON_FLOAT_MIN))

   /**
    * @brief Returns non-zero if x is neither NaN nor infinite (is finite).
    */
   #define RON_ISFINITE(x) (!RON_ISNAN(x) && !RON_ISINF(x))

   /* ------------------------------------------------------------------ */
   /* Inline utility: saturating clamp                                   */
   /* ------------------------------------------------------------------ */

   /**
    * @brief  Clamps value x to the closed interval [lo, hi].
    * @note   Caller guarantees lo <= hi (validated by configuration layer).
    */
   static inline ron_float_t ron_clamp(ron_float_t x,
                                          ron_float_t lo,
                                          ron_float_t hi)
   {
       ron_float_t result = x;
       if (result < lo) { result = lo; }
       if (result > hi) { result = hi; }
       return result;
   }

   /**
    * @brief  Returns the absolute value of x without depending on <math.h>.
    */
   static inline ron_float_t ron_fabs(ron_float_t x)
   {
       return (x < RON_FLOAT_C(0.0)) ? (-x) : x;
   }

   /* ------------------------------------------------------------------ */
   /* Compiler hint macros                                                */
   /* ------------------------------------------------------------------ */

   /** Mark a function as not returning (used for fault handlers). */
   #if defined(__GNUC__) || defined(__clang__)
       #define RON_NORETURN  __attribute__((noreturn))
   #elif defined(_MSC_VER)
       #define RON_NORETURN  __declspec(noreturn)
   #else
       #define RON_NORETURN  /* unsupported */
   #endif

   /** Suggest the compiler inline a function (performance hint only). */
   #define RON_INLINE  static inline

   #endif /* RON_PLATFORM_H */

------------------------------------------------------------------------

Type Definitions Header: ``ron_pid_types.h``
==============================================

Defines all public enumeration and structure types. This header has no dependencies other than ``ron_platform.h``.

.. code-block:: c

   /*
    * @file    ron_pid_types.h
    * @brief   Public type definitions for the Regulon PID controller module.
    * @doc     RON-IS-001
    * @version 1.0.0
    * SPDX-License-Identifier: MIT
    */

   #ifndef RON_PID_TYPES_H
   #define RON_PID_TYPES_H

   #include "ron/ron_platform.h"

   /* ================================================================== */
   /* Enumerations                                                        */
   /* ================================================================== */

   /**
    * @brief  Anti-windup scheme selection.
    *
    * Satisfies: RON-FR-030 – FR-034.
    */
   typedef enum
   {
       RON_AW_NONE       = 0, /**< Anti-windup disabled.                     */
       RON_AW_BACK_CALC  = 1, /**< Back-calculation (tracking) method.       */
       RON_AW_CLAMPING   = 2  /**< Conditional integration clamping method.  */
   } ron_aw_mode_t;

   /**
    * @brief  Derivative term computation mode.
    *
    * Satisfies: RON-FR-005.
    */
   typedef enum
   {
       RON_DERIV_ON_ERROR       = 0, /**< D term uses error difference.       */
       RON_DERIV_ON_MEASUREMENT = 1  /**< D term uses measurement difference
                                           (default — no derivative kick).     */
   } ron_deriv_mode_t;

   /**
    * @brief  Controller operating mode.
    *
    * Satisfies: RON-FR-040 – FR-042.
    */
   typedef enum
   {
       RON_MODE_AUTOMATIC = 0, /**< Closed-loop PID active.                  */
       RON_MODE_MANUAL    = 1  /**< Caller supplies output; integral frozen. */
   } ron_op_mode_t;

   /**
    * @brief  Safe-state output policy applied on fault detection.
    *
    * Satisfies: RON-SR-011.
    */
   typedef enum
   {
       RON_SAFE_HOLD_LAST = 0, /**< Freeze output at last valid value.       */
       RON_SAFE_ZERO      = 1, /**< Drive output to 0.0.                     */
       RON_SAFE_CONSTANT  = 2  /**< Drive output to ron_pid_config_t.safe_value. */
   } ron_safe_policy_t;

   /**
    * @brief  Integration method for the integral term.
    *
    * Satisfies: RON-FR-004.
    */
   typedef enum
   {
       RON_INTEG_EULER       = 0, /**< Rectangular (Euler forward) — default. */
       RON_INTEG_TRAPEZOIDAL = 1  /**< Trapezoidal (Tustin) method.           */
   } ron_integ_method_t;

   /* ================================================================== */
   /* Fault and status bitmasks                                          */
   /* ================================================================== */

   /**
    * @brief  Fault code register type.
    *
    * Multiple fault bits may be OR-ed together. Once set, bits persist until
    * explicitly cleared by ron_pid_fault_clear().
    *
    * Satisfies: RON-SR-010, RON-SR-013.
    */
   typedef uint8_t ron_fault_t;

   #define RON_FAULT_NONE              ((ron_fault_t)0x00U) /**< No fault.                       */
   #define RON_FAULT_INPUT_NAN         ((ron_fault_t)0x01U) /**< r or y is NaN or Inf.           */
   #define RON_FAULT_OUTPUT_NAN        ((ron_fault_t)0x02U) /**< Computed u is NaN or Inf.       */
   #define RON_FAULT_INTEGRAL_OVERFLOW ((ron_fault_t)0x04U) /**< |I| exceeded overflow threshold.*/
   #define RON_FAULT_CONFIG_INVALID    ((ron_fault_t)0x08U) /**< Configuration inconsistency.    */
   #define RON_FAULT_NULL_POINTER      ((ron_fault_t)0x10U) /**< NULL pointer in API call.       */

   /**
    * @brief  Status word returned after each computation step.
    *
    * Satisfies: RON-FR-070.
    */
   typedef uint16_t ron_status_t;

   #define RON_STATUS_OK               ((ron_status_t)0x0000U) /**< Normal operation.            */
   #define RON_STATUS_SATURATED        ((ron_status_t)0x0001U) /**< Output was clamped.          */
   #define RON_STATUS_RATE_LIMITED     ((ron_status_t)0x0002U) /**< Output delta was clamped.    */
   #define RON_STATUS_AW_ACTIVE        ((ron_status_t)0x0004U) /**< Anti-windup correction applied.*/
   #define RON_STATUS_MANUAL_MODE      ((ron_status_t)0x0008U) /**< Controller in manual mode.   */
   #define RON_STATUS_FAULT            ((ron_status_t)0x0010U) /**< Fault register is non-zero.  */
   #define RON_STATUS_SP_FILTER_ACTIVE ((ron_status_t)0x0020U) /**< Setpoint filter is active.   */
   #define RON_STATUS_NORMALISED       ((ron_status_t)0x0040U) /**< Normalisation is enabled.    */

   /* ================================================================== */
   /* Fault handler callback type                                        */
   /* ================================================================== */

   /**
    * @brief  Fault notification callback function pointer type.
    *
    * When registered in ron_pid_config_t, this function is called once each
    * time a new fault bit is set. It is invoked from within ron_pid_step(),
    * so it must be ISR-safe if the controller is used in an ISR context.
    * The callback must not call any Regulon API function (no re-entry).
    *
    * @param[in] fault  The newly detected fault code (single bit, not accumulated).
    */
   typedef void (*ron_fault_cb_t)(ron_fault_t fault);

   /* ================================================================== */
   /* Configuration structure                                            */
   /* ================================================================== */

   /**
    * @brief  Complete configuration record for one PID controller instance.
    *
    * This structure is copied by value into ron_pid_instance_t at
    * initialisation. The caller does not need to keep it alive afterwards.
    *
    * All fields are validated by ron_pid_config_validate() before use.
    *
    * Satisfies: RON-FR-001 – FR-007, RON-FR-010 – FR-013,
    *            RON-FR-020 – FR-027, RON-FR-030 – FR-035,
    *            RON-SR-010 – SR-013.
    */
   typedef struct
   {
       /* ── Gain parameters (parallel form) ─────────────────────────── */
       ron_float_t  Kp;            /**< Proportional gain. Must be >= 0.        */
       ron_float_t  Ki;            /**< Integral gain.     Must be >= 0.        */
       ron_float_t  Kd;            /**< Derivative gain.   Must be >= 0.        */

       /* ── Derivative filter ───────────────────────────────────────── */
       ron_float_t  N;             /**< Derivative LP filter bandwidth multiplier.
                                         0 disables the filter. Must be >= 0.   */

       /* ── 2DOF setpoint weights ───────────────────────────────────── */
       ron_float_t  b;             /**< Proportional SP weight in [0, 1].
                                         Default: 1.0 (standard PID).           */
       ron_float_t  c;             /**< Derivative SP weight in [0, 1].
                                         Default: 1.0 (standard PID).           */

       /* ── Output limits ───────────────────────────────────────────── */
       ron_float_t  u_min;         /**< Minimum control variable. Must be < u_max. */
       ron_float_t  u_max;         /**< Maximum control variable.               */

       /* ── Output rate limit ───────────────────────────────────────── */
       ron_float_t  du_max;        /**< Max |Δu| per second. <= 0 disables.    */

       /* ── Integral accumulator clamp ──────────────────────────────── */
       ron_float_t  I_min;         /**< Integral lower clamp. Must be < I_max. */
       ron_float_t  I_max;         /**< Integral upper clamp.                   */

       /* ── Anti-windup ─────────────────────────────────────────────── */
       ron_aw_mode_t  aw_mode;     /**< Anti-windup scheme.                     */
       ron_float_t    T_aw;        /**< Back-calc tracking time constant.
                                         Required > 0 when aw_mode=BACK_CALC.   */

       /* ── Integration method ──────────────────────────────────────── */
       ron_integ_method_t  integ_method; /**< Euler (default) or Trapezoidal.  */

       /* ── Derivative mode ─────────────────────────────────────────── */
       ron_deriv_mode_t  deriv_mode; /**< Default: RON_DERIV_ON_MEASUREMENT.  */

       /* ── Setpoint pre-filter ─────────────────────────────────────── */
       ron_float_t  tau_sp;        /**< SP first-order filter time constant.
                                         <= 0 disables the filter.              */

       /* ── Input/output normalisation ──────────────────────────────── */
       bool          normalise;     /**< Enable input/output normalisation.      */
       ron_float_t  in_min;        /**< Input engineering minimum (r and y).   */
       ron_float_t  in_max;        /**< Input engineering maximum. in_max > in_min if normalise=true. */
       ron_float_t  out_min;       /**< Output engineering minimum.            */
       ron_float_t  out_max;       /**< Output engineering maximum. out_max > out_min if normalise=true. */

       /* ── Safety / fault ──────────────────────────────────────────── */
       ron_safe_policy_t  safe_policy;   /**< Safe-state output policy on fault. */
       ron_float_t        safe_value;    /**< Used when safe_policy=SAFE_CONSTANT. */
       ron_float_t        I_overflow_thresh; /**< |I| > this triggers FAULT_INTEGRAL_OVERFLOW.
                                                   <= 0 disables this check.    */

       /* ── Setpoint-change integral reset ──────────────────────────── */
       ron_float_t  sp_reset_threshold; /**< |Δr| > this resets the integral.
                                              <= 0 disables this feature.       */

       /* ── Fault handler callback ──────────────────────────────────── */
       ron_fault_cb_t  fault_cb;   /**< Optional fault notification callback.
                                         NULL if not used.                      */
   } ron_pid_config_t;

   /* ================================================================== */
   /* Internal state structure                                           */
   /* ================================================================== */

   /**
    * @brief  Dynamic (mutable) state for one PID controller instance.
    *
    * This structure must be zero-initialised before first use, which is
    * performed by ron_pid_init(). The caller shall treat this as opaque;
    * fields shall only be read through the ron_pid_get_state() API.
    *
    * Satisfies: RON-FR-050 – FR-054, RON-FR-060 – FR-062.
    */
   typedef struct
   {
       ron_float_t   integral;        /**< Accumulated integral term.           */
       ron_float_t   y_prev;          /**< Previous normalised PV (for DoM).   */
       ron_float_t   e_D_prev;        /**< Previous derivative error (for DoE).*/
       ron_float_t   D_filt_prev;     /**< Previous filtered derivative output.*/
       ron_float_t   r_filt_prev;     /**< Previous filtered setpoint.         */
       ron_float_t   u_sat_prev;      /**< Previous saturated output.          */
       ron_float_t   u_prev;          /**< Previous pre-saturation output.     */
       ron_float_t   e_prev;          /**< Previous error (for trapezoidal).   */
       ron_op_mode_t mode;            /**< Current operating mode.             */
       ron_fault_t   fault_code;      /**< Accumulated fault register.         */
       ron_status_t  status;          /**< Status word from last step.         */
       bool           is_initialised;  /**< Guard: set by ron_pid_init().      */
   } ron_pid_state_t;

   /* ================================================================== */
   /* Instance structure (the opaque controller handle)                  */
   /* ================================================================== */

   /**
    * @brief  Complete PID controller instance.
    *
    * The caller allocates one of these (typically as a static variable or
    * as a field in a task control block). The library never allocates memory.
    *
    * Satisfies: RON-FR-060 – FR-062, RON-SR-003.
    *
    * @example
    * @code
    *   static ron_pid_instance_t  my_speed_pid;
    *   ron_pid_config_t cfg = { .Kp = 1.5F, .Ki = 0.3F, ... };
    *   (void)ron_pid_init(&my_speed_pid, &cfg);
    * @endcode
    */
   typedef struct
   {
       ron_pid_config_t  config;   /**< Configuration (copied at init, read-only during step). */
       ron_pid_state_t   state;    /**< Dynamic computation state.                             */
   } ron_pid_instance_t;

   /* ------------------------------------------------------------------ */
   /* Compile-time size assertions (RON-PR-021, RON-SR-022)            */
   /* ------------------------------------------------------------------ */
   RON_STATIC_ASSERT(sizeof(ron_pid_state_t)  <= 128U,
                      "ron_pid_state_t exceeds 128-byte RAM budget");
   RON_STATIC_ASSERT(sizeof(ron_pid_config_t) <= 128U,
                      "ron_pid_config_t exceeds 128-byte budget");

   #endif /* RON_PID_TYPES_H */

------------------------------------------------------------------------

Public API Header: ``ron_pid.h``
==================================

This is the **only** header that library consumers include. It provides the complete public API and includes ``ron_pid_types.h`` transitively.

.. code-block:: c

   /*
    * @file    ron_pid.h
    * @brief   Public API for the Regulon PID controller module.
    * @doc     RON-IS-001
    * @version 1.0.0
    * SPDX-License-Identifier: MIT
    *
    * Usage pattern (bare-metal, single controller, 1 kHz sample rate):
    *
    *   static ron_pid_instance_t pid;
    *
    *   void control_init(void) {
    *       ron_pid_config_t cfg = {
    *           .Kp = 2.0F, .Ki = 0.5F, .Kd = 0.1F,
    *           .N  = 10.0F,
    *           .b  = 1.0F, .c = 0.0F,
    *           .u_min = -100.0F, .u_max = 100.0F,
    *           .du_max = 500.0F,
    *           .I_min = -50.0F, .I_max = 50.0F,
    *           .aw_mode = RON_AW_BACK_CALC, .T_aw = 0.1F,
    *           .integ_method = RON_INTEG_EULER,
    *           .deriv_mode   = RON_DERIV_ON_MEASUREMENT,
    *           .normalise    = false,
    *           .safe_policy  = RON_SAFE_HOLD_LAST,
    *           .I_overflow_thresh = 200.0F,
    *       };
    *       (void)ron_pid_init(&pid, &cfg);
    *   }
    *
    *   void control_1kHz_isr(void) {
    *       ron_float_t  u;
    *       ron_status_t status;
    *       ron_fault_t  fault = ron_pid_step(&pid, setpoint, measurement,
    *                                            0.001F, &u, &status);
    *       if (fault != RON_FAULT_NONE) { handle_fault(fault); }
    *       actuator_set(u);
    *   }
    */

   #ifndef RON_PID_H
   #define RON_PID_H

   #include "ron/ron_pid_types.h"

   #ifdef __cplusplus
   extern "C" {
   #endif

   /* ================================================================== */
   /* Lifecycle                                                           */
   /* ================================================================== */

   /**
    * @brief  Initialise a PID controller instance.
    *
    * Validates the supplied configuration, copies it into the instance by
    * value, and zero-initialises all dynamic state. The caller does not need
    * to keep cfg alive after this call returns.
    *
    * @param[in,out] inst  Pointer to caller-allocated instance. Must not be NULL.
    * @param[in]     cfg   Pointer to configuration record. Must not be NULL.
    *
    * @return  RON_FAULT_NONE on success.
    * @return  RON_FAULT_NULL_POINTER if inst or cfg is NULL.
    * @return  RON_FAULT_CONFIG_INVALID if any configuration field is out of range
    *          or logically inconsistent.
    *
    * @pre     inst points to writable storage of sizeof(ron_pid_instance_t) bytes.
    * @post    inst->state.is_initialised == true  iff return == RON_FAULT_NONE.
    *
    * Satisfies: RON-FR-050, RON-SR-001, RON-SR-002.
    */
   ron_fault_t ron_pid_init(ron_pid_instance_t       *inst,
                               const ron_pid_config_t   *cfg);

   /**
    * @brief  Reset the dynamic state of an initialised controller.
    *
    * Zeroes the integral accumulator, derivative history, filter states, and
    * output history. Configuration parameters and operating mode are preserved.
    * The fault register is also cleared.
    *
    * @param[in,out] inst  Pointer to an initialised instance.
    *
    * @return  RON_FAULT_NONE on success.
    * @return  RON_FAULT_NULL_POINTER if inst is NULL.
    * @return  RON_FAULT_CONFIG_INVALID if inst is not initialised.
    *
    * Satisfies: RON-FR-051.
    */
   ron_fault_t ron_pid_reset(ron_pid_instance_t *inst);

   /* ================================================================== */
   /* Runtime                                                             */
   /* ================================================================== */

   /**
    * @brief  Execute one PID computation step.
    *
    * This is the primary runtime function. It shall be called once per sample
    * period with the current setpoint and process variable. The computed,
    * saturated, rate-limited control variable is written to *u_out.
    *
    * If the instance is in a latched fault state (fault_code != RON_FAULT_NONE),
    * no computation is performed and the safe-state output is returned immediately.
    * The caller must call ron_pid_fault_clear() before normal operation can resume.
    *
    * @param[in,out] inst      Pointer to an initialised instance.
    * @param[in]     r         Setpoint in engineering units (or normalised if enabled).
    * @param[in]     y         Process variable in engineering units.
    * @param[in]     dt        Sample period in seconds. Must be > 0 and finite.
    * @param[out]    u_out     Receives the control variable output. Must not be NULL.
    * @param[out]    status    Receives the status word for this step. Must not be NULL.
    *
    * @return  RON_FAULT_NONE if the step completed without fault.
    * @return  Non-zero ron_fault_t bitmask if a fault was detected or was already latched.
    *
    * @pre     inst was successfully initialised by ron_pid_init().
    * @pre     dt > 0.0 and RON_ISFINITE(dt).
    * @post    *u_out is in [inst->config.u_min, inst->config.u_max]
    *          OR a fault is active and the safe-state policy determines *u_out.
    *
    * Satisfies: RON-FR-001 – FR-007, RON-FR-020 – FR-035,
    *            RON-FR-070, RON-SR-010 – SR-013, RON-PR-001 – PR-002.
    */
   ron_fault_t ron_pid_step(ron_pid_instance_t *inst,
                               ron_float_t         r,
                               ron_float_t         y,
                               ron_float_t         dt,
                               ron_float_t        *u_out,
                               ron_status_t       *status);

   /* ================================================================== */
   /* Configuration update (runtime)                                     */
   /* ================================================================== */

   /**
    * @brief  Atomically update all three gain parameters.
    *
    * Validates the new gains first; only applies them if all are valid.
    * No other configuration fields are modified.
    *
    * @param[in,out] inst  Pointer to an initialised instance.
    * @param[in]     Kp    New proportional gain. Must be >= 0 and finite.
    * @param[in]     Ki    New integral gain.     Must be >= 0 and finite.
    * @param[in]     Kd    New derivative gain.   Must be >= 0 and finite.
    *
    * @return  RON_FAULT_NONE on success.
    * @return  RON_FAULT_CONFIG_INVALID if any gain is negative, NaN, or Inf.
    *
    * Satisfies: RON-FR-053.
    */
   ron_fault_t ron_pid_set_gains(ron_pid_instance_t *inst,
                                    ron_float_t         Kp,
                                    ron_float_t         Ki,
                                    ron_float_t         Kd);

   /**
    * @brief  Atomically update the output saturation limits.
    *
    * @param[in,out] inst   Pointer to an initialised instance.
    * @param[in]     u_min  New minimum output. Must be < u_max and finite.
    * @param[in]     u_max  New maximum output.
    *
    * @return  RON_FAULT_NONE on success.
    * @return  RON_FAULT_CONFIG_INVALID if u_min >= u_max or either is non-finite.
    *
    * Satisfies: RON-FR-021, RON-FR-053.
    */
   ron_fault_t ron_pid_set_limits(ron_pid_instance_t *inst,
                                     ron_float_t         u_min,
                                     ron_float_t         u_max);

   /**
    * @brief  Update the derivative filter coefficient N.
    *
    * @param[in,out] inst  Pointer to an initialised instance.
    * @param[in]     N     New filter bandwidth multiplier. 0 disables the filter.
    *                      Must be >= 0 and finite.
    *
    * @return  RON_FAULT_NONE on success.
    * @return  RON_FAULT_CONFIG_INVALID if N < 0 or non-finite.
    *
    * Satisfies: RON-FR-006, RON-FR-053.
    */
   ron_fault_t ron_pid_set_filter(ron_pid_instance_t *inst,
                                     ron_float_t         N);

   /**
    * @brief  Update the anti-windup scheme and back-calculation time constant.
    *
    * @param[in,out] inst    Pointer to an initialised instance.
    * @param[in]     mode    New anti-windup mode.
    * @param[in]     T_aw    Back-calculation time constant (only used when
    *                        mode == RON_AW_BACK_CALC). Must be > 0 in that case.
    *
    * @return  RON_FAULT_NONE on success.
    * @return  RON_FAULT_CONFIG_INVALID if mode == RON_AW_BACK_CALC and T_aw <= 0.
    *
    * Satisfies: RON-FR-033, RON-FR-053.
    */
   ron_fault_t ron_pid_set_antiwindup(ron_pid_instance_t *inst,
                                         ron_aw_mode_t       mode,
                                         ron_float_t         T_aw);

   /* ================================================================== */
   /* State mutation                                                      */
   /* ================================================================== */

   /**
    * @brief  Switch between automatic and manual operating modes.
    *
    * Performs bumpless transfer when switching from manual to automatic:
    * the integral accumulator is pre-loaded so the first automatic output
    * matches the last manual output.
    *
    * @param[in,out] inst        Pointer to an initialised instance.
    * @param[in]     mode        New operating mode.
    * @param[in]     manual_out  The desired output during manual mode, or the
    *                            handoff value when switching to automatic.
    *                            Must be finite. Will be clamped to [u_min, u_max].
    *
    * @return  RON_FAULT_NONE on success.
    * @return  RON_FAULT_NULL_POINTER if inst is NULL.
    * @return  RON_FAULT_CONFIG_INVALID if manual_out is NaN or Inf.
    *
    * Satisfies: RON-FR-040 – FR-042.
    */
   ron_fault_t ron_pid_set_mode(ron_pid_instance_t *inst,
                                   ron_op_mode_t       mode,
                                   ron_float_t         manual_out);

   /**
    * @brief  Pre-load the integral accumulator (warm start).
    *
    * The supplied value is clamped to [I_min, I_max] before being stored.
    *
    * @param[in,out] inst   Pointer to an initialised instance.
    * @param[in]     value  Desired integral accumulator value. Must be finite.
    *
    * @return  RON_FAULT_NONE on success.
    * @return  RON_FAULT_NULL_POINTER if inst is NULL.
    * @return  RON_FAULT_CONFIG_INVALID if value is NaN or Inf.
    *
    * Satisfies: RON-FR-052.
    */
   ron_fault_t ron_pid_set_integral(ron_pid_instance_t *inst,
                                       ron_float_t         value);

   /* ================================================================== */
   /* State inspection                                                   */
   /* ================================================================== */

   /**
    * @brief  Read all observable internal state in one call.
    *
    * Any output pointer that is NULL is silently skipped (no error).
    * This allows the caller to request only the fields of interest.
    *
    * @param[in]  inst      Pointer to an initialised instance.
    * @param[out] integral  Receives the current integral accumulator value.
    * @param[out] last_u    Receives the last saturated output value.
    * @param[out] last_D    Receives the last filtered derivative value.
    * @param[out] status    Receives the status word from the last step.
    * @param[out] fault     Receives the current accumulated fault code.
    *
    * @return  RON_FAULT_NONE on success.
    * @return  RON_FAULT_NULL_POINTER if inst is NULL.
    *
    * Satisfies: RON-FR-071, RON-QR-021.
    */
   ron_fault_t ron_pid_get_state(const ron_pid_instance_t *inst,
                                    ron_float_t              *integral,
                                    ron_float_t              *last_u,
                                    ron_float_t              *last_D,
                                    ron_status_t             *status,
                                    ron_fault_t              *fault);

   /* ================================================================== */
   /* Fault management                                                   */
   /* ================================================================== */

   /**
    * @brief  Clear the fault register, allowing normal operation to resume.
    *
    * @note   This function clears the fault bits but does NOT reset dynamic
    *         state. If the fault was caused by a numerical instability (e.g.,
    *         FAULT_INTEGRAL_OVERFLOW), callers should also call ron_pid_reset()
    *         to restore a known-good state before resuming automatic mode.
    *
    * @param[in,out] inst  Pointer to an initialised instance.
    *
    * @return  RON_FAULT_NONE on success.
    * @return  RON_FAULT_NULL_POINTER if inst is NULL.
    *
    * Satisfies: RON-SR-012.
    */
   ron_fault_t ron_pid_fault_clear(ron_pid_instance_t *inst);

   /* ================================================================== */
   /* Configuration validation (standalone utility)                      */
   /* ================================================================== */

   /**
    * @brief  Validate a configuration record without applying it.
    *
    * Useful for pre-checking configuration before ron_pid_init(), or for
    * validating externally loaded parameters (e.g., from NVM) before use.
    *
    * @param[in] cfg  Pointer to the configuration to validate. Must not be NULL.
    *
    * @return  RON_FAULT_NONE if the configuration is valid.
    * @return  RON_FAULT_NULL_POINTER if cfg is NULL.
    * @return  RON_FAULT_CONFIG_INVALID if any field is out of range or inconsistent.
    *
    * Satisfies: RON-SR-001, RON-SR-002.
    */
   ron_fault_t ron_pid_config_validate(const ron_pid_config_t *cfg);

   #ifdef __cplusplus
   }
   #endif

   #endif /* RON_PID_H */

------------------------------------------------------------------------

Compile-Time Configuration Constants
======================================

The following constants are defined in ``ron_platform.h`` (or in a user-supplied
``ron_config.h`` that ``ron_platform.h`` includes when ``RON_USE_CONFIG_HEADER``
is defined). They bound the static allocation of all fixed-size arrays in the library.

.. code-block:: c

   /* Signal conditioning */
   #ifndef RON_MA_MAX_WINDOW
   #define RON_MA_MAX_WINDOW        64U   /* max moving-average window length   */
   #endif

   #ifndef RON_BIQUAD_MAX_SECTIONS
   #define RON_BIQUAD_MAX_SECTIONS   8U   /* max cascaded biquad sections        */
   #endif

   /* Gain scheduling */
   #ifndef RON_GS_MAX_BREAKPOINTS
   #define RON_GS_MAX_BREAKPOINTS   16U   /* max scheduling breakpoints          */
   #endif

   /* Kalman filter */
   #ifndef RON_KF_MAX_STATES
   #define RON_KF_MAX_STATES         8U   /* max state dimension n               */
   #endif

   #ifndef RON_KF_MAX_MEASUREMENTS
   #define RON_KF_MAX_MEASUREMENTS   4U   /* max measurement dimension m         */
   #endif

   #ifndef RON_KF_MAX_INPUTS
   #define RON_KF_MAX_INPUTS         4U   /* max input dimension p               */
   #endif

   /* State-space controller / observer */
   #ifndef RON_SS_MAX_STATES
   #define RON_SS_MAX_STATES         8U
   #endif

   #ifndef RON_SS_MAX_OUTPUTS
   #define RON_SS_MAX_OUTPUTS        4U
   #endif

   #ifndef RON_SS_MAX_INPUTS
   #define RON_SS_MAX_INPUTS         4U
   #endif

   /* Auto-tuning */
   #ifndef RON_AT_MIN_CYCLES
   #define RON_AT_MIN_CYCLES         3U   /* minimum relay oscillation cycles    */
   #endif

   /* Health monitor */
   #ifndef RON_HEALTH_OSC_WINDOW
   #define RON_HEALTH_OSC_WINDOW    32U   /* oscillation detection window length */
   #endif

------------------------------------------------------------------------

New Module API Headers
======================

The headers below define the concrete C API for each new module introduced in
v1.1.0. The ``extern "C"`` guard and Doxygen documentation conventions
established for ``ron_pid.h`` apply equally to all headers.

``ron_filter.h`` — Signal Conditioning Filters
-------------------------------------------------

.. code-block:: c

   #ifndef RON_FILTER_H
   #define RON_FILTER_H
   #include "ron/ron_platform.h"
   #ifdef __cplusplus
   extern "C" {
   #endif

   /* ── First-order IIR low-pass ─────────────────────────────── */
   typedef struct { ron_float_t alpha;              } ron_lp1_config_t;
   typedef struct { ron_float_t y_prev;
                    bool         is_initialised;     } ron_lp1_state_t;
   typedef struct { ron_lp1_config_t cfg;
                    ron_lp1_state_t  state;         } ron_lp1_t;

   ron_fault_t ron_lp1_init    (ron_lp1_t *f, const ron_lp1_config_t *cfg);
   ron_fault_t ron_lp1_init_fc (ron_lp1_t *f, ron_float_t fc, ron_float_t dt);
   ron_fault_t ron_lp1_reset   (ron_lp1_t *f);
   ron_fault_t ron_lp1_step    (ron_lp1_t *f, ron_float_t x, ron_float_t *y);

   /* ── Moving average ───────────────────────────────────────── */
   typedef struct { uint8_t M;                       } ron_ma_config_t;
   typedef struct {
       ron_float_t buf[RON_MA_MAX_WINDOW];
       ron_float_t sum;
       uint8_t      idx;
       bool         is_initialised;
   } ron_ma_state_t;
   typedef struct { ron_ma_config_t cfg;
                    ron_ma_state_t  state;          } ron_ma_t;

   ron_fault_t ron_ma_init  (ron_ma_t *f, const ron_ma_config_t *cfg);
   ron_fault_t ron_ma_reset (ron_ma_t *f);
   ron_fault_t ron_ma_step  (ron_ma_t *f, ron_float_t x, ron_float_t *y);

   /* ── Biquad IIR (cascaded SOS) ────────────────────────────── */
   typedef struct {
       ron_float_t b0, b1, b2, a1, a2;
   } ron_biquad_section_t;

   typedef struct {
       ron_biquad_section_t sections[RON_BIQUAD_MAX_SECTIONS];
       uint8_t               n_sections;
   } ron_biquad_config_t;

   typedef struct {
       ron_float_t w1[RON_BIQUAD_MAX_SECTIONS];
       ron_float_t w2[RON_BIQUAD_MAX_SECTIONS];
       bool         is_initialised;
   } ron_biquad_state_t;

   typedef struct { ron_biquad_config_t cfg;
                    ron_biquad_state_t  state;      } ron_biquad_t;

   ron_fault_t ron_biquad_init         (ron_biquad_t *f,
                                           const ron_biquad_config_t *cfg);
   ron_fault_t ron_biquad_reset        (ron_biquad_t *f);
   ron_fault_t ron_biquad_step         (ron_biquad_t *f,
                                           ron_float_t x, ron_float_t *y);

   /* Coefficient generation helpers (write into an ron_biquad_section_t) */
   ron_fault_t ron_biquad_coeff_lp     (ron_biquad_section_t *s,
                                           ron_float_t fc, ron_float_t Q,
                                           ron_float_t dt);
   ron_fault_t ron_biquad_coeff_hp     (ron_biquad_section_t *s,
                                           ron_float_t fc, ron_float_t Q,
                                           ron_float_t dt);
   ron_fault_t ron_biquad_coeff_notch  (ron_biquad_section_t *s,
                                           ron_float_t f0, ron_float_t Q,
                                           ron_float_t dt);
   ron_fault_t ron_biquad_update_notch (ron_biquad_t *f, uint8_t section_idx,
                                           ron_float_t f0, ron_float_t Q,
                                           ron_float_t dt);

   /* ── Standalone rate limiter ──────────────────────────────── */
   typedef struct {
       ron_float_t rate_rise;   /**< Max positive Δy/s. */
       ron_float_t rate_fall;   /**< Max negative Δy/s (positive value). */
   } ron_ratelim_config_t;

   typedef struct {
       ron_float_t y_prev;
       bool         is_initialised;
   } ron_ratelim_state_t;

   typedef struct { ron_ratelim_config_t cfg;
                    ron_ratelim_state_t  state;     } ron_ratelim_t;

   ron_fault_t ron_ratelim_init  (ron_ratelim_t *f,
                                     const ron_ratelim_config_t *cfg);
   ron_fault_t ron_ratelim_reset (ron_ratelim_t *f);
   ron_fault_t ron_ratelim_step  (ron_ratelim_t *f, ron_float_t x,
                                     ron_float_t dt, ron_float_t *y);

   #ifdef __cplusplus
   }
   #endif
   #endif /* RON_FILTER_H */

``ron_gain_sched.h`` — Gain Scheduling
----------------------------------------

.. code-block:: c

   #ifndef RON_GAIN_SCHED_H
   #define RON_GAIN_SCHED_H
   #include "ron/ron_pid.h"
   #ifdef __cplusplus
   extern "C" {
   #endif

   typedef enum {
       RON_GS_HARD_SWITCH  = 0,
       RON_GS_LINEAR_INTERP = 1
   } ron_gs_mode_t;

   typedef struct {
       uint8_t              n_points;
       ron_float_t         sigma[RON_GS_MAX_BREAKPOINTS];
       ron_pid_config_t    configs[RON_GS_MAX_BREAKPOINTS];
       ron_gs_mode_t       mode;
       bool                 reset_integral_on_switch;
   } ron_gs_table_t;

   ron_fault_t ron_gs_init   (ron_gs_table_t *tbl);
   ron_fault_t ron_gs_update (const ron_gs_table_t *tbl,
                                 ron_pid_instance_t   *pid,
                                 ron_float_t           sigma);

   #ifdef __cplusplus
   }
   #endif
   #endif /* RON_GAIN_SCHED_H */

``ron_cascade.h`` — Cascade Controller
-----------------------------------------

.. code-block:: c

   #ifndef RON_CASCADE_H
   #define RON_CASCADE_H
   #include "ron/ron_pid.h"
   #ifdef __cplusplus
   extern "C" {
   #endif

   typedef struct {
       ron_pid_instance_t outer;
       ron_pid_instance_t inner;
   } ron_cascade_t;

   ron_fault_t ron_cascade_init  (ron_cascade_t          *c,
                                     const ron_pid_config_t *outer_cfg,
                                     const ron_pid_config_t *inner_cfg);
   ron_fault_t ron_cascade_reset (ron_cascade_t *c);
   ron_fault_t ron_cascade_step  (ron_cascade_t *c,
                                     ron_float_t    r_out,
                                     ron_float_t    y_out,
                                     ron_float_t    y_in,
                                     ron_float_t    dt,
                                     ron_float_t   *u_out,
                                     ron_status_t  *status);
   ron_fault_t ron_cascade_set_mode (ron_cascade_t *c,
                                        ron_op_mode_t  mode,
                                        ron_float_t    manual_out);

   #ifdef __cplusplus
   }
   #endif
   #endif /* RON_CASCADE_H */

``ron_trajectory.h`` — Trajectory Generators
-----------------------------------------------

.. code-block:: c

   #ifndef RON_TRAJECTORY_H
   #define RON_TRAJECTORY_H
   #include "ron/ron_platform.h"
   #ifdef __cplusplus
   extern "C" {
   #endif

   /* ── Trapezoidal profile ──────────────────────────────────── */
   typedef struct {
       ron_float_t v_max;
       ron_float_t a_max;
   } ron_trap_config_t;

   typedef struct {
       ron_float_t pos, vel, acc;
       ron_float_t target;
       uint8_t      phase;
       bool         hold;
       bool         is_initialised;
   } ron_trap_state_t;

   typedef struct { ron_trap_config_t cfg;
                    ron_trap_state_t  state; } ron_trap_t;

   ron_fault_t ron_trap_init       (ron_trap_t *t, const ron_trap_config_t *cfg,
                                       ron_float_t pos0);
   ron_fault_t ron_trap_set_target (ron_trap_t *t, ron_float_t target);
   ron_fault_t ron_trap_step       (ron_trap_t *t, ron_float_t dt,
                                       ron_float_t *pos, ron_float_t *vel,
                                       ron_float_t *acc, bool *finished);
   ron_fault_t ron_trap_hold       (ron_trap_t *t, bool hold);

   /* ── S-curve (jerk-limited) profile ──────────────────────── */
   typedef struct {
       ron_float_t v_max;
       ron_float_t a_max;
       ron_float_t j_max;
   } ron_scurve_config_t;

   typedef struct {
       ron_float_t pos, vel, acc, jrk;
       ron_float_t phase_time[7];
       ron_float_t t_phase;
       uint8_t      phase;
       bool         hold;
       bool         is_initialised;
   } ron_scurve_state_t;

   typedef struct { ron_scurve_config_t cfg;
                    ron_scurve_state_t  state; } ron_scurve_t;

   ron_fault_t ron_scurve_init       (ron_scurve_t *t,
                                         const ron_scurve_config_t *cfg,
                                         ron_float_t pos0);
   ron_fault_t ron_scurve_set_target (ron_scurve_t *t, ron_float_t target);
   ron_fault_t ron_scurve_step       (ron_scurve_t *t, ron_float_t dt,
                                         ron_float_t *pos, ron_float_t *vel,
                                         ron_float_t *acc, ron_float_t *jrk,
                                         bool *finished);
   ron_fault_t ron_scurve_hold       (ron_scurve_t *t, bool hold);

   #ifdef __cplusplus
   }
   #endif
   #endif /* RON_TRAJECTORY_H */

``ron_kalman.h`` — Discrete Linear Kalman Filter
---------------------------------------------------

.. code-block:: c

   #ifndef RON_KALMAN_H
   #define RON_KALMAN_H
   #include "ron/ron_platform.h"
   #ifdef __cplusplus
   extern "C" {
   #endif

   typedef struct {
       uint8_t      n, m, p;
       ron_float_t A[RON_KF_MAX_STATES][RON_KF_MAX_STATES];
       ron_float_t B[RON_KF_MAX_STATES][RON_KF_MAX_INPUTS];
       ron_float_t H[RON_KF_MAX_MEASUREMENTS][RON_KF_MAX_STATES];
       ron_float_t Q[RON_KF_MAX_STATES][RON_KF_MAX_STATES];
       ron_float_t R[RON_KF_MAX_MEASUREMENTS][RON_KF_MAX_MEASUREMENTS];
       ron_float_t x0[RON_KF_MAX_STATES];
       ron_float_t P0[RON_KF_MAX_STATES][RON_KF_MAX_STATES];
       bool         use_joseph_form;
       bool         steady_state;
       ron_float_t K_inf[RON_KF_MAX_STATES][RON_KF_MAX_MEASUREMENTS];
   } ron_kf_config_t;

   typedef struct {
       ron_float_t x_hat[RON_KF_MAX_STATES];
       ron_float_t P[RON_KF_MAX_STATES][RON_KF_MAX_STATES];
       bool         is_initialised;
   } ron_kf_state_t;

   typedef struct { ron_kf_config_t cfg;
                    ron_kf_state_t  state; } ron_kf_t;

   ron_fault_t ron_kf_init       (ron_kf_t *kf, const ron_kf_config_t *cfg);
   ron_fault_t ron_kf_reset      (ron_kf_t *kf);
   ron_fault_t ron_kf_predict    (ron_kf_t *kf,
                                     const ron_float_t u[RON_KF_MAX_INPUTS]);
   ron_fault_t ron_kf_update     (ron_kf_t *kf,
                                     const ron_float_t z[RON_KF_MAX_MEASUREMENTS],
                                     bool z_valid);
   ron_fault_t ron_kf_get_state  (const ron_kf_t *kf,
                                     ron_float_t x_hat[RON_KF_MAX_STATES]);

   #ifdef __cplusplus
   }
   #endif
   #endif /* RON_KALMAN_H */

``ron_autotune.h`` — Relay Feedback Auto-Tuning
-------------------------------------------------

.. code-block:: c

   #ifndef RON_AUTOTUNE_H
   #define RON_AUTOTUNE_H
   #include "ron/ron_pid.h"
   #ifdef __cplusplus
   extern "C" {
   #endif

   typedef enum {
       RON_AT_RULE_ZN        = 0,
       RON_AT_RULE_TL        = 1,
       RON_AT_RULE_SOME_OS   = 2,
       RON_AT_RULE_NO_OS     = 3
   } ron_at_rule_t;

   typedef struct {
       ron_float_t  relay_amplitude;
       ron_float_t  hysteresis;
       ron_float_t  u_bias;
       uint8_t       min_cycles;
       ron_float_t  timeout_s;
       ron_at_rule_t tuning_rule;
   } ron_at_config_t;

   typedef struct {
       ron_float_t  Ku, Tu;
       ron_float_t  Kp_result, Ki_result, Kd_result;
       uint8_t       phase;
       bool          done;
       bool          aborted;
       bool          is_initialised;
       /* internal oscillation tracking fields omitted for brevity */
   } ron_at_state_t;

   typedef struct { ron_at_config_t cfg;
                    ron_at_state_t  state; } ron_at_t;

   ron_fault_t ron_autotune_init    (ron_at_t *at, const ron_at_config_t *cfg);
   ron_fault_t ron_autotune_start   (ron_at_t *at, ron_pid_instance_t *pid);
   ron_fault_t ron_autotune_step    (ron_at_t *at, ron_float_t r,
                                        ron_float_t y, ron_float_t dt,
                                        ron_float_t *u_out);
   ron_fault_t ron_autotune_apply   (ron_at_t *at, ron_pid_instance_t *pid);
   ron_fault_t ron_autotune_abort   (ron_at_t *at, ron_pid_instance_t *pid);
   ron_fault_t ron_autotune_results (const ron_at_t *at,
                                        ron_float_t *Ku, ron_float_t *Tu,
                                        ron_float_t *Kp, ron_float_t *Ki,
                                        ron_float_t *Kd);

   #ifdef __cplusplus
   }
   #endif
   #endif /* RON_AUTOTUNE_H */

``ron_health.h`` — Control Loop Health Monitor
------------------------------------------------

.. code-block:: c

   #ifndef RON_HEALTH_H
   #define RON_HEALTH_H
   #include "ron/ron_pid_types.h"
   #ifdef __cplusplus
   extern "C" {
   #endif

   typedef uint8_t ron_health_status_t;
   #define RON_HEALTH_OK              ((ron_health_status_t)0x00U)
   #define RON_HEALTH_OUTPUT_STUCK    ((ron_health_status_t)0x01U)
   #define RON_HEALTH_DIVERGING       ((ron_health_status_t)0x02U)
   #define RON_HEALTH_OSCILLATING     ((ron_health_status_t)0x04U)
   #define RON_HEALTH_SENSOR_DROPOUT  ((ron_health_status_t)0x08U)
   #define RON_HEALTH_SP_UNREACHABLE  ((ron_health_status_t)0x10U)

   typedef void (*ron_health_cb_t)(ron_health_status_t condition);

   typedef struct {
       ron_float_t       t_sat_max;
       ron_float_t       err_diverge_thresh;
       uint8_t            osc_count_thresh;
       ron_float_t       dead_band;
       ron_float_t       dropout_time;
       ron_float_t       ss_err_thresh;
       ron_float_t       settling_time;
       ron_health_cb_t   cb;
   } ron_health_config_t;

   typedef struct {
       ron_health_status_t status;
       ron_float_t         t_saturated;
       ron_float_t         t_dropout;
       ron_float_t         t_since_step;
       uint8_t              osc_window[RON_HEALTH_OSC_WINDOW];
       uint8_t              osc_idx;
       ron_float_t         e_prev;
       ron_float_t         y_prev;
       bool                 is_initialised;
   } ron_health_state_t;

   typedef struct { ron_health_config_t cfg;
                    ron_health_state_t  state; } ron_health_t;

   ron_fault_t ron_health_init   (ron_health_t *h,
                                     const ron_health_config_t *cfg);
   ron_fault_t ron_health_step   (ron_health_t *h, ron_float_t r,
                                     ron_float_t y, ron_float_t u,
                                     ron_float_t dt);
   ron_fault_t ron_health_clear  (ron_health_t *h);
   ron_fault_t ron_health_get    (const ron_health_t *h,
                                     ron_health_status_t *status);

   #ifdef __cplusplus
   }
   #endif
   #endif /* RON_HEALTH_H */

``ron_metrics.h`` — Runtime Performance Metrics
-------------------------------------------------

.. code-block:: c

   #ifndef RON_METRICS_H
   #define RON_METRICS_H
   #include "ron/ron_platform.h"
   #ifdef __cplusplus
   extern "C" {
   #endif

   typedef enum {
       RON_METRICS_CUMULATIVE = 0,
       RON_METRICS_WINDOWED   = 1
   } ron_metrics_mode_t;

   typedef struct {
       ron_metrics_mode_t mode;
       uint32_t            window_steps;   /* used if mode = WINDOWED */
       ron_float_t        band_pct;       /* settling band, e.g. 0.02 = 2%  */
       ron_float_t        settle_confirm; /* seconds within band to confirm  */
       ron_float_t        step_thresh;    /* |Δr| to detect a setpoint step  */
   } ron_metrics_config_t;

   typedef struct {
       ron_float_t IAE, ISE, ITAE;
       ron_float_t peak_overshoot;
       ron_float_t rise_time;
       ron_float_t settling_time;
   } ron_metrics_result_t;

   typedef struct {
       ron_metrics_config_t cfg;
       /* internal state fields omitted for brevity */
       bool enabled;
       bool is_initialised;
   } ron_metrics_t;

   ron_fault_t ron_metrics_init    (ron_metrics_t *m,
                                       const ron_metrics_config_t *cfg);
   ron_fault_t ron_metrics_reset   (ron_metrics_t *m);
   ron_fault_t ron_metrics_enable  (ron_metrics_t *m, bool enable);
   ron_fault_t ron_metrics_step    (ron_metrics_t *m, ron_float_t r,
                                       ron_float_t y, ron_float_t dt);
   ron_fault_t ron_metrics_get     (const ron_metrics_t *m,
                                       ron_metrics_result_t *out);

   #ifdef __cplusplus
   }
   #endif
   #endif /* RON_METRICS_H */

------------------------------------------------------------------------

Build System Specification
==========================

C Track — CMake
---------------

The C implementation **shall** use **CMake** (version ≥ 3.21) located under
``c/``. CMake is free, open-source (BSD licence), and supported by all major
embedded IDEs and CI environments.

``c/CMakeLists.txt``
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.21)

   project(regulon
       VERSION     1.0.0
       DESCRIPTION "Regulon Control Systems Library — C Implementation"
       LANGUAGES   C
   )

   set(CMAKE_C_STANDARD          11)
   set(CMAKE_C_STANDARD_REQUIRED ON)
   set(CMAKE_C_EXTENSIONS        OFF)

   include(cmake/ron_options.cmake)

   add_library(regulon STATIC
       src/ron_pid_config.c
       src/ron_pid_core.c
       src/ron_pid_fault.c
       src/ron_pid_api.c
       src/ron_filter.c
       src/ron_feedforward.c
       src/ron_gain_sched.c
       src/ron_cascade.c
       src/ron_trajectory_trap.c
       src/ron_trajectory_scurve.c
       src/ron_kalman.c
       src/ron_statespace.c
       src/ron_observer.c
       src/ron_autotune.c
       src/ron_health.c
       src/ron_metrics.c
   )

   target_include_directories(regulon
       PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
       PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
   )

   target_compile_options(regulon PRIVATE
       $<$<C_COMPILER_ID:GNU>:
           -Wall -Wextra -Wpedantic -Werror
           -Wconversion -Wshadow -Wundef
           -fno-common -fstack-usage
       >
       $<$<C_COMPILER_ID:Clang>:
           -Wall -Wextra -Wpedantic -Werror
           -Wconversion -Wshadow -Wundef
       >
   )

   if(RON_USE_DOUBLE)
       target_compile_definitions(regulon PUBLIC RON_USE_DOUBLE=1)
   endif()

   if(RON_BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
       enable_testing()
       add_subdirectory(test)
   endif()

   install(TARGETS regulon ARCHIVE DESTINATION lib)
   install(DIRECTORY include/ron DESTINATION include)

``c/cmake/ron_options.cmake``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cmake

   option(RON_USE_DOUBLE    "Use 64-bit double precision"       OFF)
   option(RON_BUILD_TESTS   "Build unit and integration tests"  ON)
   option(RON_ENABLE_ASSERT "Enable runtime RON_ASSERT checks"  OFF)

   if(RON_ENABLE_ASSERT)
       target_compile_definitions(regulon PRIVATE
           "RON_ASSERT(cond)=do { if(!(cond)) { __builtin_trap(); } } while(0)"
       )
   endif()

Example Toolchain File: ``c/cmake/toolchains/arm-none-eabi.cmake``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cmake

   # Usage: cmake -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake \
   #              -DCMAKE_C_FLAGS="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb" \
   #              -B build_arm -S c/

   set(CMAKE_SYSTEM_NAME       Generic)
   set(CMAKE_SYSTEM_PROCESSOR  arm)
   set(CMAKE_C_COMPILER        arm-none-eabi-gcc)
   set(CMAKE_AR                arm-none-eabi-ar)
   set(CMAKE_RANLIB            arm-none-eabi-ranlib)
   set(CMAKE_SIZE              arm-none-eabi-size)
   set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
   set(CMAKE_EXE_LINKER_FLAGS_INIT   "-nostartfiles -nostdlib")

Example Toolchain File: ``c/cmake/toolchains/riscv32-unknown-elf.cmake``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cmake

   set(CMAKE_SYSTEM_NAME       Generic)
   set(CMAKE_SYSTEM_PROCESSOR  riscv)
   set(CMAKE_C_COMPILER        riscv32-unknown-elf-gcc)
   set(CMAKE_AR                riscv32-unknown-elf-ar)
   set(CMAKE_RANLIB            riscv32-unknown-elf-ranlib)
   set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
   set(CMAKE_EXE_LINKER_FLAGS_INIT   "-nostartfiles -nostdlib")

Rust Track — Cargo
-------------------

The Rust implementation uses **Cargo** (built-in with ``rustc``, free). Cross-
compilation targets are managed with ``rustup target add`` and declared in
``rust/.cargo/config.toml``.

``rust/regulon/Cargo.toml``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: toml

   [package]
   name        = "regulon"
   version     = "1.0.0"
   edition     = "2021"
   description = "Regulon Control Systems Library — Rust implementation"
   license     = "MIT"

   [features]
   default  = []
   std      = []          # opt-in: enables std-dependent impls for host testing
   libm     = ["dep:libm"] # opt-in: software FP fallback via libm

   [dependencies]
   libm       = { version = "0.2", optional = true }
   num-traits = { version = "0.2", default-features = false }

   [dev-dependencies]
   proptest = "1"          # property-based testing (MIT)

   [profile.release]
   opt-level = "s"         # optimise for size; change to 3 for performance targets
   lto       = true
   panic     = "abort"     # required for no_std bare-metal

``rust/.cargo/config.toml``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: toml

   [build]
   # Default host target; overridden by --target flag or CI matrix

   [target.thumbv7em-none-eabihf]
   # Cortex-M4F / M7F with hardware FPU
   rustflags = ["-C", "link-arg=-Tlink.x"]

   [target.riscv32imc-unknown-none-elf]
   rustflags = ["-C", "link-arg=-Tlink.x"]

------------------------------------------------------------------------

Coding Conventions
==================

C Track — File Header Template
--------------------------------

Every ``.c`` and ``.h`` file shall begin with the following comment block:

.. code-block:: c

   /*
    * @file     <filename>
    * @brief    <one-line description>
    * @module   <ron_pid_api | ron_pid_core | ron_filter | ...>
    * @doc      RON-IS-001
    * @req      <comma-separated RON-FR/PR/SR requirement IDs satisfied>
    * @version  1.0.0
    * @author   TBD
    * SPDX-License-Identifier: MIT
    */

Rust Track — File Header Template
-----------------------------------

Every ``.rs`` file shall begin with a module-level doc comment:

.. code-block:: rust

   //! # `<module_name>`
   //!
   //! <One-line description.>
   //!
   //! **Document:** RON-IS-001  
   //! **Requirements:** RON-FR-xxx, RON-SR-xxx  
   //! **SPDX-License-Identifier:** MIT

C Track — Naming Conventions
-----------------------------

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Entity
     - Convention
     - Example
   * - Public functions
     - ``ron_<module>_<verb>[_noun]``
     - ``ron_pid_init``, ``ron_pid_set_gains``
   * - Internal (static) functions
     - ``<module>_<verb>[_noun]`` (no ``ron_`` prefix)
     - ``pid_core_compute_derivative``
   * - Public types (structs, enums, typedefs)
     - ``ron_<noun>_t``
     - ``ron_pid_config_t``, ``ron_fault_t``
   * - Enum constants
     - ``RON_<CATEGORY>_<VALUE>``
     - ``RON_AW_BACK_CALC``, ``RON_FAULT_NONE``
   * - Preprocessor macros
     - ``RON_<SCREAMING_SNAKE>``
     - ``RON_ISNAN``, ``RON_FLOAT_MAX``
   * - Local variables
     - ``lower_snake_case``
     - ``delta_u``, ``I_new``
   * - Function parameters
     - ``lower_snake_case``
     - ``u_min``, ``dt``
   * - Named constants (``#define``)
     - ``RON_<SCREAMING_SNAKE>``
     - ``RON_VERSION_MAJOR``, ``RON_MA_MAX_WINDOW``

C Track — Comment Style
------------------------

- **Function-level documentation** uses Doxygen ``@`` tags (``@brief``,
  ``@param``, ``@return``, ``@pre``, ``@post``). Doxygen (GPL, free) generates
  HTML/PDF API reference.
- **Inline comments** use ``/* ... */`` block style, compatible with all C
  dialects and MISRA C:2023 Rule 3.1.
- Every non-trivial algorithmic step shall have a comment explaining *what*
  is computed and *why*.

Rust Track — Naming Conventions
---------------------------------

Rust naming follows the standard language idioms (``rustfmt`` and ``cargo
clippy`` enforce them automatically):

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Entity
     - Convention
     - Example
   * - Types (structs, enums)
     - ``PascalCase`` within module namespace
     - ``ron::pid::PidConfig``, ``ron::kalman::KalmanFilter``
   * - Traits
     - ``PascalCase``
     - ``ron::Controller``, ``ron::Filter``
   * - Functions / methods
     - ``snake_case``
     - ``pid.step()``, ``KalmanFilter::new()``
   * - Constants / statics
     - ``SCREAMING_SNAKE_CASE``
     - ``ron::platform::FLOAT_MAX``
   * - Modules
     - ``snake_case``
     - ``ron::gain_sched``, ``ron::trajectory``
   * - C-ABI exported symbols (``regulon-sys``)
     - ``ron_<module>_<verb>`` (mirrors C track)
     - ``ron_pid_step``, ``ron_kf_update``

Rust Track — Comment Style
---------------------------

- **Public items** use ``///`` doc comments with Markdown. ``rustdoc``
  (built-in, free) generates HTML API reference with ``cargo doc``.
- **Module-level** docs use ``//!`` inner doc comments (see file header
  template above).
- Every ``unsafe`` block **shall** be preceded by a ``// SAFETY:`` comment
  explaining the invariants that make the block sound.

Function Length and Complexity (Both Tracks)
---------------------------------------------

- No function / method body shall exceed **60 lines** of executable code
  (excluding comment lines and blank lines).
- Cyclomatic complexity **shall not** exceed **10** per function (RON-QR-011).
  ``clang-tidy`` (C) and ``cargo clippy`` (Rust) both enforce this.
- Where the computation pipeline would exceed these limits it **shall** be
  split into named helper functions declared with internal linkage (``static``
  in C; ``fn`` within a private ``impl`` block in Rust).

------------------------------------------------------------------------

Integration Guide
=================

C Track — Minimal Integration Steps
-------------------------------------

1. **Allocate storage** for each controller as a ``static`` variable or field
   of a task control block. Do not allocate on the stack of a shallow function.

   .. code-block:: c

      static ron_pid_instance_t  position_pid;

2. **Populate a configuration record** and call ``ron_pid_init()``.

   .. code-block:: c

      static const ron_pid_config_t position_cfg = {
          .Kp = 3.0F, .Ki = 0.8F, .Kd = 0.05F,
          .N  = 10.0F, .b = 1.0F, .c = 0.0F,
          .u_min = -1000.0F, .u_max = 1000.0F,
          .du_max = 5000.0F,
          .I_min = -200.0F, .I_max = 200.0F,
          .aw_mode = RON_AW_BACK_CALC, .T_aw = 0.05F,
          .integ_method = RON_INTEG_EULER,
          .deriv_mode = RON_DERIV_ON_MEASUREMENT,
          .normalise = false,
          .safe_policy = RON_SAFE_HOLD_LAST,
          .I_overflow_thresh = 500.0F,
          .fault_cb = NULL,
      };

      void app_init(void) {
          ron_fault_t err = ron_pid_init(&position_pid, &position_cfg);
          RON_ASSERT(err == RON_FAULT_NONE);
      }

3. **Call** ``ron_pid_step()`` **once per sample period**.

   .. code-block:: c

      void control_task_10ms(void) {
          const ron_float_t dt      = 0.01F;
          ron_float_t       u;
          ron_status_t      status;

          ron_fault_t fault = ron_pid_step(
              &position_pid,
              get_position_setpoint(),
              encoder_read_position(),
              dt, &u, &status);

          if (fault != RON_FAULT_NONE) {
              motor_disable();
              return;
          }
          motor_set_torque(u);
      }

4. **Handle faults** via ``ron_pid_fault_clear()`` and optionally
   ``ron_pid_reset()`` before resuming automatic mode.

Rust Track — Minimal Integration Steps
----------------------------------------

1. **Add dependency** in the application ``Cargo.toml``:

   .. code-block:: toml

      [dependencies]
      regulon = { path = "../../rust/regulon", default-features = false }

2. **Construct and initialise** a ``Pid`` instance using the builder or struct
   literal. All fields are validated at compile time via const assertions where
   possible, and at runtime via ``Pid::new()`` which returns ``Result``.

   .. code-block:: rust

      use ron::pid::{Pid, PidConfig, AwMode, DerivMode, SafePolicy};

      static mut POSITION_PID: Option<Pid<Automatic>> = None;

      fn app_init() {
          let cfg = PidConfig {
              Kp: 3.0, Ki: 0.8, Kd: 0.05,
              N: 10.0, b: 1.0, c: 0.0,
              u_min: -1000.0, u_max: 1000.0,
              du_max: 5000.0,
              I_min: -200.0, I_max: 200.0,
              aw_mode: AwMode::BackCalc { T_aw: 0.05 },
              deriv_mode: DerivMode::OnMeasurement,
              normalise: None,
              safe_policy: SafePolicy::HoldLast,
              I_overflow_thresh: Some(500.0),
              fault_cb: None,
              ..PidConfig::default()
          };
          // SAFETY: single-core init, called once before scheduler starts
          unsafe { POSITION_PID = Some(Pid::new(cfg).unwrap()); }
      }

3. **Call** ``pid.step()`` each sample period.

   .. code-block:: rust

      fn control_task_10ms() {
          const DT: f32 = 0.01;
          // SAFETY: exclusive access guaranteed by task scheduler
          let pid = unsafe { POSITION_PID.as_mut().unwrap() };
          match pid.step(get_setpoint(), encoder_read(), DT) {
              Ok((u, _status)) => motor_set_torque(u),
              Err(e) => { motor_disable(); handle_fault(e); }
          }
      }

   For RTOS environments, wrapping the instance in a mutex (e.g.,
   ``cortex_m::interrupt::Mutex<core::cell::RefCell<...>>``) provides safe
   concurrent access without ``unsafe``.

Sample Rate Considerations (Both Tracks)
-----------------------------------------

- The ``dt`` parameter **must** match the actual elapsed time between calls.
  For hard real-time schedulers a fixed constant is appropriate; if jitter is
  significant, measure elapsed time and pass it directly.
- At very high sample rates (>50 kHz) with software floating-point, verify
  WCET fits within the CPU budget via measurement (see tool tables above).
- At very low sample rates (<1 Hz), verify the derivative filter pole
  :math:`z = 1/(1 + N \cdot dt)` remains meaningful; consider increasing
  ``N`` or disabling the filter.

ISR Safety (Both Tracks)
-------------------------

Both implementations are ISR-safe provided the instance is not concurrently
modified from another context. Specifically:

- **C track**: disable interrupts or use a critical section around any
  ``ron_pid_set_gains()`` / ``ron_pid_set_mode()`` call made outside the ISR
  that also calls ``ron_pid_step()``.
- **Rust track**: the borrow checker prevents concurrent mutable access at
  compile time when using safe abstractions (e.g., RTIC resources, Mutex).
  Instances accessed from ``unsafe`` code require the same mutual-exclusion
  discipline as the C track.

Neither implementation calls OS services, uses shared global mutable state,
or contains non-reentrant functions.

------------------------------------------------------------------------

Traceability Summary
====================

The table below maps IS deliverables to RON-SRS-001 requirements. Entries
marked ``(both)`` apply to both implementation tracks; ``(C)`` or ``(Rust)``
denote track-specific deliverables.

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - IS Deliverable
     - SRS Requirements Satisfied
   * - Language selection: C11 + Rust Edition 2021 (both)
     - RON-DC-001, RON-QR-001, RON-QR-003, RON-QR-004
   * - MISRA C:2023 coding standard (C)
     - RON-SR-030, RON-SR-033
   * - MISRA Rust:2024 + Clippy (Rust)
     - RON-SR-030, RON-SR-033
   * - Free toolchain: GCC/Clang + cppcheck + CBMC (C)
     - RON-SR-031, RON-SR-033, RON-QR-022 – QR-023
   * - Free toolchain: rustc + Clippy + Kani + Miri (Rust)
     - RON-SR-031, RON-SR-033, RON-QR-022 – QR-023
   * - ``ron_platform.h``: ``ron_float_t`` type alias (C)
     - RON-PR-010, RON-PR-011, RON-QR-001 – QR-003
   * - ``ron::platform::RonFloat`` type alias (Rust)
     - RON-PR-010, RON-PR-011, RON-QR-001
   * - ``RON_ISNAN``, ``RON_ISINF`` macros (C)
     - RON-SR-020, RON-PR-012
   * - ``ron_float_t.is_nan()``, ``.is_infinite()`` (Rust)
     - RON-SR-020, RON-PR-012
   * - ``ron_clamp()``, ``ron_fabs()`` (C)
     - RON-FR-020, RON-FR-035
   * - ``f32::clamp()``, ``f32::abs()`` (Rust std/libm)
     - RON-FR-020, RON-FR-035
   * - ``RON_STATIC_ASSERT`` + ``_Static_assert`` (C)
     - RON-PR-021, RON-DC-002
   * - Compile-time const assertions via ``const {}`` (Rust)
     - RON-PR-021
   * - ``ron_pid_types.h``: all enumerations (C)
     - RON-FR-004 – FR-005, RON-FR-033, RON-FR-040, RON-SR-011
   * - ``ron::pid`` enums: ``AwMode``, ``DerivMode``, etc. (Rust)
     - RON-FR-004 – FR-005, RON-FR-033, RON-FR-040, RON-SR-011
   * - ``ron_fault_t`` bitmask (C) / ``RonError`` enum (Rust)
     - RON-SR-010, RON-SR-013
   * - ``ron_status_t`` bitmask (C) / ``PidStatus`` bitflags (Rust)
     - RON-FR-070
   * - ``ron_pid_config_t`` / ``PidConfig`` (both)
     - RON-FR-001 – FR-007, RON-FR-010 – FR-013, RON-FR-020 – FR-035
   * - ``ron_pid_state_t`` / ``PidState`` (both)
     - RON-FR-050 – FR-054, RON-FR-060 – FR-062
   * - ``ron_pid_instance_t`` / ``Pid<State>`` typestate (both)
     - RON-SR-003, RON-FR-060 – FR-062, RON-PR-022
   * - Compile-time size assertions (both)
     - RON-PR-021
   * - Full PID API: init / reset / step / set_* / get_state / fault_clear (both)
     - RON-FR-040 – FR-054, RON-FR-070 – FR-071, RON-SR-001 – SR-002, RON-SR-012
   * - Compile-time constants: ``RON_MA_MAX_WINDOW``, ``RON_KF_MAX_STATES``, etc. (C)
     - RON-FR-116, RON-FR-121, RON-FR-301, RON-FR-601, RON-FR-607, RON-FR-723
   * - Const generics ``<const N: usize, …>`` replacing macros (Rust)
     - RON-FR-116, RON-FR-121, RON-FR-301, RON-FR-601, RON-FR-607, RON-FR-723
   * - Filter headers / modules: LP1, MA, biquad, rate limiter (both)
     - RON-FR-100 – FR-131
   * - Feed-forward API (both)
     - RON-FR-200 – FR-205
   * - Gain scheduling API (both)
     - RON-FR-300 – FR-306
   * - Cascade controller API (both)
     - RON-FR-400 – FR-406
   * - Trajectory generator API (both)
     - RON-FR-500 – FR-513
   * - Kalman filter API (both)
     - RON-FR-600 – FR-607
   * - State-space + observer API (both)
     - RON-FR-700 – FR-723
   * - Auto-tuning API (both)
     - RON-FR-800 – FR-807
   * - Health monitor API (both)
     - RON-FR-900 – FR-905
   * - Performance metrics API (both)
     - RON-FR-950 – FR-954
   * - CMake build + cross-compilation toolchain files (C)
     - RON-DC-005, RON-QR-004
   * - Cargo workspace + ``.cargo/config.toml`` cross targets (Rust)
     - RON-DC-005, RON-QR-004
   * - ``extern "C"`` guards in C headers / ``regulon-sys`` C-ABI crate (both)
     - RON-QR-001
   * - No heap allocation in either track (both)
     - RON-DC-002, RON-SR-003
   * - GitHub Actions CI workflows for both tracks (both)
     - RON-QR-020, RON-QR-022 – QR-023

------------------------------------------------------------------------

Open Items
==========

.. list-table::
   :header-rows: 1
   :widths: 10 60 30

   * - ID
     - Description
     - Resolution Target
   * - OI-01
     - MISRA C:2023 deviation records to be populated after first ``cppcheck
       --addon=misra.py`` scan on the C implementation. Deviations stored in
       ``docs/deviations/MISRA_C_deviations.rst``.
     - Implementation phase
   * - OI-02
     - MISRA Rust:2024 deviation records to be populated after first
       ``cargo clippy -D clippy::all`` pass on the Rust crate. Deviations
       stored in ``docs/deviations/MISRA_Rust_deviations.rst``.
     - Implementation phase
   * - OI-03
     - WCET measurement methodology and acceptance thresholds to be
       documented per integration target in RON-SIR-001 (TBD). OTAWA
       static analysis to be evaluated for supported targets.
     - Integration phase
   * - OI-04
     - Kani proof harnesses to be written for all RON-SR-010 fault
       detection paths and all saturation/anti-windup logic. Coverage
       target: all bounded reachable states.
     - Implementation phase
   * - OI-05
     - Test framework for C track confirmed as **Unity** (MIT); test runner
       integration with CTest to be finalised in RON-TP-001.
     - Test plan phase
   * - OI-06
     - Ferrocene toolchain evaluation: assess cost/benefit for projects
       requiring IEC 61508 SIL 3+ certification with the Rust track.
       Document recommendation in RON-SIR-001.
     - Integration phase
   * - OI-07
     - Licence selection (MIT placeholder) to be confirmed by project
       governance. Both tracks shall carry the same licence.
     - Project governance
   * - OI-08
     - ``regulon-sys`` C-ABI wrapper crate ABI stability policy to be
       defined: semantic versioning constraints, ``#[repr(C)]`` audit,
       and interoperability test suite against the C track headers.
     - Implementation phase

------------------------------------------------------------------------

*End of Document — RON-IS-001 v1.1.0*
