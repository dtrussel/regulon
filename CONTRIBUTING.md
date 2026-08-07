# Contributing to Regulon

Thanks for considering a contribution. Regulon is developed spec-first: the
`.rst` documents under `docs/specs/` are the ground truth, and every change
to the C11 or Rust implementations traces back to a requirement ID and
forward to a test ID. This document is a quick orientation; the
authoritative rules live in [`AGENTS.md`](AGENTS.md) (repository-wide) and
[`regulon-c/AGENTS.md`](regulon-c/AGENTS.md) / `regulon-rs/AGENTS.md`
(implementation-specific) — read those before opening a pull request.

## Before you write code

1. Find the requirement ID(s) your change relates to:
   `rg -n "RON-FR-0XX" docs/specs/SRS_ControlLib.rst`
2. Read the corresponding design in `docs/specs/SADS_ControlLib.rst`.
3. Find or add the test ID(s) in `docs/specs/TP_ControlLib.rst` — **every
   test ID must exist in `TP_ControlLib.rst` before the test that uses it
   is written**.
4. Write or update the test/proof first, then the implementation.
5. Keep your change aligned with the active roadmap iteration
   (`docs/plans/c11-roadmap.md` for C11) — don't opportunistically start
   work on a future module.

## Rules that apply to every change

No `malloc`/`free`/`Box`/`Vec`, no recursion, no VLAs, no `goto`/`setjmp`,
no `panic!`/`unwrap()` in library code, no global mutable state, no magic
numbers, no unbounded loops. See `AGENTS.md` for the full list and the
per-language specifics (naming, error-handling order, permitted headers,
formal-proof conventions) in the implementation-specific `AGENTS.md`.

## Before opening a pull request

- [ ] Every new function has a `Satisfies:` + `Test:` annotation in the
      implementation-specific style.
- [ ] Every new test ID exists in `TP_ControlLib.rst`.
- [ ] Relevant lint, format, test, coverage, and formal-verification gates
      pass locally (see `regulon-c/AGENTS.md` for the exact commands, or
      `regulon-c/scripts/verify_pid.ps1` for a local runner on Windows).
- [ ] `CHANGELOG.rst` is updated.
- [ ] For C11 changes: `bash regulon-c/scripts/check_manifest.sh` passes
      (new sources/headers must be registered in
      `regulon-c/scripts/lib_sources.txt` / `format_files.txt`).

CI (`.github/workflows/ci_c.yml`) enforces the full gate set — GCC/Clang
builds with sanitizers, double-precision regression, `clang-format`,
`cppcheck`/MISRA C:2023, complexity, 100% statement/branch coverage, CBMC
formal proofs, ARM/RISC-V cross-compile smoke builds, and a source-manifest
drift check — on every push and pull request.

## Reporting bugs / requesting features

Use the issue templates under `.github/ISSUE_TEMPLATE/`. For anything that
might be a security issue, see [`SECURITY.md`](SECURITY.md) instead of
opening a public issue.

## License

By contributing, you agree that your contributions will be licensed under
the project's [MIT License](LICENSE).
