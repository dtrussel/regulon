## Summary

<!-- What does this change do, and why? Link the requirement ID(s) it
     satisfies, e.g. RON-FR-730. -->

## Implementation(s) touched

- [ ] C11 (`regulon-c/`)
- [ ] Rust (`regulon-rs/`)
- [ ] Specifications (`docs/specs/`)
- [ ] Build / CI

## Checklist

- [ ] Every new function has a `Satisfies:` + `Test:` annotation in the
      implementation-specific style (`AGENTS.md`).
- [ ] Every new test ID exists in `docs/specs/TP_ControlLib.rst`.
- [ ] `CHANGELOG.rst` updated.
- [ ] For C11: `bash regulon-c/scripts/check_manifest.sh` passes (new
      sources/headers registered in `regulon-c/scripts/lib_sources.txt` /
      `format_files.txt`).
- [ ] Relevant local gates pass (format, static analysis, tests, coverage,
      formal proofs where applicable — see `regulon-c/AGENTS.md` /
      `regulon-rs/AGENTS.md` for exact commands).

## Test plan

<!-- How was this verified? New/changed tests, local gate output, formal
     proof results, manual verification steps, etc. -->
