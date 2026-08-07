# Security Policy

## Reporting a vulnerability

If you believe you've found a security vulnerability in Regulon (e.g. a
memory-safety issue, an out-of-bounds access, a case where the library's
"no dynamic allocation / bounded execution" guarantees don't hold, or a
condition that could cause an unsafe controller output), please report it
privately rather than opening a public issue:

- Preferred: use GitHub's private vulnerability reporting for this
  repository — go to the **Security** tab → **Report a vulnerability**
  (or `https://github.com/dtrussel/regulon/security/advisories/new`).
  This opens a private advisory visible only to the maintainer until a fix
  is ready.

Please include:

- The affected module(s), file(s), and (if known) requirement/test IDs
  (`RON-FR-*` / `RON-TC-*`) from `docs/specs/`.
- A minimal reproduction (a failing unit test, CBMC harness, or standalone
  C/Rust snippet is ideal).
- The impact you believe it has (e.g. undefined behavior, a violated
  safety requirement, a formal-proof counterexample).

## Scope

Regulon is a library, not a deployed service — there is no hosted
infrastructure to report against. Reports should be about the library code
itself: `regulon-c/`, `regulon-rs/`, the build/CI configuration, or the
specifications in `docs/specs/` if a requirement itself is unsafe.

## Response

This is a community-maintained project without a formal SLA. Reports will
be acknowledged as soon as reasonably possible, and a fix or mitigation
will be prioritized according to severity. Given the safety-relevant nature
of a control-systems library, memory-safety and bounded-execution issues
are treated as high priority.
