# Firmware Version Control & Release Clarity

## 1. Versioning Standard

Use semantic version format:

`vMAJOR.MINOR.PATCH` (example: `v1.0.0`)

Rules:

- **MAJOR**: breaking behavior or interface change
- **MINOR**: backward-compatible feature addition or tuning changes
- **PATCH**: bug fixes, non-breaking calibration adjustments, documentation-only fixes tied to release

## 2. Build Labels

Every build must include exactly one release class:

- `stable` — validated build approved for production or field deployment
- `experimental` — development/test build not approved for production

Recommended build identifier format:

`vX.Y.Z-stable` or `vX.Y.Z-experimental`

## 3. Release Notes Requirement

Every released build must include release notes.

Use `RELEASE_NOTES.md` as the canonical log, with one section per build containing:

- version
- release class (stable/experimental)
- date
- summary of changes
- BOM impact
- known issues
- validation status

## 4. Source Control Expectations

- Tag each released version in source control with the same version string.
- Keep one commit (or PR) that includes:
  1. code changes,
  2. any BOM/pin/protocol documentation updates,
  3. release note entry.

## 5. Stable Release Gate (Minimum)

Before marking a build `stable`, confirm:

- build and upload succeed on target board,
- core sensor-to-sound path passes smoke tests,
- no unresolved critical blocker in issue tracker,
- BOM alignment document reflects current hardware assumptions.
