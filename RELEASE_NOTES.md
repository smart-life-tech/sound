# Release Notes

This file is the canonical firmware release log.

## [v0.1.0-experimental] - 2026-03-03

### Release Class

`experimental`

### Summary

- Established formal documentation package under `docs/`.
- Documented architecture, pin mapping, protocols, integration notes, setup/deployment, troubleshooting, BOM alignment, and release policy.
- Baseline for structured handoff and manufacturing-readiness process.

### BOM Impact

- No component substitution recorded in this release.
- Documentation now reflects Teensy 4.1 + SGTL5000 audio path assumptions.

### Known Issues

- Sensor calibration constants may require hardware-specific tuning.
- Verify flex normalization bounds against physical sensor output range.

### Validation

- Documentation-only release package generated.
- Firmware behavior not changed in this release.

---

## Release Entry Template

Copy this template for each new build:

```markdown
## [vX.Y.Z-stable|experimental] - YYYY-MM-DD

### Release Class
`stable` or `experimental`

### Summary
- ...

### BOM Impact
- ...

### Known Issues
- ...

### Validation
- Build: pass/fail
- Flash: pass/fail
- Smoke test: pass/fail
```
