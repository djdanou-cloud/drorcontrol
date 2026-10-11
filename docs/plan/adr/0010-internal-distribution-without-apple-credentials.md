# ADR-0010: Internal distribution without Apple developer credentials

* Status: accepted
* Date: 2026-10-11
* Deciders: product owner
* Validation: CI packaging job (T0-10): universal `.app`, ad-hoc signature verified, launched on
  arm64 and x86_64 runners; SHA-256 checksum published with every artefact

## Context
ADR-0005 planned Developer ID signing, Hardened Runtime and Apple notarization, which needs a
paid Apple Developer account and credentials in CI. The product owner decided DrorControl is used
only on the team's own computers and shared as a file. It is not distributed through Apple, so
Apple credentials are not wanted.

## Decision drivers
* No Apple Developer account or Apple credentials anywhere in the project.
* The app still runs on Apple Silicon, which refuses unsigned code, and on Intel Macs.
* Recipients can check that a file is the one the team built.

## Options considered
| Option | Pros | Cons | Score |
|--------|------|------|-------|
| Developer ID + notarization (ADR-0005) | Opens without warnings | Needs an Apple account and credentials | rejected by owner |
| **Ad-hoc signature + checksum (chosen)** | No account; works on arm64; reproducible in CI | First launch per Mac needs "Open Anyway"; macOS may re-ask privacy prompts after updates | chosen |
| Self-made code-signing certificate | Stable signature, so fewer repeat prompts | Each Mac must trust the certificate once; key must be guarded | optional later |

## Decision
We will ship a universal (arm64 + x86_64) `DrorControl.app` that is **ad-hoc signed**
(`codesign --sign -`, every nested binary inside-out, no Apple identity). It is packaged as a
`.zip` and a `.dmg`, each published with a **SHA-256 checksum**. There is no notarization, no
Apple credentials, and no App Sandbox (as ADR-0005). The CI build verifies the signature
(`codesign --verify --deep --strict`), the architectures (`lipo`), and that the app starts on both
architectures.

First launch on each Mac, once, when the file arrived by browser, AirDrop or a messaging app (the
quarantine flag): try to open the app, then System Settings → Privacy & Security → **Open
Anyway**. Alternatively, run `xattr -dr com.apple.quarantine /Applications/DrorControl.app`.
Copies from a USB drive or network share usually carry no quarantine flag. Windows builds stay
unsigned likewise: SmartScreen → More info → Run anyway.

## Consequences
* Positive: no Apple account, no secrets in CI; T0-10 no longer waits on credentials.
* Negative / risks (with mitigations):
  * Gatekeeper warning on first launch. Mitigation: documented one-time step.
  * An ad-hoc signature changes with every build, so macOS may ask again for permissions it
    remembers by signature (e.g. Local Network). Mitigation: a self-made certificate trusted on
    the team's Macs, if this becomes annoying.
  * No Apple malware scan or revocation. Mitigation: share files over trusted channels; publish
    and check SHA-256 checksums. In-app updates (Sparkle, XC-003) verify their own EdDSA
    signatures independently of Apple.
  * Apple-restricted entitlements that need a provisioning profile (e.g. the multicast networking
    entitlement) are unavailable. Plain unsandboxed sockets do not need them; revisit if a
    sandboxed or App Store build is ever made.

## Follow-ups
* T0-10 re-scoped to "ad-hoc signed universal build packaged in CI with checksums".
* Plan 03 §4–5, plan 01 S0-6 and plan 08 packaging gate updated to match.
