# On-vehicle / hardware-in-the-loop CI — design notes

Status: **proposal, parked for future consideration** (2026-06-07). Nothing here is built or
wired up. This document records the analysis so it is actionable when/if we revisit it.

## Question that started this

> For ovms-modern we created self-hosted runners to do the CI — would that make sense here, and
> can we design CI tests that run on the actual hardware and test features?

## TL;DR conclusions

1. **Self-hosted runners do not help the existing compile CI**, and would *hurt* on two axes:
   - GitHub Actions is **free and unlimited for public repos** — this repo (the `kezarjg` fork) is
     public, so there are no metered minutes to save. A self-hosted box only adds cost/maintenance.
   - The current `build.yml` is already well-optimised (cached xtensa toolchain, cached pinned
     ESP-IDF, ccache). The warm-cache speedup a persistent runner buys is marginal.
2. **The only thing that justifies a self-hosted runner here is an on-device stage** — flashing a
   build to a real module and exercising it. Hosted runners fundamentally cannot do that.
3. The ovms-modern precedent does **not** transfer cleanly: that repo is **private**, so wiring the
   runner to `pull_request` is safe there. This repo is **public**, where a self-hosted runner on
   `pull_request` means *any PR author runs arbitrary code on the runner* — and that runner can
   flash the live car. See "Security model" below.
4. On-device **feature** testing is feasible and valuable, via **firmware-in-the-loop CAN
   injection**, but it requires a **dedicated bench module** (not the in-car one) and has one
   unverified path (UDS poll replies) to prove out before it covers poll-based features.

## Security model (the load-bearing constraint)

- Trigger must be **`workflow_dispatch` only**. GitHub restricts `workflow_dispatch` to users with
  **write access** to the repo — i.e. the maintainer only. No untrusted PR author can reach it.
  This gives exactly the "only I can trigger it" property without extra gating.
- **Never** wire the self-hosted runner to `pull_request` on this public repo.
- Treat the runner as compromisable by whatever it builds/flashes. It can flash the car's module,
  so isolate it: minimal network reach (only what flashing needs), not the k3s control plane,
  ephemeral if practical.

## Current CI baseline (for reference)

- `.github/workflows/build.yml` — compile-only check on hosted `ubuntu-22.04`. Builds the base
  `NONE` stub plus the vehicle modules under development (`ENABLE_VEHICLES`), not the full set.
  Caches: xtensa toolchain, pinned ESP-IDF fork, ccache. No on-device stage.
- `.github/workflows/docs.yml`, `web-assets-check.yml` — unrelated.
- Keep `build.yml` exactly as-is on hosted runners for `pull_request`. Any on-device work is a
  **separate, manually-triggered workflow** — it does not replace the PR compile check.

## What "test on hardware" actually means in OVMS

The on-device `test` command (`main/test_framework.cpp`) is **not** an automated pass/fail suite.
It is a grab-bag of manual diagnostics and stress tests, several of which are **deliberately
destructive** and must never run in an automated lane on a real module:

- Destructive / crashing: `test watchdog` (fires the task watchdog), `test stackoverflow`
  ("crashes"), `test heapcorruption`, `test sleep` (deep sleep).
- Interactive diagnostics with human-read output, no uniform PASS/FAIL signal: `sdcard`, `cantx`,
  `canrx`, `spiram`, `realloc`, `string`, `filewriter`, `commands`, …

So "run `test` and capture pass/fail" does not map to anything real. Two on-device approaches do.

### Tier 1 — boot smoke test (works with any module, incl. the in-car one)

Assertable, non-destructive signal after a flash:

1. Flash the hosted artifact via the fast path (`ota flash http`, ~51 s — see the deploy notes,
   not scp which is ESP32-bottlenecked to ~4 KB/s).
2. After reboot, over SSH assert:
   - `boot status` → clean boot reason / no crash (catches a build that panic-loops).
   - firmware version metric matches the flashed SHA (confirms the *new* image is running).
   - vehicle module loaded + key metrics go valid.
3. Restore known-good build.

Catches won't-boot, boot-loops, wrong-image-flashed, dead vehicle driver — without running any
crash-on-purpose test.

### Tier 2 — firmware-in-the-loop feature tests via CAN injection (the real prize)

Verified mechanism in the firmware (`components/can/src/can.cpp`):

- **`can rx standard|extended <id> <data…>`** (`can.cpp:1097-1098`, "Simulate reception of a CAN
  frame") pushes a synthetic frame into `can::IncomingFrame()` →
  `ExecuteCallbacks(frame, tx=false, …)` (`can.cpp:1140,1145`) → the active vehicle's
  `IncomingFrameCan1..4` handlers fire **exactly as if the frame arrived off the wire, without
  transmitting on the physical bus.** This is a clean injection primitive.
- **`can play`** (`components/can/src/canplay_vfs.cpp`) replays a whole captured CAN log from
  `/sd`; **`can log`** captures one. Capture formats include crtd, pcap, gvret, panda, lawicel,
  raw (`canformat_*`).

We already have real captures in the companion `solterra-can` repo to use as fixtures.

**Test loop per scenario:** flash candidate build → inject a captured trace (`can rx` sequence or
`can play`) → read resulting metrics back over SSH (`metrics list` / specific names) → diff against
a checked-in **golden JSON** (metric → value/tolerance) → restore known-good.

This is genuine, deterministic feature regression testing ("does this exact charging capture still
decode to the right `v.b.soc` after my refactor?") with no dependency on the car being in any
particular state.

## Decisions taken (2026-06-07)

| Topic | Decision |
|-------|----------|
| Trigger | `workflow_dispatch` only (maintainer-only by GitHub's write-access rule) |
| Build source | On-device job **downloads the hosted-CI artifact** (`ovms3.bin`); the runner only flashes + tests. Keeps the heavy build on free hosted runners. |
| End state | **Re-flash a known-good Solterra build** after the test, so the car is never left unmonitored. (Needs a designated known-good image/URL.) |
| Bench module | Will **acquire a second OVMS3** as an isolated bench unit. Design now, build when hardware arrives. |
| First coverage | **Passive-CAN features** first (drive-input / throttle / brake pins, gateway-relay TPMS) — decode straight from injected frames, no partner responder needed. |

## Why a dedicated bench module (not the in-car one)

Feature injection requires it, for two reasons:

1. **Safety** — injecting test frames into a module plugged into a live vehicle bus is unsafe.
2. **Determinism** — real ECU traffic on the live bus would pollute the test.

The in-car Solterra module stays for real-world validation and (optionally) Tier-1 smoke tests
only. Tier 2 runs on the bench unit, isolated, never connected to the car.

## Open question to validate on hardware (gates poll-based coverage)

Passive-CAN features decode directly from injected frames — `can rx` drives `IncomingFrameCan`.
But much of e-TNGA's data comes from **UDS polling**: the module TX's a request and
`IncomingPollReply` matches the ECU's response. If we inject only the *reply* frame, the poller may
drop it because it has no outstanding request it sent.

**Unverified:** whether an injected reply drives `IncomingPollReply` standalone. Must be checked on
real hardware before designing poll-based feature tests. If it does not, poll-based features need a
**CAN partner responder** on the bench (a second node replaying the ECU side), which is more rig.
Deferred — first coverage is passive-CAN only, which does not hit this.

## Proposed build (when revisited)

Hardware-independent, buildable without the module:

- `onvehicle-test.yml` — `workflow_dispatch`; `runs-on: [self-hosted, ovms-onvehicle]`;
  download-artifact → flash → run harness → restore known-good. Runner/host/SSH/known-good-URL as
  top-of-file env vars.
- Harness script — drives `can rx` / `can play` over SSH, reads metrics, diffs golden JSON, emits
  pass/fail + a log artifact. Reuse the proven wolfSSH SSH options from `scripts/ovms-deploy.sh`
  (`HostKeyAlgorithms=+ssh-rsa`, `PubkeyAcceptedAlgorithms=+ssh-rsa`, legacy `-O` scp).
- Scenario format — one folder per scenario: CAN capture (from `solterra-can`) + inject
  script/command list + `expected.json`. Adding coverage = adding a folder, no harness change.

Hardware-gated (cannot be faked):

- Capturing real golden metric values.
- Confirming inject→decode timing/state.
- The UDS poll-reply injection question above.

## Runner prerequisites (maintainer, one-time, outside the YAML)

- Register a self-hosted runner (repo Settings → Actions → Runners) on a box with the network path
  to the bench module (e.g. on os-k3s / "Old Stone").
- Label it `ovms-onvehicle` so only this workflow targets it.
- Lock it down per "Security model" above.
