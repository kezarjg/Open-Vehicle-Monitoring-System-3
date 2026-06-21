# e-TNGA 12V aux-battery telemetry — complete the "12V Battery History" chart

**Date:** 2026-06-19
**Component:** `vehicle_toyota_etnga` (base — shared by Solterra / bZ4X / Lexus RZ)
**Branch:** `feature/etnga-12v-aux-health` (rebased onto current master `ee1c46e3`)

> **2026-06-21 update — live current source resolved.** The current DID was originally
> `0x15FD`, which on-vehicle testing (2026-06-19) found to be **static** (frozen `0x5D00`,
> see fork issue #117); the current metric + `charging12v`/`aux12v` flags were dropped pending
> a confirmed source. solterra-can then pinned the **live** aux/12V current at **`0x15F7`** on
> the same EV ECU (`0x7D2`) by watching which DID the Techstream app composes into its dynamic
> slot (commit `ebe6fc2`). This revision re-adds current + the two flags using `0x15F7` with a
> biased-32768 decode. See the validation caveat at the end.

## Problem

The OVMS "12V Battery History" chart (OVMS app / server, fed by the server-v2 `D` record)
carries five 12V fields: **voltage, voltage-ref, current, `charging12v` flag, `aux12v` flag.**
On e-TNGA today only voltage (from the module's onboard ESP32 ADC, via housekeeping) and the
auto-set ref are populated. So the chart's **current trace is flat and the charging indicator
never lights.**

The car exposes a full 12V telemetry set on the **EV ECU (`0x7D2`/`0x7DA`, "F45 Hybrid Vehicle
Control")**, bus-verified in the `solterra-can` project. We want to populate the chart and add
the useful aux-battery health readouts ("Approach A — headline health").

## Goal

Populate the standard chart fields **and** add headline 12V health metrics, on the **base
e-TNGA component** so all platform variants inherit it.

### In scope

| Metric | Type | Source DID (EV ECU `0x7D2`) | Decode |
|--------|------|------------------------------|--------|
| `v.b.12v.current` | standard | `0x15F7` | `(u16BE − 0x8000) × 0.0038147` A (biased-32768, bidirectional) |
| `v.e.charging12v` | standard flag | derived | `v.b.12v.current > 0.5 A` |
| `v.e.aux12v` | standard flag | derived | true on any valid 12V reply; false on entering SLEEP |
| `xte.v.b.12v.voltage` | custom | `0x15EE` | `u16BE × 5/4096` V (high-res; **does NOT** overwrite ADC `v.b.12v.voltage`) |
| `xte.v.b.12v.temp` | custom | `0x15F8` | `(u16BE − 400) × 0.1` °C |
| `xte.v.b.12v.cac` | custom | `0x15E5` | `u8 × 0.5` Ah (12V SoH proxy) |
| `xte.v.b.12v.charge.ah` | custom | `0x15E8` bytes 1–4 | `u32BE × 0.1` Ah (lifetime in) |
| `xte.v.b.12v.discharge.ah` | custom | `0x15E8` bytes 5–8 | `u32BE × 0.1` Ah (lifetime out) |
| `xte.v.b.12v.readyon.h` | custom | `0x15E8` bytes 11–12 | `u16BE` hours |

The current/voltage/temp/cac path and decode helpers (`CalculateAux12v*`) **already exist** on
the branch (commits `3b3a43a1`, `91a7f2e6`, `9bff2261`) and are correct — they are reused, not
rewritten. **New work** is only: (a) the two flags, (b) the `0x15E8` integrator DID + decode.

### Out of scope (YAGNI)

- Full `0x15E8` decode (capacity-after-IG, long-term-leaving count, thermal load) and the
  `0x15EA` 5-trip history bundle — slow, proprietary-unit values nothing charts.
- Custom web panel — the standard chart fills from `v.b.12v.current` + flags; the `xte.*` extras
  are visible in the metrics list / app without a dedicated page.
- Any change to how `v.b.12v.voltage` is sourced — it stays the ESP32 ADC (housekeeping). Polling
  `0x15EE` into it would fight housekeeping and is useless as a sleep wake-trigger.

## Why rebase (not fresh / not finish-as-is)

The branch is based on master `92716dc`; current master is `7d4bc1b`. The 3 commits touch only
6 files (`changes.txt`, `etnga_metrics.cpp`, `etnga_poll_processor.cpp`, `etnga_poll_states.cpp`,
`vehicle_toyota_etnga.cpp`, `vehicle_toyota_etnga.h`), ~83 net lines. Everything else in the stale
`master..HEAD` diff is just the branch being behind (smarteq/vfs/partitions/charge-report drift),
which the rebase resolves automatically. Rebasing keeps the validated decode work and its history.

### Rebase plan & expected conflicts

Rebase `feature/etnga-12v-aux-health` onto `master`. Conflicts are confined to the 6 touched files:

1. **`vehicle_toyota_etnga.cpp` — the one real fixup.** The branch adds 4 poll rows in the **old
   7-column** cadence format (`{S,A,D,HS,W,AC,DC}`). Master now uses the **offset-block** layout
   (PR #106): `obdii_polls_base` (offset 0, columns `{SLEEP, AWAKE, DRIVING, -}`) and a charge
   block (offset 3, columns `{HANDSHAKE, WAIT, AC, DC}`). The 4 rows (plus the new `0x15E8` row)
   must be **re-expressed** as base + charge entries — see cadence table below.
2. **`etnga_metrics.cpp`, `vehicle_toyota_etnga.h`** — context drift from the metric-naming
   cleanup (PR #108) and review fixes (#104/#105). Re-apply the metric inits / PID defines /
   member pointers against current surroundings. Keep the existing `xte.*` names (already
   convention-compliant).
3. **`etnga_poll_processor.cpp`** — re-apply the `IncomingPollReply` dispatch cases alongside
   master's length-guard / `IncomingPollError`-tracing changes.
4. **`etnga_poll_states.cpp`** — re-apply the "clear PID metrics when car off" block; add
   `v.e.aux12v = false` on SLEEP entry here.
5. **`changes.txt`** — trivial append conflict.

## Poll-list design (current offset-block format)

EV-ECU (`HYBRID_CONTROL_SYSTEM_TX/RX` = `0x7D2`/`0x7DA`) polls. **Match the established e-TNGA
pattern: every existing EV-ECU poll runs in DRIVING + charge states, never bare AWAKE** (strong
evidence the EV ECU is unresponsive when merely awake — avoids `IncomingPollError` noise and
CAN2-wedge risk). The stale branch polled these in AWAKE@30s; that column is dropped pending
validation (see Open question).

Base block columns `{SLEEP, AWAKE, DRIVING, -}`; charge block `{HANDSHAKE, WAIT, AC, DC}`:

| PID | base `{S,A,D,-}` | charge `{HS,W,AC,DC}` | rationale |
|-----|------------------|------------------------|-----------|
| `0x15FD` current | `{0,0,10,0}` | `{0,0,10,10}` | live during drive + AC/DC charge (charge = where current matters most) |
| `0x15EE` voltage | `{0,0,30,0}` | `{0,0,30,30}` | diagnostic compare vs ADC |
| `0x15F8` temp | `{0,0,120,0}` | `{0,0,120,120}` | slow |
| `0x15E5` cac | `{0,0,120,0}` | `{0,0,120,120}` | health, slow |
| `0x15E8` integrators | `{0,0,120,0}` | `{0,0,120,120}` | health, slow; **multi-frame** (17 B) |

Cadences are tunable; values chosen to keep added bus load modest. No NSTATES change — purely
additive rows in the existing blocks.

## Decode & flag logic

In `etnga_poll_processor.cpp` `IncomingPollReply`, add dispatch cases (reuse the `CalculateAux12v*`
helpers; add one for `0x15E8`). Length-guard every field before reading, per existing convention:

- `0x15FD` (≥2 B): set `v.b.12v.current`; set `v.e.charging12v = (current > 0.5)`.
- `0x15EE` (≥2 B): set `xte.v.b.12v.voltage`.
- `0x15F8` (≥2 B): set `xte.v.b.12v.temp`.
- `0x15E5` (≥1 B): set `xte.v.b.12v.cac`.
- `0x15E8` (≥12 B, **multi-frame** via the existing ISO-TP reassembly path used by `0x182E`/`0x1814`):
  set `xte.v.b.12v.charge.ah` (bytes 1–4), `xte.v.b.12v.discharge.ah` (bytes 5–8),
  `xte.v.b.12v.readyon.h` (bytes 11–12). (Byte numbering is 1-indexed per the solterra-can doc,
  i.e. payload `data[0..]` after the service+DID echo.)
- `v.e.aux12v`: set **true** on any valid 12V reply (handler); set **false** on entering SLEEP
  (`etnga_poll_states.cpp`, alongside the existing PID-metric clear).

## Error handling

Reuse the established e-TNGA guards: bounds/length-check before every field read; rely on the
poller's multi-frame accumulator for `0x15E8`; no SLEEP/AWAKE polling of the EV ECU. Additive
poll rows only — no change to the per-state cadence design or NSTATES.

## Surfacing

- **`changes.txt`** — the branch already has a 12V entry; update it to reflect the final metric
  set (populated `v.b.12v.current` + `charging12v`/`aux12v` flags; new `xte.v.b.12v.*`). No new
  config keys.
- **`components/vehicle_toyota_etnga/docs/index.rst`** — add the 5 PIDs (`0x15F7/EE/F8/E5/E8`) to
  the PID table, each paired with the EV ECU (`0x7D2`).

## Validation

- CI (GitHub Actions) build green.
- On-vehicle: `v.b.12v.current` reads a plausible small idle current (≈ +2–3 A at float, matching
  the 2026-06-21 Techstream readout) and **varies with 12V load** (the key check — see caveat);
  `v.e.charging12v` lights when the DC-DC is charging and clears when not; the app/server 12V
  chart shows the current trace + charging flag; `xte.v.b.12v.temp/cac/charge.ah/discharge.ah/
  readyon.h` plausible; `xte.v.b.12v.voltage` ≈ ADC `v.b.12v.voltage` (cross-check — already
  confirmed agreeing to 0.01 V at 14.07 V, 2026-06-19). No new `IncomingPollError` in the added
  states; no CAN2 wedge regression.

### ⚠️ Validation caveat — direct-read of `0x15F7` not yet confirmed

solterra-can pinned `0x15F7` by capturing it **listen-only while Techstream had it defined in a
`0x2C` dynamic slot (`F301`)**. We have **not** yet confirmed that a plain OVMS direct read
(`22 15 F7`, which is how the poll list reads it) returns *live* data — this is exactly the trap
`0x15FD` fell into (decode correct, but the ECU served a frozen value). So the **must-pass** check
is that `v.b.12v.current` is not static: watch it while toggling a 12V load (HVAC / headlights) or
across an ignition/charge transition. If it reads frozen, the fallback is to replicate Techstream's
`0x2C` dynamic-DID define and read the value out of `F301` rather than direct `22 15 F7`. Until this
varies on-vehicle, treat the current trace + `charging12v` flag as **provisional**.

## Open question (for validation, not blocking design)

Does the EV ECU answer `0x15xx` reads in **AWAKE** (car parked but woken, HV off)? The stale
branch assumed yes (polled AWAKE@30s); the established pattern says no. Design ships **DRIVING +
charge only**; if on-vehicle testing shows the ECU responds in AWAKE without errors, add an AWAKE
column to the current/voltage rows so the chart also fills while parked-and-awake.
