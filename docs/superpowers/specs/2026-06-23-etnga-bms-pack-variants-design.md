# e-TNGA BMS — pack-variant–correct cell/temperature arrangement

**Date:** 2026-06-23
**Component:** `vehicle_toyota_etnga` (base — shared by Subaru Solterra / Toyota bZ4X / Lexus RZ)
**Status:** Design approved; supersedes open PRs #130, #131, #132.

## Problem

The e-TNGA per-cell BMS handling is pinned to the **original-generation (2022–24) 96-cell pack**, in the
wrong place (per-badge leaf wrappers) and on the wrong axis (badge, not model year). Battery-pack research
(2026-06-23) documents three distinct e-TNGA HV packs:

| Generation | Cells (series) | Modules × cells/module | Capacity | `0x182E` reply len |
|------------|----------------|------------------------|----------|--------------------|
| 2022–24 (current fleet) | **96** | 4 × 24 | 71.4 kWh (Panasonic) / 72.8 kWh (CATL) | 192 B |
| 2025/26 FWD | **78** | 3 × 26 | 57.7 kWh | 156 B |
| 2025/26 AWD / high-cap | **104** | 4 × 26 | 73.1 kWh | 208 B |

Notes from the research that inform the design:
- Cell **count** and cells-**per-module** both vary across model years (24/module original, 26/module
  refresh). Neither badge nor supplier predicts the pack — model year / generation does.
- The Panasonic original cell is internally a triple jellyroll, but the BMS still reports **96 electrical
  cells**, so the `0x182E` voltage count equals the series cell count. Supplier (Panasonic vs CATL) affects
  charge-power capability, not BMS arrangement; the `2.5–4.3 V` limits cover both chemistries.
- The Solterra cell count is inferred (shares the pack; early Solterra matches the 71.4/72.8 kWh bZ4X spec),
  not separately documented.

### Current behaviour (master + the three open PRs) vs the matrix

- **96-cell:** correct. Decode reads 192 B → 96 cells; declared `4 × 24` grouping; no-op for the auto-detect.
- **104-cell:** count auto-grows correctly (#132), but module grouping stays the declared `24`/module, so
  `/bms/cellmon` would show `4 × 24` + a leftover 8 instead of `4 × 26`. **Data correct, display wrong.**
- **78-cell:** a 156 B reply is rejected by the `m_rxbuf.size() < 192` guard in `etnga_poll_processor.cpp`
  before decode → **no per-cell BMS at all. Broken.**

Two root defects: (1) the fixed `< 192`/`< 48` byte-floor guards reject the smaller pack; (2) module
grouping is hardcoded to the original generation.

## Scope

Be **correct for all three documented packs (78 / 96 / 104)** — "forward-compatible". Out of scope:
undocumented future packs (rejected safely, not guessed), and any change to charge-power / capacity
modelling. "Correct" includes the cosmetic `/bms/cellmon` module grouping, not only the cell data/alerting.

## Approach (A): pack knowledge in the base, count-keyed

### 1. Ownership & structure
- All BMS pack knowledge moves **into the `OvmsVehicleToyotaETNGA` base**.
- The base constructor sets the shared-chemistry defaults once — `BmsSetCellLimitsVoltage(2.5, 4.3)`,
  `BmsSetCellLimitsTemperature(-30, 60)`, `BmsSetCellDefaultThresholdsVoltage(0.020, 0.030)`,
  `BmsSetCellDefaultThresholdsTemperature(4, 8)` — **and** a bootstrap arrangement
  (`BmsSetCellArrangementVoltage(96, 24)`, `BmsSetCellArrangementTemperature(24, 6)`) so the BMS routing
  gate (`BmsGetCellArangementVoltage() > 0`) is open from boot, before the first battery reply resolves the
  real generation.
- The leaf wrappers (`vehicle_subaru_solterra`, `vehicle_toyota_bz4x`, `vehicle_lexus_rz`) **drop their BMS
  blocks** — constructors become just the identifying log line.

**Accepted tradeoff:** per-cell BMS + deviation alerting is now **on by default for every e-TNGA vehicle**,
including the experimental Lexus RZ. This is intended under the forward-compatible scope; it removes the
previous implicit per-wrapper opt-in.

### 2. Count → layout resolution
A single static lookup in the base maps detected **cell count → module count**:

```
78 → 3 modules,   96 → 4 modules,   104 → 4 modules
```

- On a `0x182E` (cell-voltage) reply: `N = decoded cell count`. If `N` is a table key,
  `cellsPerModule = N / modules` (78/3 = 26, 96/4 = 24, 104/4 = 26 — matches spec exactly) and apply via
  `BmsCheckChangeCellArrangementVoltage(N, cellsPerModule)`. Cache the resolved module count in a member
  (e.g. `m_bms_modules`, initialised to `4` to match the bootstrap 96-cell arrangement) for the temperature
  side, so a temperature reply that arrives before the first voltage reply still has a sane module count.
- On a `0x1814` (temperature) reply: `M = decoded sensor count`. Using the cached module count,
  `sensorsPerModule = M / modules` and apply via `BmsCheckChangeCellArrangementTemperature(M, sensorsPerModule)`.
  This derives `6`/module from the 96-cell pack's 24 sensors and self-derives for 78/104 from whatever they
  report — **no temperature-sensor-count data we don't have is required.**
- An **unrecognised** cell count (not in the table) → log a warning and **skip the update**, keeping the
  last good arrangement; never re-arrange to a garbage value.
- Edge case: if `M` does not divide evenly by the module count, fall back to `sensorsPerModule = M` (single
  group) and log; the cell data is still correct, only the display grouping degrades.

`BmsCheckChangeCellArrangement*` is a no-op (returns false) when the count and grouping already match, so a
steady-state pack re-resolves to the same arrangement every poll at no cost.

### 3. Safety guard
- Remove the fixed `m_rxbuf.size() < 192` / `< 48` byte-floor guards in `etnga_poll_processor.cpp` (these
  are what reject the 78-cell pack).
- Replace with: the **whitelist** in §2 (unrecognised count → no re-arrange + log) plus a nonempty /
  even-length sanity check.
- **No hysteresis.** Justification: `IncomingPollReply` dispatches to `IncomingHybridBatterySystem` only when
  `job.mlremain == 0` — i.e. ISOTP has signalled the **complete declared** multi-frame response. A
  mid-transfer truncation never arrives as a short dispatched buffer (it times out → `IncomingPollError`).
  Therefore the dispatched length always equals the ECU's declared whole-reply length, and a 96-cell pack
  cannot be mis-read as a complete 78-cell reply. A count-whitelist is sufficient.

### Net behaviour per pack
- **96-cell (your Solterra / all current fleet):** complete reply declares 96 → resolves to `4 × 24` and
  24 sensors → `6`/module. **Identical to today — a true no-op.**
- **104-cell:** resolves to `4 × 26` (count *and* grouping correct).
- **78-cell:** accepted (guard removed), resolves to `3 × 26`.

## Migration / packaging

This supersedes the three open PRs, reorganised into **two new PRs** off current master; close #130, #131,
#132.

- **PR 1 — e-TNGA base pack-variant BMS** (`feature/etnga-bms-pack-variants`):
  base defaults + bootstrap arrangement (§1), the count→layout resolver and table (§2), the guard change
  (§3); empty the Solterra and bZ4X BMS blocks; carry over the **bZ4X destructor-log fix and the bZ4X +
  e-TNGA doc updates** from #130; `changes.txt` entry. This is upstreamable framework work.
- **PR 2 — experimental Lexus RZ wrapper** (fork-only): the empty `LEXRZ` wrapper (component, Kconfig
  `default n`, CI toggle, `changes.txt`) on top of PR 1, with its BMS block already absent.

## Testing / validation

- **On-device (the only validatable path):** flash PR 1 to the Solterra and confirm the **no-op** — 96 cells,
  24 temperature sensors, 4 modules of 24 + 6, pack voltage sane — i.e. `/bms/cellmon` unchanged from today.
- **78 / 104 packs are NOT validatable here** (no such hardware). Their behaviour is reasoned from spec only
  and MUST be marked as such in code comments, the spec, and `changes.txt`.
- **Compile:** CI (GitHub Actions) with the e-TNGA vehicles enabled.
- **Optional** (decide at planning): a `test` shell sub-command that feeds synthetic 156-/192-/208-byte
  replies through the resolver and asserts the resulting arrangement (3×26 / 4×24 / 4×26), to exercise the
  table logic without the missing hardware. Marked optional / low priority.

## Risks & open items

- 78/104 arrangement is unvalidated (no hardware) — flagged everywhere, accepted.
- The whitelist must be extended when a genuinely new e-TNGA pack count appears; an unknown count degrades
  gracefully (keeps last arrangement, logs) rather than guessing.
- Solterra cell count remains inferred (96), consistent with the matrix; not separately confirmed against a
  Subaru spec sheet.
