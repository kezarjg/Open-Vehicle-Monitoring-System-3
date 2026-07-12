# e-TNGA: derive `v.e.charging12v` from the 12V rail voltage

**Issue:** [#147](https://github.com/kezarjg/Open-Vehicle-Monitoring-System-3/issues/147)
**Date:** 2026-07-11
**Scope:** `components/vehicle_toyota_etnga` only — fork-local, no framework changes.

## Problem

A healthy 12V aux battery raises a false `12V Battery critical` alert, and because the alert cannot
self-clear, `powermgmt` deep-sleeps the module 30 minutes later. Observed on the live Solterra
2026-07-11; the module was ~10 minutes from going dark.

The battery is fine. The *reference* it is measured against is wrong:

| metric | value |
|---|---|
| `v.b.12v.voltage` | 12.27 V — a normal resting aux battery |
| `v.b.12v.voltage.ref` | **14.09 V** — a DC-DC *float* voltage, not a resting one |
| `v.b.12v.voltage.alert` | yes (latched) |

`vehicle.cpp:975` alerts when `max(ref, 12.6) − volt > 1.6`. With the bogus reference,
`14.09 − 12.27 = 1.82` → fires. Against a correct 12.6 V reference it would be `0.33` — nowhere near.

### Root cause

The base class samples the reference when its "calmdown" ticker expires (`vehicle.cpp:947-961`). The
ticker accumulates while `v.e.charging12v` is true and drains when it is false; on reaching zero it
takes `ref = current 12V voltage`. **The design assumes `charging12v == false` implies the rail is at
rest.**

e-TNGA breaks that assumption (`etnga_metrics.cpp:556`):

```cpp
StandardMetrics.ms_v_env_charging12v->SetValue(v > 0.5f);   // v = DC-DC current, from CAN
```

Two independent routes to a bad reference follow:

1. **Current taper.** Once the aux battery tops up mid-drive, DC-DC current falls below 0.5 A *while
   the rail is still held at ~14.1 V*. The flag flaps false/true every 11–22 s (visible in the log as
   rapid `Charging 12V battery..` / `No longer charging..` pairs), which can drain the ticker to zero
   mid-drive → the reference is sampled at float voltage.

2. **CAN outage (what actually happened).** `charging12v` is CAN-derived. On 2026-07-11 the MCP2515 was
   left deaf for an entire drive (#148), so the flag could never become true while the car ran at
   14.1 V. The ticker sat at zero and the reference was sampled at float voltage.

The corruption is self-reinforcing: the 12V wake trigger (`etnga_poll_states.cpp:76`) fires on
`v12 > ref + 0.2`. With `ref = 14.09` that needs >14.29 V — essentially never — so a bad reference also
disables the fallback that is supposed to wake the module and reset CAN when the bus is dead.

## Key insight

`v.b.12v.voltage` is read from the **module's own ADC** (`ovms_housekeeping.cpp:90`,
`MyPeripherals->m_esp32adc->read()`). It is the one signal in this system that does **not** come over
CAN, so it stays truthful when the bus dies. `v.e.charging12v` and `v.e.on` are both CAN-derived and go
silently false in exactly the situation we must survive.

Therefore the flag should be derived from the rail voltage, not from a CAN-sourced current.

## Design

### The rule

A resting lead-acid aux battery never exceeds ~13 V. Anything higher means something — the DC-DC — is
holding it up. So: *the rail being above resting **is** the definition of "charging".*

New method, called at the top of `OvmsVehicleToyotaETNGA::Ticker1()`:

```cpp
static const float AUX_12V_CHARGING_ON_V  = 13.2f;  // rail above resting -> DC-DC is charging
static const float AUX_12V_CHARGING_OFF_V = 12.9f;  // rail back at rest  -> not charging

void OvmsVehicleToyotaETNGA::UpdateCharging12v()
{
    float v = StandardMetrics.ms_v_bat_12v_voltage->AsFloat();
    if (v <= 0.0f)  // ADC not ready yet
        return;
    bool charging = StandardMetrics.ms_v_env_charging12v->AsBool();
    if (!charging && v > AUX_12V_CHARGING_ON_V)
        charging = true;
    else if (charging && v < AUX_12V_CHARGING_OFF_V)
        charging = false;
    StandardMetrics.ms_v_env_charging12v->SetValue(charging);
}
```

It runs every second regardless of poll state, CAN health, or whether the vehicle is awake — which is
the entire point.

### Ordering

`OvmsVehicle::VehicleTicker1()` calls the virtual `Ticker1()` at `vehicle.cpp:875` and then runs the
base 12V monitor at `vehicle.cpp:947`, in the same tick. Updating the flag inside e-TNGA's `Ticker1()`
means the base monitor sees the fresh value immediately. No new hook is required.

### Single owner

`charging12v` currently has three writers. After this change the `Ticker1` rule is its only owner:

| Site | Change |
|---|---|
| `etnga_metrics.cpp:556` (`SetAux12vCurrent`) | Stop setting `charging12v`. Keep setting `v.b.12v.current` and `v.e.aux12v`. |
| `etnga_poll_states.cpp:384` (→ SLEEP) | Stop setting `charging12v` false. Keep clearing the PID-sourced metrics (`v.b.12v.current`, `v.e.aux12v`, PID voltage/temp). |
| `etnga_poll_states.cpp:414` (→ AWAKE) | Same as above. |

The two transition-handler writes are not merely redundant under the new rule, they are **actively
wrong**: entering AWAKE while the car is running at 14 V would force the flag false and re-open the bug.

### Thresholds

Hardcoded constants next to the existing `AUX_12V_WAKE_SET_V` / `AUX_12V_WAKE_CLEAR_V`, matching the
file's style. Not configuration — YAGNI. 13.2 / 12.9 V sits comfortably between a fully-charged resting
lead-acid (~12.6–12.8 V) and DC-DC float (~14.0–14.4 V); the 0.3 V hysteresis prevents chatter as the
rail decays after shutdown.

## Behaviour after the fix

| Situation | Rail (ADC) | `charging12v` | Reference sampled? |
|---|---|---|---|
| Driving, DC-DC current tapered below 0.5 A | ~14.1 V | **true** | No — ticker stays charged |
| Driving, **CAN bus dead** | ~14.1 V | **true** | No — ADC is unaffected by CAN |
| AC/DC HV charging (DC-DC also charges aux) | ~14 V | true | No | ← **WRONG, see below** |
| Parked, car off | ~12.3 V | false | Yes — at genuine resting voltage ✅ |

> **Correction (2026-07-12, measured on the car — issue #153).** Two rows of the table above are wrong,
> and the error is the same in both: this design assumed the rail cleanly separates "charging" from
> "resting". It does not.
>
> - **HV charging is NOT ~14 V.** The DC-DC floats the aux battery at only **~12.85 V** (current
>   +2.64 A tapering to +0.64 A, positive throughout a 3 h charge) — *below* `AUX_12V_CHARGING_OFF_V`.
>   So `charging12v` reads **false** for an entire charge while the battery is demonstrably being charged.
> - **Driving is not a steady float either.** The DC-DC modulates; the rail was measured at **12.86 V**
>   mid-drive, which makes the flag flap.
>
> Note 12.86 V occurs while HV-charging *and* while driving, and both sit inside a healthy battery's
> resting band (12.6–12.8 V) — so **no pair of voltage thresholds can separate these states.**
> The fix is a union (rail ∥ `v.e.on` ∥ CHARGE_AC/CHARGE_DC), not a retune.
>
> The safety invariant this spec relies on is unaffected: `ref` is still only ever sampled while
> `charging12v` is false, so it can never latch a charging voltage.

The currently-corrupted persisted reference **self-heals** on the first drive-and-park cycle after this
ships: `charging12v` goes true during the drive, charging the calmdown ticker; on parking it drains and
the base re-samples `ref` at true resting voltage, overwriting 14.09 V.

Fixing `ref` also restores the 12V wake trigger, which the bad reference had disabled.

## Out of scope (deliberate)

- **Base-class `vehicle.cpp` reference logic is untouched.** Every other vehicle keeps its current
  behaviour. Hardening the base so *no* vehicle can latch a charging voltage as a resting reference is a
  real and separate improvement, but it is a framework change and belongs in its own upstream PR.
- **The 12V wake trigger's dependence on `ref`** (`etnga_poll_states.cpp:76`) is left as-is. It works
  correctly once `ref` is sane; decoupling it is a separate change.
- **#148** (the MCP2515 failure that caused the CAN outage) is tracked separately. This fix makes the
  12V reference survive such an outage; it does not prevent one.

## Verification

There is no host test suite; firmware compiles in CI and behaviour is validated on the vehicle.

1. **CI:** `Firmware build` green on the PR.
2. **On-vehicle, one drive-and-park cycle:**
   - During the drive: `v.e.charging12v` is `yes` continuously, and the log shows **no**
     `Charging 12V battery..` / `No longer charging 12V battery..` flapping.
   - After parking and the calmdown expiry: `v.b.12v.voltage.ref` lands at resting voltage
     (~12.3–12.7 V), **not** ~14 V.
   - `v.b.12v.voltage.alert` remains `no`.
   - No `12V Battery critical` notification; no `powermgmt: 12V battery alert detected` in the log.
3. **Regression check:** the module still wakes on the 12V trigger
   (`Aux 12V has exceeded the threshold`) when the car powers on.

## Note on the interim workaround

The live module's reference was manually corrected to 12.6 V on 2026-07-11
(`metrics set v.b.12v.voltage.ref 12.6`), which cleared the latched alert and cancelled the pending
deep-sleep. That is a one-off repair of the *value*, not the bug; without this fix the reference can
re-corrupt on any drive.
