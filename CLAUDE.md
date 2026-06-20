# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

OVMS3 (Open Vehicle Monitoring System v3) is firmware for an **ESP32-based** hardware module that
plugs into a vehicle's OBD2 port to monitor, diagnose, and control electric vehicles. It is a
C++ application built on **ESP-IDF 3.3.x** (the legacy `make` build system, *not* `idf.py`/CMake).
The codebase is large and modular: a core framework plus ~100 ESP-IDF components, including one
component per supported vehicle.

The actual firmware lives under `vehicle/OVMS.V3/`. The repo root also holds `docs/` (Sphinx),
`plugins/` (web/JS plugins, not firmware), and `scripts/`.

## Build, flash, deploy

All firmware commands run from `vehicle/OVMS.V3/`. ESP-IDF must be on `IDF_PATH` (the devcontainer
sets this to `/opt/esp-idf` and installs the xtensa toolchain).

```bash
cd vehicle/OVMS.V3

# First time only: seed an sdkconfig from a hardware preset (see support/ for variants).
cp support/sdkconfig.default.hw31 sdkconfig     # hw31 = base module v3.1–3.3; hw30 for v3.0

make menuconfig          # configure: select target vehicle(s) under "OVMS - Vehicle Support", set HW model
make -j$(nproc)          # build -> build/ovms3.bin
make flash               # flash over USB serial (set port in menuconfig)
make monitor             # serial console
make size / make clean
```

The default sdkconfig presets do **not** enable any single vehicle by default — selecting the
vehicle(s) to compile in is a menuconfig step (`CONFIG_OVMS_VEHICLE_*`). The devcontainer
(`.devcontainer/`) is the supported dev environment; the build is also exercised in CI
(`.travis.yml`) which just copies `support/sdkconfig.default.hw31` and runs `make`.

Deploy a freshly built image to a running module over the network (SCP + `ota flash vfs`):

```bash
scripts/ovms-deploy.sh [host] [user]    # defaults via scripts/.env (gitignored); see header for OVMS_* env vars
```

Docs: `cd docs && make html` (Sphinx; requires `sphinx_rtd_theme`, `sphinx_mdinclude`).

There is no unit-test suite to run on a host; "tests" run on-device via the `test` command
(`main/test_framework.cpp`). `tests/` and `*.pl` files are simulation/scratch helpers.

## Architecture

### Global manager singletons

The framework is a set of globally-instantiated managers, each `My<Name>`, declared `extern` in the
matching header and used everywhere. The important ones:

| Singleton | Header | Role |
|-----------|--------|------|
| `MyVehicleFactory` | `components/vehicle/vehicle.h` | Registers/instantiates the active vehicle |
| `MyMetrics` | `main/ovms_metrics.h` | Typed, named, observable data values (the data backbone) |
| `MyConfig` | `main/ovms_config.h` | Persisted config: `[instance] param = value` |
| `MyEvents` | `main/ovms_events.h` | Async event bus (`vehicle.on`, `ticker.1`, …) |
| `MyCommandApp` | `main/ovms_command.h` | Hierarchical shell command tree (SSH/web/serial/server) |
| `MyNotify` | `main/ovms_notify.h` | User-facing notifications (alerts, info, data records) |
| `MyPcpApp` / `MyPowerMgmt` | `components/pcp`, `components/powermgmt` | Power Control Protocol devices / power state |
| `MyBoot`, `MyNetManager`, `MyTime` | `main/` | Boot/crash state, networking, clock |

`StandardMetrics` (`main/metrics_standard.h`, the `StdMetrics`/`MyMetricsStandard` set) defines the
canonical cross-vehicle metric names (`ms_v_bat_soc`, `ms_v_pos_latitude`, …). **Always write vehicle
data into existing standard metrics** rather than inventing new ones, so the apps/server/web UI pick
them up automatically.

### Components & registration

Every feature is an ESP-IDF component under `vehicle/OVMS.V3/components/<name>/`. Each has a
`component.mk` (legacy build), often a `CMakeLists.txt`, and a `Kconfig` fragment surfaced through
`main/Kconfig` so it can be toggled in menuconfig. Components self-register at boot using a
**static init object with an explicit `init_priority`**, e.g.:

```cpp
class OvmsVehicleXyzInit { public: OvmsVehicleXyzInit(); }
  MyOvmsVehicleXyzInit __attribute__ ((init_priority (9000)));
```

This constructor runs before `app_main` and typically calls `MyVehicleFactory.RegisterVehicle<...>()`
or `MyCommandApp.RegisterCommand(...)`. Higher-level components use higher priority numbers.

### Vehicle components — the main extension point

Most work (including recent commits) happens in `components/vehicle_*`. The base class is
`OvmsVehicle` (`components/vehicle/vehicle.h`), a large interface a vehicle overrides selectively:

- **CAN frames:** `IncomingFrameCan1..4(CAN_frame_t*)` for passively-read buses.
- **OBD2/UDS polling:** declare a poll list and implement `IncomingPollReply` / `IncomingPollError`;
  drive poller state with `PollSetState(...)`. See `components/poller/`.
- **Time base:** `Ticker1/10/60/300/600/3600(uint32_t)` periodic hooks.
- **Commands:** override `CommandWakeup`, `CommandClimateControl`, `CommandLock`, `CommandStat`, etc.
  (each returns a `vehicle_command_t`); unimplemented ones report "not supported".
- **CAN setup:** call `RegisterCanBus(bus, mode, speed, ...)` to configure a bus.

Register the class in an init object as above: `MyVehicleFactory.RegisterVehicle<OvmsVehicleXyz>("CODE","Friendly Name")`. The short code (e.g. `"SUBSOL"`) is the user-facing `vehicle module` selector.

**Vehicles can extend other vehicles.** Platform families share a base: e.g.
`OvmsVehicleSubaruSolterra : public OvmsVehicleToyotaETNGA : public OvmsVehicle` — the Solterra
inherits the Toyota e-TNGA polling/CAN logic and overrides only what differs (BMS cell arrangement,
thresholds). When touching one of a shared family, check whether the change belongs in the base.

### BMS (battery) API

Per-cell battery data goes through the `OvmsVehicle` BMS helpers in `components/vehicle/vehicle_bms.cpp`
rather than raw metrics: `BmsSetCellArrangementVoltage/Temperature(readings, perModule)` to declare
pack geometry, then `BmsSetCellVoltage(i, v)` / `BmsSetCellTemperature(i, t)` per reading. This drives
min/max/deviation/alerting automatically.

## Conventions

- **C++ matching ESP-IDF 3.3 / older GCC.** Match the surrounding file's existing style exactly
  (the project explicitly asks contributors to stick to each module's code style).
- **Logging:** `ESP_LOGx(TAG, ...)`; each class defines a `static const char* TAG`.
- **Always pair a PID with its ECU.** A PID (e.g. `0x10D4`) is only meaningful in the context of
  the ECU it's polled on — the same PID number means different things on different ECUs. When
  referencing, discussing, or documenting a PID, always name the ECU/poll target alongside it
  (e.g. "`0x10D4` on the Plug-In Control System ECU"), never the PID alone.
- **User-facing changes** are recorded in `vehicle/OVMS.V3/changes.txt` (note the entry format:
  date/author header, `-` bullets, and a `New configs:` / `Config extension:` sub-block listing any
  added `[instance] param` keys). `knownissues.txt` and `todo.txt` track the rest.
  Per maintainer guidance, `changes.txt` is for changes users need to act on or notice — new
  features, config changes, behavior changes. **Do not add plain bug fixes** unless they require
  the user to change something on their side.
- **PRs are single-purpose** — one vehicle/feature/fix per PR; only mix vehicle + framework changes
  when they depend on each other (see README "A note on pull requests").
- This is a hardware project: most changes can only be fully validated by building and flashing to a
  physical module on a real vehicle/CAN bus. Don't assert behavior you can't verify here.

## Note on AI-generated contributions

The README states firmly that unvalidated AI output and "vibe" submissions are rejected. Treat any
generated code, analysis, or docs as a draft to be verified against real framework behavior before it
is presented as correct — function-call analysis and OVMS-specific patterns are easy to get subtly wrong.
