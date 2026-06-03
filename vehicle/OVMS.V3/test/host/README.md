# Host (native) test harness

Compile a vehicle module's real `.cpp` against a **mock OVMS framework** and run
its decode/dispatch logic on a laptop or CI runner — no ESP-IDF, no xtensa
toolchain, no hardware. Fast (seconds) and reproducible.

This is *not* an emulator. The ESP32 QEMU fork does not emulate the TWAI/CAN
controller (a vehicle module's entire input) and targets ESP-IDF 4.4+, so it
can't exercise this code; driving the module's `IncomingFrameCan*` /
`IncomingPollReply` entry points directly with synthetic frames covers more of
the real path at a fraction of the cost. Anything genuinely hardware-bound (CAN
driver, timing, RTOS) still needs the on-device `test` command.

## Layout

```
test/host/
  mock/mock_ovms.hpp   force-included framework mock (metrics, poller, CAN, base class)
  mock/mock_ovms.cpp   one definition of the mock globals + helpers
  mock/*.h             stub redirects for ovms_log.h / ovms_config.h / vehicle.h
  check.h              CHECK() macro + run/pass counters (no external framework)
```

Per-vehicle suites live in `components/vehicle_<name>/tests/` with their own
`Makefile` + `test_*.cpp`, and reuse this shared mock via `-I../../../test/host`.

## Running

```bash
cd vehicle/OVMS.V3/components/vehicle_toyota_etnga/tests
make test          # compiles the module + mock, runs the suite
```

CI runs every `components/*/tests/` suite automatically
(`.github/workflows/host-tests.yml`).

## Extending the mock

The mock only covers the framework surface the tested modules actually touch.
When a new vehicle (or new code in an existing one) references a metric, base
method, or constant the mock lacks, add it here rather than forking a per-vehicle
copy — keep this the single shared mock. Stubs return benign defaults; add real
behaviour only where a test depends on it.

## Writing a suite

`Calculate*`/`Set*` decode functions are typically private, so drive the public
entry points (`IncomingPollReply`, `IncomingFrameCan*`) with synthetic data and
assert the resulting `StandardMetrics`. Reset state between tests by reassigning
`g_metrics = MetricStore{}` and constructing a fresh vehicle. See
`vehicle_toyota_etnga/tests/test_etnga_decode.cpp` for a worked example.
