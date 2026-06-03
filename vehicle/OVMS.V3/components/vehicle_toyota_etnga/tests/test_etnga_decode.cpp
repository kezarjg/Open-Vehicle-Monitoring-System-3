// test_etnga_decode.cpp — native host tests for Toyota e-TNGA decode logic.
//
// Two layers:
//   1. Direct tests of the header-inline GetRxB* byte helpers (pure, no vehicle).
//   2. Public-path tests: construct the vehicle and drive IncomingPollReply()
//      with a synthetic poll_job_t + reply bytes, then assert the resulting
//      metric. (The Calculate*/Set* functions are private, so the faithful entry
//      point is IncomingPollReply — same approach as vehicle_vwegolf/tests.)
//
// Scope is deliberately weighted to stable decode paths (SOC, pack V/I, temps,
// gear, charge-type, odometer, HVAC). The charge-state machine is intentionally
// not covered here while it is under active development.
//
// Run:  make test   (from this directory)

#include "mock_ovms.hpp"
#include "vehicle_toyota_etnga.h"
#include "check.h"

#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static OvmsVehicleToyotaETNGA* make_vehicle() {
    g_metrics = MetricStore{};            // reset all metric state before each test
    return new OvmsVehicleToyotaETNGA();
}

static std::string buf(std::initializer_list<uint8_t> bytes) {
    std::string s;
    for (uint8_t b : bytes) s.push_back(static_cast<char>(b));
    return s;
}

// Drive a complete single-frame poll reply through the public entry point.
static void feed(OvmsVehicleToyotaETNGA* v, uint32_t module_rx, uint16_t pid,
                 std::initializer_list<uint8_t> bytes) {
    std::vector<uint8_t> data(bytes);
    OvmsPoller::poll_job_t job{};
    job.moduleid_rec = module_rx;
    job.pid          = pid;
    job.mlframe      = 0;
    job.mlremain     = 0;
    v->IncomingPollReply(job, data.data(), static_cast<uint8_t>(data.size()));
}

// ---------------------------------------------------------------------------
// 1. GetRxB* byte-extraction helpers (pure, big-endian)
// ---------------------------------------------------------------------------
void test_getrxb_helpers() {
    printf("\ntest_getrxb_helpers\n");
    CHECK(GetRxBByte(buf({0xAB}), 0) == 0xAB,                 "GetRxBByte");
    CHECK(GetRxBUint16(buf({0x12, 0x34}), 0) == 0x1234,       "GetRxBUint16 big-endian");
    CHECK(GetRxBUint32(buf({0x00, 0x01, 0x86, 0xA0}), 0) == 100000u, "GetRxBUint32 big-endian");
    CHECK(GetRxBInt16(buf({0xFF, 0xFF}), 0) == -1,            "GetRxBInt16 two's-complement");
    CHECK(GetRxBInt8(buf({0xFF}), 0) == -1,                   "GetRxBInt8 two's-complement");
    CHECK(GetRxBBit(buf({0x02}), 0, 1) == true,              "GetRxBBit set");
    CHECK(GetRxBBit(buf({0x02}), 0, 0) == false,             "GetRxBBit clear");
}

// ---------------------------------------------------------------------------
// 2. Public-path decode tests via IncomingPollReply
// ---------------------------------------------------------------------------
void test_battery_soc() {
    printf("\ntest_battery_soc\n");
    auto* v = make_vehicle();
    feed(v, PLUG_IN_CONTROL_SYSTEM_RX, PID_BATTERY_SOC, {0x55});   // 0x55 = 85
    CHECK(check_near(StandardMetrics.ms_v_bat_soc->AsFloat(), 85.0f), "SOC 85% from 0x55");
    delete v;
}

void test_battery_voltage_current_power() {
    printf("\ntest_battery_voltage_current_power\n");
    auto* v = make_vehicle();
    // voltage = u16[2..3]/64 ; current = s16[4..5]/10 ; power = V*I/1000
    // 0x5780=22400 -> 350.0 V ; 0x0064=100 -> 10.0 A ; 3.5 kW
    feed(v, HYBRID_CONTROL_SYSTEM_RX, PID_BATTERY_VOLTAGE_AND_CURRENT,
         {0x00, 0x00, 0x57, 0x80, 0x00, 0x64});
    CHECK(check_near(StandardMetrics.ms_v_bat_voltage->AsFloat(), 350.0f), "pack voltage 350 V");
    CHECK(check_near(StandardMetrics.ms_v_bat_current->AsFloat(), 10.0f),  "pack current 10 A");
    CHECK(check_near(StandardMetrics.ms_v_bat_power->AsFloat(),   3.5f),   "pack power 3.5 kW");
    delete v;
}

void test_ambient_temperature_ev() {
    printf("\ntest_ambient_temperature_ev\n");
    auto* v = make_vehicle();
    feed(v, HYBRID_CONTROL_SYSTEM_RX, PID_AMBIENT_TEMPERATURE_EV, {60});  // 60-40 = 20
    CHECK(check_near(StandardMetrics.ms_v_env_temp->AsFloat(), 20.0f), "ambient (EV) 20 C");
    delete v;
}

void test_cabin_temperature() {
    printf("\ntest_cabin_temperature\n");
    auto* v = make_vehicle();
    feed(v, AIR_CONDITIONER_RX, PID_CABIN_TEMPERATURE, {0x07, 0xD0});  // 2000/100 = 20.0
    CHECK(check_near(StandardMetrics.ms_v_env_cabintemp->AsFloat(), 20.0f), "cabin temp 20 C");
    delete v;
}

void test_shift_position() {
    printf("\ntest_shift_position\n");
    auto* v = make_vehicle();
    feed(v, HYBRID_CONTROL_SYSTEM_RX, PID_SHIFT_POSITION, {6});   // Drive
    CHECK(StandardMetrics.ms_v_env_gear->AsInt() == 1,  "Drive -> gear 1");
    feed(v, HYBRID_CONTROL_SYSTEM_RX, PID_SHIFT_POSITION, {2});   // Reverse
    CHECK(StandardMetrics.ms_v_env_gear->AsInt() == -1, "Reverse -> gear -1");
    feed(v, HYBRID_CONTROL_SYSTEM_RX, PID_SHIFT_POSITION, {0});   // Park
    CHECK(StandardMetrics.ms_v_env_gear->AsInt() == 0,  "Park -> gear 0");
    delete v;
}

void test_charge_type_raw_zero() {
    // Regression lock for the v.c.type raw-0 fix: 0x00 must clear (not "ccs").
    printf("\ntest_charge_type_raw_zero\n");
    auto* v = make_vehicle();
    feed(v, PLUG_IN_CONTROL_SYSTEM_RX, PID_CHARGING_VOLTAGE_TYPE, {0x00});
    CHECK(StandardMetrics.ms_v_charge_type->AsString() == "", "charge type raw 0 -> cleared");
    feed(v, PLUG_IN_CONTROL_SYSTEM_RX, PID_CHARGING_VOLTAGE_TYPE, {0x01});
    CHECK(StandardMetrics.ms_v_charge_type->AsString() == "type1", "charge type raw 1 -> type1");
    delete v;
}

void test_hvac_setpoint_piecewise() {
    printf("\ntest_hvac_setpoint_piecewise\n");
    auto* v = make_vehicle();
    feed(v, AIR_CONDITIONER_RX, PID_HVAC_SETPOINT, {0});    // special-cased 0
    CHECK(check_near(StandardMetrics.ms_v_env_cabinsetpoint->AsFloat(), 0.0f),  "HVAC setpoint raw 0 -> 0");
    feed(v, AIR_CONDITIONER_RX, PID_HVAC_SETPOINT, {20});   // <28: 20/2 + 15.5 = 25.5
    CHECK(check_near(StandardMetrics.ms_v_env_cabinsetpoint->AsFloat(), 25.5f), "HVAC setpoint raw 20 -> 25.5 C");
    delete v;
}

void test_odometer() {
    printf("\ntest_odometer\n");
    auto* v = make_vehicle();
    feed(v, HYBRID_CONTROL_SYSTEM_RX, PID_ODOMETER, {0x00, 0x01, 0x86, 0xA0});  // 100000/10
    CHECK(check_near(StandardMetrics.ms_v_pos_odometer->AsFloat(), 10000.0f, 0.5f), "odometer 10000 km");
    delete v;
}

int main() {
    printf("=== vehicle_toyota_etnga host decode tests ===\n");
    test_getrxb_helpers();
    test_battery_soc();
    test_battery_voltage_current_power();
    test_ambient_temperature_ev();
    test_cabin_temperature();
    test_shift_position();
    test_charge_type_raw_zero();
    test_hvac_setpoint_piecewise();
    test_odometer();
    return CHECK_SUMMARY();
}
