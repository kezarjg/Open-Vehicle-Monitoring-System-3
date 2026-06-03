// check.h — minimal assertion helpers shared by host test suites.
//
// No external test framework: a tiny CHECK macro plus run/pass counters keeps
// the harness dependency-free (same spirit as the vehicle_vwegolf/tests suite).
// Each test binary defines main() and ends with CHECK_SUMMARY().

#pragma once
#include <cstdio>
#include <cmath>

inline int g_tests_run = 0;
inline int g_tests_passed = 0;

#define CHECK(cond, msg) do {                                           \
    g_tests_run++;                                                      \
    if (cond) { g_tests_passed++; printf("  PASS: %s\n", (msg)); }      \
    else      { printf("  FAIL: %s\n", (msg)); }                        \
  } while (0)

inline bool check_near(float a, float b, float tol = 0.01f) {
    return std::fabs(a - b) < tol;
}

// Returns process exit code: 0 = all passed, 1 = any failure.
#define CHECK_SUMMARY() ([]{                                            \
    printf("\n%d/%d checks passed\n", g_tests_passed, g_tests_run);     \
    return (g_tests_passed == g_tests_run) ? 0 : 1;                     \
  }())
