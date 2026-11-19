/*
 * test_algorithm.c - PC-side unit test for coupling.c and pid.c.
 * Build with gcc (or clang) on a PC. Run: make test-c
 */
#include <stdio.h>
#include <math.h>
#include "coupling.h"
#include "pid.h"

static int failures = 0;

#define CHECK(cond, msg) do {                                   \
    if (!(cond)) { printf("  FAIL %s\n", msg); failures++; }    \
    else printf("  PASS %s\n", msg);                            \
} while (0)

int main(void)
{
    int i;
    float out = 0.0f;
    PID_t pid;

    printf("[1] coupling\n");
    CHECK(Coupling_ALimit(1000) == 1000, "k=1 limit is 1000");
    CHECK(abs(Coupling_ALimit(500) - 333) <= 1, "k=0.5 limit is 333");
    CHECK(Coupling_SolveB(500, 1000) == 500, "k=1 gives b=a");
    CHECK(Coupling_SolveB(0, 1000) == 0, "zero command gives zero");
    CHECK(Coupling_SolveB(500, 0) == 0, "k=0 does not divide by zero");

    printf("[2] ramp\n");
    {
        int cur = 0, maxStep = 0, prev = 0;
        for (i = 0; i < 60; i++) {
            cur = Coupling_Ramp(cur, 1000, 25);
            if (i > 0 && cur - prev > maxStep) maxStep = cur - prev;
            prev = cur;
        }
        CHECK(maxStep <= 25, "ramp step <= 25 per tick");
        CHECK(cur == 1000, "reaches full speed in 40 ticks");
    }

    printf("[3] PI\n");
    PID_Init(&pid, 0.40f, 1.50f, -1000.0f, 1000.0f);
    for (i = 0; i < 1200; i++) {
        out = PID_Update(&pid, 500.0f - out, 0.005f);
    }
    CHECK(fabsf(out - 500.0f) < 15.0f, "step response converges");

    PID_Init(&pid, 0.30f, 5.0f, -100.0f, 100.0f);
    for (i = 0; i < 500; i++) {
        (void)PID_Update(&pid, 1000.0f, 0.005f);
    }
    CHECK(pid.integ <= 100.0f / 5.0f + 0.001f, "integral anti-windup clamp");

    printf("\n%s (%d failure(s))\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
