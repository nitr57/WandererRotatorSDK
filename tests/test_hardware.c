/* *******************************************************************************
 * MIT License
 *
 * Copyright (c) 2025-2026 Nico Trost
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * **************************************************************************** */

/* SDK checks against a real rotator (the first one found by WRRotatorScan).
 *
 * WARNING: this moves the rotator - by up to 450 degrees in one go, so the
 * rotator must be able to turn freely (no cables that can wrap). Every test
 * returns to the starting position. WRRotatorSyncPosition is not tested, as it
 * would overwrite the zero position stored on the device.
 *
 * Usage: test_hardware [-y]   (-y skips the confirmation prompt) */
#include "WandererRotatorSDK.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static int g_fail = 0;
#define CHECK(cond, ...) do { int ok_ = (cond); printf(ok_ ? "  PASS  " : "  FAIL  "); printf(__VA_ARGS__); printf("\n"); if (!ok_) g_fail++; } while (0)

static double Now(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

static WR_ROTATOR_STATUS St(int id) { WR_ROTATOR_STATUS s; memset(&s, 0, sizeof s); WRRotatorGetStatus(id, &s); return s; }

static int WaitIdle(int id, double sec)
{
    double end = Now() + sec;
    while (Now() < end) { if (!St(id).moving) return 1; usleep(50000); }
    return 0;
}

/* signed angular difference a-b in (-180, 180] */
static double Diff(double a, double b) { double d = fmod(a - b + 540.0, 360.0) - 180.0; return d; }

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    if (!(argc > 1 && strcmp(argv[1], "-y") == 0))
    {
        printf("This test moves the rotator by up to 450 degrees. Continue? [y/N] ");
        int c = getchar();
        if (c != 'y' && c != 'Y')
            return 1;
    }

    char ver[WR_VERSION_LEN];
    WRGetSDKVersion(ver);
    printf("SDK %s\n", ver);

    int ids[WR_MAX_NUM], n = 0;
    double t0 = Now();
    WR_ERROR_TYPE r = WRRotatorScan(&n, ids);
    printf("scan: result=%d devices=%d in %.2fs\n", r, n, Now() - t0);
    if (r != WR_SUCCESS || n < 1) return 1;
    int id = ids[0];

    t0 = Now();
    CHECK(WRRotatorOpen(id) == WR_SUCCESS, "open (%.2fs)", Now() - t0);

    WR_VERSION v; WRRotatorGetVersion(id, &v);
    WR_ROTATOR_CONFIG cfg; memset(&cfg, 0, sizeof cfg); WRRotatorGetConfig(id, &cfg);
    WR_ROTATOR_STATUS s = St(id);
    double start = s.position;
    printf("model=%s firmware=%u reverse=%d backlash=%.2f position=%.3f stepSize=%.6f\n",
           v.model, v.firmware, cfg.reverseDirection, cfg.backlash, start, s.stepSize);

    printf("\n[move +5 / -5]\n");
    t0 = Now();
    CHECK(WRRotatorMove(id, 5.0f) == WR_SUCCESS, "move +5 returned in %.2fs", Now() - t0);
    CHECK(St(id).moving, "moving reported");
    t0 = Now();
    CHECK(WaitIdle(id, 60), "completed in %.2fs", Now() - t0);
    CHECK(fabs(Diff(St(id).position, start + 5)) < 0.05, "position %.3f ~ start+5", St(id).position);
    WRRotatorMove(id, -5.0f); WaitIdle(id, 60);
    CHECK(fabs(Diff(St(id).position, start)) < 0.05, "back at start: %.3f", St(id).position);

    printf("\n[overshoot: two back-to-back segments]\n");
    WR_ROTATOR_CONFIG oc; memset(&oc, 0, sizeof oc);
    oc.mask = MASK_ROTATOR_OVERSHOOT | MASK_ROTATOR_OVERSHOOT_ANGLE | MASK_ROTATOR_OVERSHOOT_DIRECTION;
    oc.overshoot = 1; oc.overshootAngle = 1.0f; oc.overshootDirection = 0;
    WRRotatorSetConfig(id, &oc);
    t0 = Now();
    WRRotatorMove(id, 5.0f);
    CHECK(WaitIdle(id, 60), "move +5 with 1 deg overshoot completed in %.2fs", Now() - t0);
    CHECK(fabs(Diff(St(id).position, start + 5)) < 0.05, "position %.3f ~ start+5", St(id).position);
    WRRotatorMove(id, -5.0f); WaitIdle(id, 60);
    CHECK(fabs(Diff(St(id).position, start)) < 0.05, "back at start: %.3f", St(id).position);
    oc.overshoot = 0; oc.mask = MASK_ROTATOR_OVERSHOOT;
    WRRotatorSetConfig(id, &oc);

    printf("\n[stop during a +20 move]\n");
    WRRotatorMove(id, 20.0f);
    usleep(500000);
    t0 = Now();
    CHECK(WRRotatorStopMove(id) == WR_SUCCESS, "stop returned in %.3fs", Now() - t0);
    CHECK(!St(id).moving, "not moving right after stop");
    usleep(1000000);
    double p = St(id).position;
    CHECK(Diff(p, start) > 0.05 && Diff(p, start) < 19.9, "stopped part-way: %.3f (start %.3f)", p, start);
    t0 = Now();
    CHECK(WRRotatorMoveTo(id, (float)start) == WR_SUCCESS, "MoveTo start accepted after %.2fs", Now() - t0);
    WaitIdle(id, 60);
    CHECK(fabs(Diff(St(id).position, start)) < 0.05, "back at start: %.3f", St(id).position);

    printf("\n[move while moving waits]\n");
    WRRotatorMove(id, 5.0f);
    usleep(200000);
    t0 = Now();
    CHECK(WRRotatorMove(id, -5.0f) == WR_SUCCESS, "second move returned after %.2fs", Now() - t0);
    WaitIdle(id, 60);
    CHECK(fabs(Diff(St(id).position, start)) < 0.05, "both applied, back at start: %.3f", St(id).position);

    printf("\n[SetConfig during a move (same values)]\n");
    WRRotatorMove(id, 5.0f);
    usleep(200000);
    WR_ROTATOR_CONFIG same = cfg;
    same.mask = MASK_ROTATOR_REVERSE_DIRECTION | MASK_ROTATOR_BACKLASH;
    t0 = Now();
    CHECK(WRRotatorSetConfig(id, &same) == WR_SUCCESS, "SetConfig waited %.2fs", Now() - t0);
    CHECK(fabs(Diff(St(id).position, start + 5)) < 0.05, "move unaffected: %.3f", St(id).position);
    WR_ROTATOR_CONFIG back; memset(&back, 0, sizeof back); WRRotatorGetConfig(id, &back);
    CHECK(back.reverseDirection == cfg.reverseDirection && fabs(back.backlash - cfg.backlash) < 0.01,
          "config unchanged (reverse=%d backlash=%.2f)", back.reverseDirection, back.backlash);
    WRRotatorMove(id, -5.0f); WaitIdle(id, 60);

    printf("\n[MoveTo during a move]\n");
    WRRotatorMove(id, 5.0f);
    usleep(200000);
    CHECK(WRRotatorMoveTo(id, (float)start) == WR_SUCCESS, "MoveTo accepted");
    WaitIdle(id, 60);
    CHECK(fabs(Diff(St(id).position, start)) < 0.05, "at start: %.3f", St(id).position);

    printf("\n[close during a move, reopen]\n");
    WRRotatorMove(id, 5.0f);
    usleep(200000);
    t0 = Now();
    WRRotatorClose(id);
    CHECK(Now() - t0 < 1.0, "close returned in %.2fs", Now() - t0);
    sleep(5);
    t0 = Now();
    CHECK(WRRotatorOpen(id) == WR_SUCCESS, "reopen (%.2fs)", Now() - t0);
    CHECK(fabs(Diff(St(id).position, start + 5)) < 0.05, "motor finished the move after close: %.3f", St(id).position);
    WRRotatorMoveTo(id, (float)start);
    WaitIdle(id, 60);

    printf("\n[stop 150 ms after a move starts (used to merge with the move)]\n");
    WRRotatorMove(id, 20.0f);
    usleep(150000);
    WRRotatorStopMove(id);
    usleep(500000);
    p = St(id).position;
    CHECK(Diff(p, start) > 0.05 && Diff(p, start) < 3.0, "move started at once and stopped early: %.3f (start %.3f)", p, start);
    WRRotatorMoveTo(id, (float)start); WaitIdle(id, 60);

    printf("\n[reopen while still moving]\n");
    WRRotatorMove(id, 90.0f);
    sleep(2);
    WRRotatorClose(id);
    t0 = Now();
    CHECK(WRRotatorOpen(id) == WR_SUCCESS, "reopen mid-move (%.2fs)", Now() - t0);
    p = St(id).position;
    CHECK(Diff(p, start) > 2.0 && Diff(p, start) < 20.0, "rotator halted part-way: %.3f", p);
    sleep(3);
    WRRotatorClose(id);
    WRRotatorOpen(id);
    CHECK(fabs(Diff(St(id).position, p)) < 0.05, "no unwanted move afterwards: %.3f", St(id).position);
    WRRotatorMoveTo(id, (float)start); WaitIdle(id, 60);
    CHECK(fabs(Diff(St(id).position, start)) < 0.05, "back at start: %.3f", St(id).position);

    printf("\n[450 deg move = two back-to-back segments]\n");
    t0 = Now();
    WRRotatorMove(id, 450.0f);
    CHECK(WaitIdle(id, 400), "completed in %.1fs", Now() - t0);
    p = St(id).position;
    CHECK(fabs(Diff(p, start + 90.0)) < 0.05 && p >= 0.0 && p < 360.0, "position %.3f ~ start+450 (mod 360)", p);
    t0 = Now();
    WRRotatorMove(id, -90.0f);
    WaitIdle(id, 120);
    CHECK(fabs(Diff(St(id).position, start)) < 0.05, "back at start: %.3f", St(id).position);

    printf("\n[final]\n");
    WRRotatorMoveTo(id, (float)start); WaitIdle(id, 60);
    s = St(id);
    CHECK(fabs(Diff(s.position, start)) < 0.05, "final position %.3f (start %.3f)", s.position, start);
    WRRotatorClose(id);
    printf(g_fail ? "\n%d FAILED\n" : "\nALL PASSED\n", g_fail);
    return g_fail != 0;
}
