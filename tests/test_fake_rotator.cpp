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

/* Automated SDK tests against a simulated rotator on a pseudo-terminal.
 *
 * The fake answers the same serial protocol the SDK speaks (status query,
 * sync, reverse direction, backlash, moves and "stop") and flags any command
 * that arrives while a move is still running, other than "stop".
 *
 * Usage: test_fake_rotator [test-name]   (no argument runs all tests) */

#include "WandererRotatorSDK.h"
#include "WandererRotatorDevice.h"
#include "WandererRotatorProtocol.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <string>
#include <sys/resource.h>
#include <sys/stat.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono;
using namespace WandererRotator;

static const int kStepsPerDegree = 1199; /* LiteV2 */

/* Behaves like the real firmware (LiteV2, verified on hardware where noted):
 * - Moves are answered once complete with "<rotated>A<mechanical>A\r\n"; the
 *   mechanical angle (thousandths of a degree) is not wrapped at 0/360.
 * - Any command received during a move ends it and is answered with the move
 *   result. A numeric command received mid-move also starts a new, unwanted
 *   move ("runaway"); "stop" does not.
 * - A move command without '\n' only runs after ~1s without further input, so
 *   anything sent in that window is merged into it.
 * - "stop" while idle and zero-step moves get no answer.
 * - After power-up the firmware ignores all input for a while (measured:
 *   ~450ms after the port is first opened); see bootMs. */
struct FakeRotator
{
    int master = -1;
    std::string slavePath;
    std::thread thread;
    std::atomic<bool> run{true};

    /* Behaviour knobs - set before the SDK talks to the fake */
    std::atomic<int> moveMs{200};       /* duration of a move */
    std::atomic<bool> silent{false};    /* never answer moves */
    std::atomic<bool> garbled{false};   /* send junk when a move starts */
    std::atomic<int> reverse{0};
    std::atomic<int> startMovingMs{0};  /* start a move of this duration, as if commanded earlier */
    std::atomic<int> bootMs{0};         /* ignore all input for this long after creation */
    steady_clock::time_point created = steady_clock::now();

    /* State, owned by the fake's thread unless noted */
    std::atomic<int> mech{90000};       /* thousandths of a degree */
    std::mutex cmdMutex;
    std::vector<std::string> cmds;      /* guarded by cmdMutex */
    std::atomic<int> commandsWhileMoving{0}; /* commands other than "stop" received mid-move */
    std::atomic<int> runaways{0};       /* unwanted moves started by those */
    bool moving = false;
    int moveSteps = 0;
    steady_clock::time_point moveStart, moveEnd;

    FakeRotator()
    {
        master = posix_openpt(O_RDWR | O_NOCTTY);
        grantpt(master);
        unlockpt(master);
        slavePath = ptsname(master);
        struct termios t;
        tcgetattr(master, &t);
        cfmakeraw(&t);
        tcsetattr(master, TCSANOW, &t);
        thread = std::thread([this] { Loop(); });
    }

    ~FakeRotator() { Stop(); }

    void Stop()
    {
        run = false;
        if (thread.joinable())
            thread.join();
        if (master >= 0)
            close(master);
        master = -1;
    }

    /* Simulate a USB unplug: closing the master side hangs up the tty */
    void Hangup() { Stop(); }

    std::vector<std::string> Cmds()
    {
        std::lock_guard<std::mutex> lock(cmdMutex);
        return cmds;
    }

    std::vector<std::string> MoveCmds()
    {
        std::vector<std::string> out;
        for (auto &c : Cmds())
            if (IsMove(c))
                out.push_back(c);
        return out;
    }

    static bool IsMove(const std::string &c)
    {
        if (c.empty() || !(isdigit((unsigned char)c[0]) || c[0] == '-'))
            return false;
        for (char ch : c.substr(1))
            if (!isdigit((unsigned char)ch))
                return false;
        return !(c.size() == 7 && (c.compare(0, 2, "15") == 0 || c.compare(0, 2, "16") == 0 || c.compare(0, 2, "17") == 0));
    }

    static bool IsFixedCommand(const std::string &c)
    {
        return c == "1500001" || c == "1500002" || c == "1700000" || c == "1700001" || c == "stop";
    }

    void Send(const std::string &s) { (void)!write(master, s.data(), s.size()); }

    void StartMove(int steps, int ms)
    {
        moving = true;
        moveSteps = steps;
        moveStart = steady_clock::now();
        moveEnd = moveStart + milliseconds(ms);
    }

    void FinishMove(double fraction)
    {
        int steps = (int)lround(moveSteps * fraction);
        double deg = steps / (double)kStepsPerDegree;
        mech = mech + (int)lround(deg * 1000.0);
        char b[64];
        snprintf(b, sizeof b, "%.2fA%dA\r\n", deg, mech.load());
        Send(b);
        moving = false;
    }

    static std::string Trim(std::string c)
    {
        while (!c.empty() && (c.back() == '\n' || c.back() == '\r'))
            c.pop_back();
        return c;
    }

    void Record(const std::string &c)
    {
        std::lock_guard<std::mutex> lock(cmdMutex);
        cmds.push_back(c);
    }

    /* Bytes received while a move is running */
    void Interrupt(const std::string &raw)
    {
        std::string c = Trim(raw);
        Record(c);
        double total = duration<double>(moveEnd - moveStart).count();
        double done = duration<double>(steady_clock::now() - moveStart).count();
        FinishMove(total > 0 ? std::min(1.0, done / total) : 1.0);
        if (c != "stop")
        {
            commandsWhileMoving++;
            if (!c.empty() && isdigit((unsigned char)c[0]))
            {
                runaways++;
                StartMove(-720 * kStepsPerDegree, 60000);
            }
        }
    }

    void Handle(const std::string &raw)
    {
        std::string c = Trim(raw);
        Record(c);

        if (c == "1500001")
        {
            char b[128];
            snprintf(b, sizeof b, "WandererRotatorLiteV2A20250101A%dA0.00A%dA\r\n", mech.load(), reverse.load());
            Send(b);
        }
        else if (c == "1500002")
            mech = 0;
        else if (c == "1700000" || c == "1700001")
            reverse = c.back() - '0';
        else if (IsMove(c))
        {
            int steps = atoi(c.c_str()) - 1000000;
            if (silent || steps == 0)
                return;
            StartMove(steps, moveMs);
            if (garbled)
                Send("xyzA");
        }
    }

    void Loop()
    {
        std::string acc;
        auto lastByte = steady_clock::now();
        while (run)
        {
            if (int ms = startMovingMs.exchange(0))
                StartMove(30 * kStepsPerDegree, ms);
            if (moving && steady_clock::now() >= moveEnd)
                FinishMove(1.0);

            struct pollfd p = {master, POLLIN, 0};
            int r = poll(&p, 1, 10);
            if (r > 0 && (p.revents & POLLIN))
            {
                char b[256];
                ssize_t n = read(master, b, sizeof b);
                if (n > 0)
                {
                    acc.append(b, n);
                    lastByte = steady_clock::now();
                    continue;
                }
                std::this_thread::sleep_for(milliseconds(10)); /* slave side closed (EIO) */
            }
            else if (r > 0)
                std::this_thread::sleep_for(milliseconds(10));

            if (acc.empty())
                continue;
            if (steady_clock::now() < created + milliseconds(bootMs.load()))
            {
                acc.clear(); /* still booting */
                continue;
            }
            long quietMs = (long)duration_cast<milliseconds>(steady_clock::now() - lastByte).count();
            if (moving)
            {
                if (quietMs >= 20)
                {
                    Interrupt(acc);
                    acc.clear();
                }
            }
            /* Idle: '\n' ends a command at once, known fixed commands after a
             * short pause, anything else (moves) only after ~1s of silence */
            else if (acc.back() == '\n' || (quietMs >= 20 && IsFixedCommand(acc)) || quietMs >= 1000)
            {
                Handle(acc);
                acc.clear();
            }
        }
    }
};

static int g_failures = 0;

#define CHECK(cond, ...)                 \
    do                                   \
    {                                    \
        bool ok_ = (cond);               \
        printf(ok_ ? "  PASS  " : "  FAIL  "); \
        printf(__VA_ARGS__);             \
        printf("\n");                    \
        if (!ok_)                        \
            g_failures++;                \
    } while (0)

/* Register a device for the fake's port, as WRRotatorScan would (scanning
 * itself needs a real CH340 adapter on udev). */
static int AddDevice(FakeRotator &fake)
{
    auto device = std::make_shared<Device>();
    device->portName = fake.slavePath;
    std::lock_guard<std::mutex> lock(g_globalMutex);
    int id = 0;
    while (g_devices.count(id))
        id++;
    g_devices[id] = device;
    return id;
}

static WR_ROTATOR_STATUS Status(int id)
{
    WR_ROTATOR_STATUS s = {};
    WRRotatorGetStatus(id, &s);
    return s;
}

static bool WaitIdle(int id, int ms)
{
    auto end = steady_clock::now() + milliseconds(ms);
    while (steady_clock::now() < end)
    {
        if (!Status(id).moving)
            return true;
        std::this_thread::sleep_for(milliseconds(20));
    }
    return false;
}

static long ElapsedMs(steady_clock::time_point start)
{
    return (long)duration_cast<milliseconds>(steady_clock::now() - start).count();
}

static double CpuSeconds()
{
    struct rusage u;
    getrusage(RUSAGE_SELF, &u);
    return u.ru_utime.tv_sec + u.ru_stime.tv_sec + (u.ru_utime.tv_usec + u.ru_stime.tv_usec) / 1e6;
}

static void TestBasic()
{
    FakeRotator fake;
    int id = AddDevice(fake);
    CHECK(WRRotatorOpen(id) == WR_SUCCESS, "open");
    CHECK(WRRotatorMove(id, 10.0f) == WR_SUCCESS, "move");
    CHECK(WaitIdle(id, 3000), "move completes");
    CHECK(fabsf(Status(id).position - 100.0f) < 0.05f, "position %.3f ~ 100", Status(id).position);
    CHECK(WRRotatorMoveTo(id, 45.0f) == WR_SUCCESS && WaitIdle(id, 3000), "move to 45");
    CHECK(fabsf(Status(id).position - 45.0f) < 0.05f, "position %.3f ~ 45", Status(id).position);
    WRRotatorClose(id);
}

static void TestOvershoot()
{
    FakeRotator fake;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WR_ROTATOR_CONFIG c = {};
    c.mask = MASK_ROTATOR_OVERSHOOT | MASK_ROTATOR_OVERSHOOT_ANGLE;
    c.overshoot = 1;
    c.overshootAngle = 2.0f;
    WRRotatorSetConfig(id, &c);
    WRRotatorMove(id, 10.0f);
    CHECK(WaitIdle(id, 3000), "completes");
    auto mv = fake.MoveCmds();
    CHECK(mv.size() == 2 && mv[0] == "1014388" && mv[1] == "997602", "move +12 then -2 (%s %s)",
          mv.size() > 0 ? mv[0].c_str() : "-", mv.size() > 1 ? mv[1].c_str() : "-");
    CHECK(fabsf(Status(id).position - 100.0f) < 0.05f, "position %.3f ~ 100", Status(id).position);

    c.mask = MASK_ROTATOR_OVERSHOOT_ANGLE;
    c.overshootAngle = 1.0e9f;
    WRRotatorSetConfig(id, &c);
    CHECK(WRRotatorMove(id, 10.0f) == WR_ERROR_INVALID_PARAMETER, "absurd overshoot rejected by Move");
    WRRotatorClose(id);
}

static void TestLargeMove()
{
    FakeRotator fake;
    fake.moveMs = 50;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WRRotatorMove(id, 450.0f);
    CHECK(WaitIdle(id, 3000), "completes");
    bool inRange = true;
    long total = 0;
    for (auto &c : fake.MoveCmds())
    {
        int v = atoi(c.c_str());
        inRange &= v > 500000 && v < 1500000;
        total += v - 1000000;
    }
    CHECK(inRange, "every move command stays inside the move range");
    CHECK(total == 450L * kStepsPerDegree, "total steps %ld", total);
    WRRotatorClose(id);
}

static void TestInvalidInput()
{
    FakeRotator fake;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    size_t afterOpen = fake.Cmds().size();
    CHECK(WRRotatorMove(id, NAN) == WR_ERROR_INVALID_PARAMETER, "Move(NaN) rejected");
    CHECK(WRRotatorMoveTo(id, NAN) == WR_ERROR_INVALID_PARAMETER, "MoveTo(NaN) rejected");
    CHECK(WRRotatorMove(id, 3600.5f) == WR_ERROR_INVALID_PARAMETER, "Move(3600.5) rejected");
    WR_ROTATOR_CONFIG c = {};
    c.mask = MASK_ROTATOR_BACKLASH;
    c.backlash = NAN;
    CHECK(WRRotatorSetConfig(id, &c) == WR_ERROR_INVALID_PARAMETER, "backlash NaN rejected");
    c.backlash = 10000.0f;
    CHECK(WRRotatorSetConfig(id, &c) == WR_ERROR_INVALID_PARAMETER, "backlash 10000 rejected");
    CHECK(fake.Cmds().size() == afterOpen, "nothing sent after open (%zu commands)", fake.Cmds().size() - afterOpen);
    WRRotatorClose(id);
}

static void TestZeroMove()
{
    FakeRotator fake;
    fake.silent = true;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    CHECK(WRRotatorMove(id, 0.0f) == WR_SUCCESS, "Move(0) ok");
    CHECK(fake.MoveCmds().empty(), "no move command sent");
    CHECK(WaitIdle(id, 500), "not left moving");
    WRRotatorClose(id);
}

static void TestConfig()
{
    FakeRotator fake;
    fake.reverse = 1;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WR_ROTATOR_CONFIG c = {};
    WRRotatorGetConfig(id, &c);
    CHECK(c.reverseDirection == 1, "GetConfig reports the device's reverse direction (%d)", c.reverseDirection);

    c = {};
    c.mask = MASK_ROTATOR_REVERSE_DIRECTION | MASK_ROTATOR_BACKLASH;
    c.reverseDirection = 0;
    c.backlash = -1;
    CHECK(WRRotatorSetConfig(id, &c) == WR_ERROR_INVALID_PARAMETER, "invalid backlash rejected");
    std::this_thread::sleep_for(milliseconds(100));
    CHECK(fake.reverse == 1, "reverse direction not applied when another field is invalid");

    c.backlash = 0.3f;
    CHECK(WRRotatorSetConfig(id, &c) == WR_SUCCESS, "valid config applied");
    WR_ROTATOR_CONFIG r = {};
    WRRotatorGetConfig(id, &r);
    CHECK(r.reverseDirection == 0 && fabsf(r.backlash - 0.3f) < 1e-4f, "read back reverse=%d backlash=%.2f",
          r.reverseDirection, r.backlash);
    WRRotatorClose(id);
}

static void TestCloseDuringMove()
{
    FakeRotator fake;
    fake.moveMs = 8000;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WRRotatorMove(id, 10.0f);
    std::this_thread::sleep_for(milliseconds(100));
    size_t beforeClose = fake.Cmds().size();
    auto start = steady_clock::now();
    WRRotatorClose(id);
    long ms = ElapsedMs(start);
    CHECK(ms < 1000, "Close returned in %ld ms", ms);
    std::this_thread::sleep_for(milliseconds(100));
    bool sentStop = false;
    auto cmds = fake.Cmds();
    for (size_t i = beforeClose; i < cmds.size(); i++)
        sentStop |= cmds[i] == "stop";
    CHECK(!sentStop, "Close did not stop the motor");
    CHECK(!Status(id).moving, "closed device reports not moving");
}

static void TestStopDuringMove()
{
    FakeRotator fake;
    fake.moveMs = 8000;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WRRotatorMove(id, 10.0f);
    std::this_thread::sleep_for(milliseconds(400));
    auto start = steady_clock::now();
    CHECK(WRRotatorStopMove(id) == WR_SUCCESS, "stop");
    CHECK(ElapsedMs(start) < 500, "stop returned promptly");
    CHECK(!Status(id).moving, "idle right after stop");
    std::this_thread::sleep_for(milliseconds(300));
    float pos = Status(id).position;
    CHECK(pos > 90.05f && pos < 99.0f, "position from the stop reply: %.3f", pos);

    start = steady_clock::now();
    CHECK(WRRotatorMove(id, 1.0f) == WR_SUCCESS, "move after stop");
    CHECK(ElapsedMs(start) < 1000, "move after stop did not wait for a timeout (%ld ms)", ElapsedMs(start));
    WRRotatorStopMove(id);
    WRRotatorClose(id);
}

static void TestStopDuringOvershoot()
{
    FakeRotator fake;
    fake.moveMs = 300;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WR_ROTATOR_CONFIG c = {};
    c.mask = MASK_ROTATOR_OVERSHOOT | MASK_ROTATOR_OVERSHOOT_ANGLE;
    c.overshoot = 1;
    c.overshootAngle = 2.0f;
    WRRotatorSetConfig(id, &c);
    WRRotatorMove(id, 10.0f);
    std::this_thread::sleep_for(milliseconds(100));
    WRRotatorStopMove(id);
    std::this_thread::sleep_for(milliseconds(800));
    CHECK(fake.MoveCmds().size() == 1, "no overshoot return move after stop (%zu move commands)", fake.MoveCmds().size());
    WRRotatorClose(id);
}

static void TestUnplug()
{
    FakeRotator fake;
    fake.silent = true;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WRRotatorMove(id, 10.0f);
    std::this_thread::sleep_for(milliseconds(100));
    double cpuStart = CpuSeconds();
    fake.Hangup();
    bool idle = WaitIdle(id, 6000);
    double cpu = CpuSeconds() - cpuStart;
    CHECK(idle, "moving cleared after unplug");
    CHECK(cpu < 1.0, "no busy loop while waiting (%.2fs CPU)", cpu);
    auto start = steady_clock::now();
    WRRotatorClose(id);
    CHECK(ElapsedMs(start) < 2000, "Close after unplug took %ld ms", ElapsedMs(start));
}

static void TestGarbledReply()
{
    FakeRotator fake;
    fake.garbled = true;
    fake.moveMs = 2000;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WRRotatorMove(id, 10.0f);
    CHECK(WaitIdle(id, 4000), "moving cleared after a garbled reply mid-move");
    CHECK(fake.runaways == 0, "recovery stopped the move before querying (%d runaway moves)", fake.runaways.load());
    float expected = MechanicalAngleToPosition(fake.mech);
    CHECK(fabsf(Status(id).position - expected) < 0.01f, "position re-queried from the device: %.3f (device %.3f)",
          Status(id).position, expected);
    WRRotatorClose(id);
}

static void TestOpenWhileMoving()
{
    FakeRotator fake;
    fake.startMovingMs = 10000;
    std::this_thread::sleep_for(milliseconds(100));
    int id = AddDevice(fake);
    CHECK(WRRotatorOpen(id) == WR_SUCCESS, "open while the rotator is still moving");
    CHECK(fake.runaways == 0, "handshake did not start an unwanted move (%d runaway moves)", fake.runaways.load());
    CHECK(!Status(id).moving, "reported idle");
    WRRotatorClose(id);
}

static void TestStopRightAfterMove()
{
    FakeRotator fake;
    fake.moveMs = 2000;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WRRotatorMove(id, 10.0f);
    std::this_thread::sleep_for(milliseconds(300));
    WRRotatorStopMove(id);
    std::this_thread::sleep_for(milliseconds(300));
    float pos = Status(id).position;
    CHECK(pos > 90.05f && pos < 92.0f, "move started at once and was stopped part-way: %.3f", pos);
    CHECK(fake.commandsWhileMoving == 0, "stop was not merged into the move command (%d)", fake.commandsWhileMoving.load());
    WRRotatorClose(id);
}

static void TestPositionWraps()
{
    FakeRotator fake;
    fake.mech = 1000;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WRRotatorMove(id, -5.0f);
    WaitIdle(id, 3000);
    CHECK(fabsf(Status(id).position - 356.0f) < 0.05f, "position wrapped to 0..360: %.3f (device %d)", Status(id).position, fake.mech.load());
    CHECK(WRRotatorMoveTo(id, 10.0f) == WR_SUCCESS && WaitIdle(id, 3000), "MoveTo across 0");
    CHECK(fabsf(Status(id).position - 10.0f) < 0.05f && fake.mech == 10000, "at 10 via the short way (device %d)", fake.mech.load());
    WRRotatorClose(id);
}

static void TestMoveWhileMoving()
{
    FakeRotator fake;
    fake.moveMs = 600;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WRRotatorMove(id, 10.0f);
    std::this_thread::sleep_for(milliseconds(100));
    auto start = steady_clock::now();
    CHECK(WRRotatorMove(id, 5.0f) == WR_SUCCESS, "second move accepted");
    long ms = ElapsedMs(start);
    CHECK(ms >= 400, "second Move waited for the first to finish (%ld ms)", ms);
    CHECK(WaitIdle(id, 3000), "idle afterwards");
    CHECK(fake.commandsWhileMoving == 0 && fake.runaways == 0, "no command reached the device mid-move (%d)", fake.commandsWhileMoving.load());
    CHECK(fabsf(Status(id).position - 105.0f) < 0.05f, "both moves applied: %.3f ~ 105", Status(id).position);
    WRRotatorClose(id);
}

static void TestCommandsDuringMove()
{
    FakeRotator fake;
    fake.moveMs = 500;
    int id = AddDevice(fake);
    WRRotatorOpen(id);
    WRRotatorMove(id, 10.0f);
    std::this_thread::sleep_for(milliseconds(100));

    WR_ROTATOR_CONFIG c = {};
    c.mask = MASK_ROTATOR_REVERSE_DIRECTION;
    c.reverseDirection = 1;
    CHECK(WRRotatorSetConfig(id, &c) == WR_SUCCESS, "SetConfig during move");
    CHECK(!Status(id).moving, "SetConfig returned after the move finished");
    CHECK(fabsf(Status(id).position - 100.0f) < 0.05f, "move completed normally: %.3f", Status(id).position);

    WRRotatorMove(id, 10.0f);
    std::this_thread::sleep_for(milliseconds(100));
    CHECK(WRRotatorSyncPosition(id, 0.0f) == WR_SUCCESS, "SyncPosition during move");
    CHECK(fabsf(Status(id).position) < 0.001f, "synced to 0 after the move: %.3f", Status(id).position);

    WRRotatorMove(id, 10.0f);
    std::this_thread::sleep_for(milliseconds(100));
    CHECK(WRRotatorMoveTo(id, 0.0f) == WR_SUCCESS && WaitIdle(id, 3000), "MoveTo during move");
    CHECK(fabsf(Status(id).position) < 0.05f, "ended at 0: %.3f", Status(id).position);

    CHECK(fake.commandsWhileMoving == 0 && fake.runaways == 0, "no command reached the device mid-move (%d)", fake.commandsWhileMoving.load());
    WRRotatorClose(id);
}

static void TestNotOpen()
{
    FakeRotator fake;
    int id = AddDevice(fake);
    CHECK(WRRotatorMove(id, 1.0f) == WR_ERROR_INVALID_STATE, "Move on closed device");
    CHECK(WRRotatorMoveTo(id, 1.0f) == WR_ERROR_INVALID_STATE, "MoveTo on closed device");
    CHECK(WRRotatorStopMove(id) == WR_ERROR_INVALID_STATE, "StopMove on closed device");
    CHECK(WRRotatorSyncPosition(id, 0.0f) == WR_ERROR_INVALID_STATE, "SyncPosition on closed device");
    WR_ROTATOR_CONFIG c = {};
    c.mask = MASK_ROTATOR_OVERSHOOT;
    c.overshoot = 1;
    CHECK(WRRotatorSetConfig(id, &c) == WR_SUCCESS, "SDK-side config allowed while closed");
    c.mask = MASK_ROTATOR_BACKLASH;
    CHECK(WRRotatorSetConfig(id, &c) == WR_ERROR_INVALID_STATE, "device config on closed device");
    CHECK(WRRotatorMove(99, 1.0f) == WR_ERROR_INVALID_ID, "unknown id");
}

static void TestPermissionDenied()
{
    if (geteuid() == 0)
    {
        printf("  SKIP  running as root\n");
        return;
    }
    FakeRotator fake;
    int id = AddDevice(fake);
    chmod(fake.slavePath.c_str(), 0);
    auto start = steady_clock::now();
    CHECK(WRRotatorOpen(id) == WR_ERROR_COMMUNICATION, "open fails");
    long ms = ElapsedMs(start);
    CHECK(ms < 3000, "permission error reported quickly (%ld ms)", ms);
    chmod(fake.slavePath.c_str(), 0620);
}

/* The handshake WRRotatorScan uses on a port (scanning itself needs udev and
 * a real CH340 adapter): open the port as a scan worker does, then probe. */
static bool ProbeLikeScan(FakeRotator &fake, long &elapsedMs)
{
    auto device = std::make_shared<Device>();
    device->portName = fake.slavePath;
    device->port = std::make_shared<SerialPort>();
    auto start = steady_clock::now();
    bool ok = device->port->Open(fake.slavePath.c_str()) && ProbeStatus(device) == WR_SUCCESS;
    elapsedMs = ElapsedMs(start);
    device->port->Close();
    return ok;
}

static void TestScanProbe()
{
    long ms;
    {
        FakeRotator fake;
        CHECK(ProbeLikeScan(fake, ms), "booted rotator found in %ld ms", ms);
        CHECK(ms < 900, "quick when the rotator answers at once");
    }
    {
        FakeRotator fake;
        fake.bootMs = 1100; /* ~2.5x the measured boot time */
        CHECK(ProbeLikeScan(fake, ms), "freshly plugged-in rotator (1.1s boot) found in %ld ms", ms);
    }
    {
        FakeRotator fake;
        fake.bootMs = 60000; /* never answers: some other CH340 device */
        CHECK(!ProbeLikeScan(fake, ms), "silent device rejected");
        CHECK(ms < 1700, "silent port released after %ld ms", ms);
    }
}

static void TestVersion()
{
    char buf[64];
    memset(buf, 'x', sizeof buf);
    WRGetSDKVersion(buf);
    size_t len = strlen(buf);
    CHECK(len < sizeof(buf) - 1 && buf[len + 1] == 'x', "writes only the version string (\"%s\")", buf);
}

int main(int argc, char **argv)
{
    setvbuf(stdout, nullptr, _IONBF, 0);

    struct
    {
        const char *name;
        void (*fn)();
    } tests[] = {
        {"basic", TestBasic},
        {"overshoot", TestOvershoot},
        {"large", TestLargeMove},
        {"invalid", TestInvalidInput},
        {"zero", TestZeroMove},
        {"config", TestConfig},
        {"close", TestCloseDuringMove},
        {"stop", TestStopDuringMove},
        {"stop-overshoot", TestStopDuringOvershoot},
        {"unplug", TestUnplug},
        {"garbled", TestGarbledReply},
        {"open-while-moving", TestOpenWhileMoving},
        {"stop-after-move", TestStopRightAfterMove},
        {"position-wraps", TestPositionWraps},
        {"move-while-moving", TestMoveWhileMoving},
        {"commands-during-move", TestCommandsDuringMove},
        {"not-open", TestNotOpen},
        {"permission", TestPermissionDenied},
        {"scan-probe", TestScanProbe},
        {"version", TestVersion},
    };

    const char *only = argc > 1 ? argv[1] : nullptr;
    for (auto &t : tests)
    {
        if (only && strcmp(only, t.name) != 0)
            continue;
        printf("%s\n", t.name);
        t.fn();
    }

    printf(g_failures ? "\n%d check(s) FAILED\n" : "\nALL PASSED\n", g_failures);

    /* Skip static destructors: devices of finished tests stay registered */
    fflush(stdout);
    _exit(g_failures ? 1 : 0);
}
