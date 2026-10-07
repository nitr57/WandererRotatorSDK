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

#include "WandererRotatorSDK.h"
#include "WandererRotatorLogging.h"
#include "WandererRotatorDevice.h"
#include "WandererRotatorProtocol.h"
#include "WandererRotatorSerialPort.h"
#include <map>
#include <mutex>
#include <thread>
#include <memory>
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <deque>
#include <algorithm>
#include <exception>
#include <cmath>
#include <cctype>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <dirent.h>
#include <libudev.h>

#define SDK_VERSION "1.5.0"

/* Import internal implementation for use in public C API */
using namespace WandererRotator;

/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */

/* No C++ exception may cross the extern "C" API boundary. */
#define WR_API_CATCH                                                  \
    catch (const std::exception &e)                                   \
    {                                                                 \
        WR_ERROR("%s: unexpected exception: %s", __func__, e.what()); \
        return WR_ERROR_COMMUNICATION;                                \
    }                                                                 \
    catch (...)                                                       \
    {                                                                 \
        WR_ERROR("%s: unexpected exception", __func__);               \
        return WR_ERROR_COMMUNICATION;                                \
    }

/* Largest step count sent in a single move command. Moves are encoded as
 * 1000000 + steps, so anything at or beyond +/-500000 steps would land in the
 * 15xxxxx/16xxxxx/17xxxxx ranges used for status, backlash and reverse
 * direction. Larger moves are split into consecutive segments instead. */
static const int kMaxStepsPerCommand = 499999;

/* Upper bound on the steps of one move phase. Far above the 3600 degree API
 * limit; it only keeps the float -> integer conversion well defined. */
static const float kMaxMoveSteps = 1.0e8f;

/* Serializes WRRotatorScan calls without blocking any other API call */
static std::mutex g_scanMutex;

static std::shared_ptr<Device> FindDevice(int id)
{
    std::lock_guard<std::mutex> lock(g_globalMutex);
    auto it = g_devices.find(id);
    return it == g_devices.end() ? nullptr : it->second;
}

/* Waits until no move is in progress on this device, then returns holding its
 * ioMutex. Calls that need the serial port while a move is running (Move,
 * MoveTo, SetConfig, SyncPosition) therefore run once that move completes -
 * which can take minutes for a very large move (~3.7 degrees/s). */
static std::unique_lock<std::mutex> LockIdle(const std::shared_ptr<Device> &device)
{
    while (true)
    {
        {
            std::unique_lock<std::mutex> state(device->stateMutex);
            device->moveDoneCv.wait(state, [&] { return !device->status.moving && !device->listenerRunning; });
        }

        std::unique_lock<std::mutex> io(device->ioMutex);
        if (!device->status.moving)
        {
            /* Reap the finished listener thread */
            StopMoveListener(device);
            return io;
        }
        /* Another call started a move in between - wait for that one too */
    }
}

static void AppendMoveSegments(std::deque<int> &segments, long long steps)
{
    while (steps != 0)
    {
        long long chunk = std::max<long long>(-kMaxStepsPerCommand, std::min<long long>(kMaxStepsPerCommand, steps));
        segments.push_back((int)chunk);
        steps -= chunk;
    }
}

/* Caller must hold device->ioMutex (see LockIdle) and have checked the device is open. */
static WR_ERROR_TYPE MoveInternalLocked(std::shared_ptr<Device> device, float angle)
{
    int overshoot, overshootDirection, stepsPerDegree;
    float overshootAngle;
    {
        std::lock_guard<std::mutex> lock(device->stateMutex);
        overshoot = device->overshoot;
        overshootAngle = device->overshootAngle;
        overshootDirection = device->overshootDirection;
        stepsPerDegree = device->stepsPerDegree;
    }

    /* Check if overshoot applies for this movement
     * Overshoot is only applied in one direction based on overshootDirection flag
     * overshootDirection: 0 = apply overshoot for positive angles (CCW)
     *                    1 = apply overshoot for negative angles (CW)
     */
    int shouldApplyOvershoot = 0;
    if (overshoot && overshootAngle > 0.0f)
    {
        if ((overshootDirection == 0 && angle > 0.0f) ||
            (overshootDirection == 1 && angle < 0.0f))
        {
            shouldApplyOvershoot = 1;
        }
    }

    /* Phase 1: Move to the desired angle (+ overshoot if applicable)
     * Phase 2: Move back by the overshoot amount to land on the actual target */
    float moveAngle = angle;
    float returnAngle = 0.0f;
    if (shouldApplyOvershoot)
    {
        /* Add overshoot in the direction of movement */
        if (angle > 0.0f)
        {
            moveAngle = angle + overshootAngle;
            returnAngle = -overshootAngle;
        }
        else
        {
            moveAngle = angle - overshootAngle;
            returnAngle = overshootAngle;
        }
        WR_INFO("Applying overshoot: moving to %.2f (target: %.2f, overshoot: %.2f)", 
                moveAngle, angle, overshootAngle);
    }

    /* Relative movement by angle in degrees
     * Positive angle = counterclockwise
     * Negative angle = clockwise
     * Command: 1000000 + (angle * stepsPerDegree)
     */
    float moveSteps = moveAngle * stepsPerDegree;
    float returnSteps = returnAngle * stepsPerDegree;
    if (!std::isfinite(moveSteps) || fabsf(moveSteps) > kMaxMoveSteps ||
        !std::isfinite(returnSteps) || fabsf(returnSteps) > kMaxMoveSteps)
    {
        return WR_ERROR_INVALID_PARAMETER;
    }

    std::deque<int> segments;
    AppendMoveSegments(segments, llroundf(moveSteps));
    AppendMoveSegments(segments, llroundf(returnSteps));

    if (segments.empty())
    {
        /* Less than one step - the rotator would not move and never reply */
        WR_DEBUG("MoveInternal: angle=%.4f is below one step, nothing to do", angle);
        return WR_SUCCESS;
    }

    /* Newline-terminated: without it the device waits ~1s for more input
     * before starting, and a command sent within that second (e.g. "stop")
     * would be merged into this one. */
    int command_value = 1000000 + segments.front();
    segments.pop_front();
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "%d\n", command_value);

    WR_DEBUG("MoveInternal: angle=%.2f, command=%s, %zu more segment(s)", moveAngle, cmd, segments.size());

    /* Drain any leftover data in the buffer before sending move command */
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    device->port->Flush();

    /* The listener sends the remaining segments as each one completes */
    {
        std::lock_guard<std::mutex> lock(device->pendingMovesMutex);
        device->pendingMoves = std::move(segments);
    }
    device->currentSegmentSteps = command_value - 1000000;

    if (SendCommand(device, cmd) != WR_SUCCESS)
    {
        std::lock_guard<std::mutex> lock(device->pendingMovesMutex);
        device->pendingMoves.clear();
        return WR_ERROR_COMMUNICATION;
    }

    /* Mark device as moving - status will be updated when response arrives */
    device->status.moving = 1;

    /* Listener will get the rotation feedback */
    if (!StartMoveListener(device))
    {
        {
            std::lock_guard<std::mutex> lock(device->pendingMovesMutex);
            device->pendingMoves.clear();
        }
        SetMoving(device.get(), false);
        return WR_ERROR_COMMUNICATION;
    }

    return WR_SUCCESS;
}

/* Structure for parallel device scanning */
struct ScanWorkerTask
{
    std::string portName;
    std::shared_ptr<WandererRotator::Device> device;
    bool isValid;
    
    ScanWorkerTask(const char *port) : portName(port), isValid(false) {}
};

/* Worker thread function for testing a single device */
static void ScanWorkerThread(ScanWorkerTask &task)
try
{
    auto port = std::make_shared<SerialPort>();
    
    /* Allow up to ~6 seconds for competing SDK scans to release the port.
     * Each competing scan holds a port for up to ~1.5s (see ProbeStatus).
     * With multiple SDKs scanning simultaneously, worst case is several
     * sequential holds before we get a window. Workers run in parallel
     * so this does not multiply scan duration across ports. */
    port->SetRetryParams(120, 50);  /* 120 * 50ms = 6000ms total timeout, 50ms poll interval */
    
    if (!port->Open(task.portName.c_str()))
    {
        WR_DEBUG("ScanWorkerThread: Failed to open port %s (skipped, in use by another app)", task.portName.c_str());
        return;
    }

    auto tempDevice = std::make_shared<Device>();
    tempDevice->port = port;
    tempDevice->portName = task.portName;

    /* Quick handshake - a device that stays silent is released after ~1.5s */
    WR_ERROR_TYPE stat = ProbeStatus(tempDevice);

    if(stat != WR_SUCCESS)
    {
        port->Close();
        return;
    }

    WR_DEBUG("ScanWorkerThread: Valid device found on %s", task.portName.c_str());

    /* Valid device found - close port */
    port->Close();
    
    task.device = tempDevice;
    task.isValid = true;
}
catch (const std::exception &e)
{
    /* An exception escaping a std::thread would call std::terminate */
    WR_ERROR("ScanWorkerThread: unexpected exception on %s: %s", task.portName.c_str(), e.what());
}

/* ============================================================================
 * PUBLIC SDK API IMPLEMENTATION
 * ============================================================================ */

WRAPI WR_ERROR_TYPE WRGetSDKVersion(char *version)
try
{
    if (!version)
    {
        return WR_ERROR_NULL_POINTER;
    }

    static_assert(sizeof(SDK_VERSION) <= WR_VERSION_LEN, "SDK_VERSION too long");
    memcpy(version, SDK_VERSION, sizeof(SDK_VERSION));
    return WR_SUCCESS;
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorScan(int *number, int *ids)
try
{
    WR_INFO("Scanning for devices");
    if (!number || !ids)
    {
        WR_ERROR("Invalid pointer parameters");
        return WR_ERROR_NULL_POINTER;
    }

    /* Only one scan at a time. g_globalMutex is not held while probing, so
     * calls on already-registered devices are never blocked by a scan. */
    std::lock_guard<std::mutex> scanLock(g_scanMutex);

    /* Ports that already belong to a registered device are not probed again:
     * an open one would just fail/hang, and re-registering it would give the
     * same device a second id. */
    std::map<std::string, int> connectedPorts;
    {
        std::lock_guard<std::mutex> lock(g_globalMutex);
        for (const auto &pair : g_devices)
        {
            if (!pair.second->portName.empty())
            {
                connectedPorts[pair.second->portName] = pair.first;
            }
        }
    }

    /* Create udev context */
    std::unique_ptr<struct udev, decltype(&udev_unref)> udev(udev_new(), &udev_unref);
    if (!udev)
    {
        return WR_ERROR_COMMUNICATION;
    }

    /* Create enumeration for tty devices */
    std::unique_ptr<struct udev_enumerate, decltype(&udev_enumerate_unref)> enumerate(
        udev_enumerate_new(udev.get()), &udev_enumerate_unref);
    if (!enumerate)
    {
        return WR_ERROR_COMMUNICATION;
    }

    /* Filter for tty subsystem */
    udev_enumerate_add_match_subsystem(enumerate.get(), "tty");
    udev_enumerate_scan_devices(enumerate.get());

    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate.get());
    struct udev_list_entry *entry;

    /* Step 1: Collect all candidate CH340 devices */
    std::vector<std::string> candidatePorts;
    udev_list_entry_foreach(entry, devices)
    {
        const char *path = udev_list_entry_get_name(entry);
        std::unique_ptr<struct udev_device, decltype(&udev_device_unref)> device(
            udev_device_new_from_syspath(udev.get(), path), &udev_device_unref);
        if (!device)
        {
            continue;
        }

        /* Get the parent USB device */
        struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(
            device.get(), "usb", "usb_device");

        if (!parent)
        {
            continue;
        }

        /* Check VID and PID for CH340 (1a86:7523) */
        const char *vid = udev_device_get_sysattr_value(parent, "idVendor");
        const char *pid = udev_device_get_sysattr_value(parent, "idProduct");

        if (!vid || !pid)
        {
            continue;
        }

        if (strcmp(vid, "1a86") != 0 || strcmp(pid, "7523") != 0)
        {
            continue;
        }

        /* Get the device node (e.g., /dev/ttyUSB0) */
        const char *deviceNode = udev_device_get_devnode(device.get());
        if (deviceNode)
        {
            WR_DEBUG("Found CH340 device: %s", deviceNode);
            candidatePorts.push_back(std::string(deviceNode));
        }
    }

    /* Step 2: Scan candidate devices in parallel, skipping ports that already
     * belong to a registered device. */
    std::vector<ScanWorkerTask> tasks;
    std::vector<std::thread> workerThreads;

    for (const auto &port : candidatePorts)
    {
        if (connectedPorts.find(port) != connectedPorts.end())
            continue;
        if ((int)tasks.size() >= WR_MAX_NUM)
            break;
        tasks.emplace_back(port.c_str());
    }

    /* Spawn worker threads for each candidate port */
    for (auto &task : tasks)
    {
        try
        {
            workerThreads.emplace_back(ScanWorkerThread, std::ref(task));
        }
        catch (...)
        {
            /* Destroying a joinable std::thread calls std::terminate - join the
             * workers already started before letting the error propagate. */
            for (auto &thread : workerThreads)
            {
                thread.join();
            }
            throw;
        }
    }

    /* Wait for all threads to complete */
    for (auto &thread : workerThreads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }

    /* Step 3: Forget registered devices whose port has disappeared (unless the
     * caller still has them open), report the remaining ones under their
     * existing id, and give newly found devices the smallest free id. */
    std::lock_guard<std::mutex> lock(g_globalMutex);

    for (auto it = g_devices.begin(); it != g_devices.end();)
    {
        bool present = std::find(candidatePorts.begin(), candidatePorts.end(),
                                 it->second->portName) != candidatePorts.end();
        if (!present && !it->second->isOpen)
        {
            WR_DEBUG("Scan: Device id=%d on %s is gone, removing", it->first, it->second->portName.c_str());
            it = g_devices.erase(it);
        }
        else
        {
            ++it;
        }
    }

    int count = 0;
    for (const auto &pair : g_devices)
    {
        if (count >= WR_MAX_NUM)
            break;
        ids[count] = pair.first;
        count++;
    }

    for (auto &task : tasks)
    {
        if (task.isValid && count < WR_MAX_NUM)
        {
            int id = 0;
            while (g_devices.find(id) != g_devices.end())
                id++;

            g_devices[id] = task.device;
            ids[count] = id;
            count++;
        }
    }

    *number = count;

    WR_INFO("Scan complete: found %d device(s)", count);
    return WR_SUCCESS;
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorOpen(int id)
try
{
    WR_DEBUG("WRRotatorOpen: Opening device id=%d", id);

    auto device = FindDevice(id);
    if (!device)
    {
        WR_ERROR("WRRotatorOpen: Device id=%d not found", id);
        return WR_ERROR_INVALID_ID;
    }

    WR_DEBUG("WRRotatorOpen: Found device, portName=%s", device->portName.c_str());

    /* Per-device lock only: opening can wait up to ~10s for a busy port and
     * must not block calls on other devices. */
    std::lock_guard<std::mutex> io(device->ioMutex);

    if (device->isOpen)
    {
        WR_DEBUG("WRRotatorOpen: Device already open");
        return WR_SUCCESS;
    }

    /* Create a new SerialPort instance if needed */
    if (!device->port)
    {
        WR_DEBUG("WRRotatorOpen: Creating new SerialPort instance");
        device->port = std::make_shared<SerialPort>();
    }

    /* Always update retry params before open - port may have been created during
     * scan with scan-phase params (120*50ms=6s). Open needs a longer budget to
     * outlast the full scan+open overlap window across all SDKs.
     * 20 * 500ms = 10 seconds total wait, polling every 50ms. */
    device->port->SetRetryParams(20, 500);

    WR_DEBUG("WRRotatorOpen: Attempting to open port %s", device->portName.c_str());
    if (!device->port->Open(device->portName.c_str()))
    {
        WR_ERROR("WRRotatorOpen: Failed to open port");
        return WR_ERROR_COMMUNICATION;
    }

    WR_DEBUG("WRRotatorOpen: Port opened successfully, performing handshake");

    /* The rotator may still be moving (e.g. closed mid-move); the status query
     * below would then end that move and start an unwanted one. */
    StopAndDrain(device.get());

    /* Perform status handshake with retry mechanism */
    WR_ERROR_TYPE stat = QueryStatus(device);

    if (stat != WR_SUCCESS)
    {
        WR_ERROR("WRRotatorOpen: Querying for status handshake failed");
        device->port->Close();
        return WR_ERROR_COMMUNICATION;
    }

    device->isOpen = true;

    WR_INFO("[OK] Rotator opened");
    return WR_SUCCESS;
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorClose(int id)
try
{
    auto device = FindDevice(id);
    if (!device)
    {
        return WR_ERROR_INVALID_ID;
    }

    {
        /* Does not wait for a move in progress to finish (and does not stop
         * the motor) - the listener is interrupted within one read slice. */
        std::lock_guard<std::mutex> io(device->ioMutex);

        StopMoveListener(device);

        {
            std::lock_guard<std::mutex> lock(device->pendingMovesMutex);
            device->pendingMoves.clear();
        }

        if (device->port)
        {
            device->port->Close();
        }

        device->isOpen = false;
    }

    SetMoving(device.get(), false);

    WR_INFO("[OK] Rotator closed");
    return WR_SUCCESS;
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorGetConfig(int id, WR_ROTATOR_CONFIG *config)
try
{
    if (!config)
    {
        return WR_ERROR_NULL_POINTER;
    }

    auto device = FindDevice(id);
    if (!device)
    {
        return WR_ERROR_INVALID_ID;
    }

    std::lock_guard<std::mutex> lock(device->stateMutex);
    config->reverseDirection = device->reverseDirection;
    config->backlash = device->backlash / 10.0f; /* Convert from internal format */
    config->overshoot = device->overshoot;
    config->overshootAngle = device->overshootAngle;
    config->overshootDirection = device->overshootDirection;

    return WR_SUCCESS;
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorSetConfig(int id, const WR_ROTATOR_CONFIG *config)
try
{
    if (!config)
    {
        return WR_ERROR_NULL_POINTER;
    }

    auto device = FindDevice(id);
    if (!device)
    {
        return WR_ERROR_INVALID_ID;
    }

    /* Validate every requested field before sending anything, so an invalid
     * field can't leave the device with only part of the config applied.
     * Backlash is sent as 10*x + 1600000 and must stay below the 17xxxxx
     * reverse-direction commands. */
    if ((config->mask & MASK_ROTATOR_BACKLASH) &&
        !(config->backlash >= 0.0f && config->backlash < 10000.0f))
    {
        return WR_ERROR_INVALID_PARAMETER;
    }

    if ((config->mask & MASK_ROTATOR_OVERSHOOT_ANGLE) &&
        !(config->overshootAngle >= 0.0f && std::isfinite(config->overshootAngle)))
    {
        return WR_ERROR_INVALID_PARAMETER;
    }

    /* Reverse direction and backlash are stored on the device itself */
    if (config->mask & (MASK_ROTATOR_REVERSE_DIRECTION | MASK_ROTATOR_BACKLASH))
    {
        auto io = LockIdle(device);

        if (!device->isOpen)
        {
            return WR_ERROR_INVALID_STATE;
        }

        if (config->mask & MASK_ROTATOR_REVERSE_DIRECTION)
        {
            /* Send reverse direction command: 1700000 or 1700001 */
            const char *cmd = ReverseDirectionToCommand(config->reverseDirection);
            if (SendCommand(device, cmd) != WR_SUCCESS)
            {
                return WR_ERROR_COMMUNICATION;
            }

            std::lock_guard<std::mutex> lock(device->stateMutex);
            device->reverseDirection = config->reverseDirection != 0;
        }

        if (config->mask & MASK_ROTATOR_BACKLASH)
        {
            /* Send backlash command: 10*x + 1600000 */
            int command_value = BacklashToCommand(config->backlash);
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "%d\n", command_value);

            if (SendCommand(device, cmd) != WR_SUCCESS)
            {
                return WR_ERROR_COMMUNICATION;
            }

            std::lock_guard<std::mutex> lock(device->stateMutex);
            device->backlash = command_value - 1600000;
        }
    }

    /* Overshoot is applied by the SDK; a change takes effect from the next move */
    std::lock_guard<std::mutex> lock(device->stateMutex);

    if (config->mask & MASK_ROTATOR_OVERSHOOT)
    {
        device->overshoot = config->overshoot != 0;
        WR_DEBUG("Set overshoot to %d", config->overshoot);
    }

    if (config->mask & MASK_ROTATOR_OVERSHOOT_ANGLE)
    {
        device->overshootAngle = config->overshootAngle;
        WR_DEBUG("Set backlash overshoot to %.2f degrees", config->overshootAngle);
    }

    if (config->mask & MASK_ROTATOR_OVERSHOOT_DIRECTION)
    {
        device->overshootDirection = config->overshootDirection != 0;
        WR_DEBUG("Set backlash overshoot direction to %d", device->overshootDirection);
    }

    return WR_SUCCESS;
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorGetStatus(int id, WR_ROTATOR_STATUS *status)
try
{
    if (!status)
    {
        return WR_ERROR_NULL_POINTER;
    }

    auto device = FindDevice(id);
    if (!device)
    {
        return WR_ERROR_INVALID_ID;
    }

    /* If currently moving, hardware does not support fetching latest status */
    std::lock_guard<std::mutex> lock(device->stateMutex);
    status->position = device->status.position;
    status->moving = device->status.moving;
    status->stepsPerRevolution = device->status.stepsPerRevolution;
    status->stepSize = device->status.stepSize;

    return WR_SUCCESS;
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorGetVersion(int id, WR_VERSION *version)
try
{
    if (!version)
    {
        return WR_ERROR_NULL_POINTER;
    }

    auto device = FindDevice(id);
    if (!device)
    {
        return WR_ERROR_INVALID_ID;
    }

    std::lock_guard<std::mutex> lock(device->stateMutex);
    version->firmware = device->firmwareVersion;
    strncpy(version->model, device->modelType.c_str(), sizeof(version->model) - 1);
    version->model[sizeof(version->model) - 1] = '\0';

    return WR_SUCCESS;
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorFindHome(int id)
try
{
    return WRRotatorMoveTo(id, 0.0f);
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorSyncPosition(int id, float angle)
try
{
    auto device = FindDevice(id);
    if (!device)
    {
        return WR_ERROR_INVALID_ID;
    }

    // So far, we only support setting to zero
    if (angle != 0.0f)
    {
        return WR_ERROR_INVALID_PARAMETER;
    }

    auto io = LockIdle(device);

    if (!device->isOpen)
    {
        return WR_ERROR_INVALID_STATE;
    }

    /* Set the current mechanical position as zero (home)
     * Command: 1500002 (newline-terminated so a following command can't be
     * merged into it)
     */
    if (SendCommand(device, "1500002\n") != WR_SUCCESS)
    {
        return WR_ERROR_COMMUNICATION;
    }

    /* Update the status position to reflect the sync */
    device->mechanicalAngle = 0;
    device->status.position = angle;

    return WR_SUCCESS;
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorMove(int id, float angle)
try
{
    auto device = FindDevice(id);
    if (!device)
    {
        return WR_ERROR_INVALID_ID;
    }

    /* Moves larger than one command can carry are split into segments; this
     * limit only rejects nonsensical requests. */
    if (!std::isfinite(angle) || fabsf(angle) > 3600.0f)
    {
        return WR_ERROR_INVALID_PARAMETER;
    }

    /* A move already in progress on this device finishes first */
    auto io = LockIdle(device);

    if (!device->isOpen)
    {
        return WR_ERROR_INVALID_STATE;
    }

    return MoveInternalLocked(device, angle);
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorMoveTo(int id, float angle)
try
{
    auto device = FindDevice(id);
    if (!device)
    {
        return WR_ERROR_INVALID_ID;
    }

    if (!std::isfinite(angle) || angle < 0.0f || angle >= 360.0f)
    {
        return WR_ERROR_INVALID_PARAMETER;
    }

    /* A move already in progress on this device finishes first */
    auto io = LockIdle(device);

    if (!device->isOpen)
    {
        return WR_ERROR_INVALID_STATE;
    }

    /* Perform status update */
    RETURN_IF_ERROR(QueryStatus(device));

    float currentAngle = (float)device->mechanicalAngle / 1000.0f;

    /* Absolute positioning
     * Calculate relative movement needed from current position
     */
    float delta = angle - currentAngle;

    /* Normalize delta to shortest path */
    delta = fmodf(delta + 180.0f, 360.0f);
    delta = (delta < 0.0f) ? delta + 180.0f : delta - 180.0f;

    // Skip 0 delta
    if (delta == 0.0f)
    {
        return WR_SUCCESS;
    }

    WR_DEBUG("Moving from %f by %f to %f\n", currentAngle, delta, angle);

    return MoveInternalLocked(device, delta);
}
WR_API_CATCH

WRAPI WR_ERROR_TYPE WRRotatorStopMove(int id)
try
{
    auto device = FindDevice(id);
    if (!device)
    {
        return WR_ERROR_INVALID_ID;
    }

    /* ioMutex is only held for short command exchanges, never while waiting
     * for a move, so a stop is sent promptly. */
    std::lock_guard<std::mutex> io(device->ioMutex);

    if (!device->isOpen)
    {
        return WR_ERROR_INVALID_STATE;
    }

    {
        /* Cancel the remaining segments (large-move chunks, overshoot return) and
         * send "stop" under the lock the listener holds while sending a segment,
         * so no segment can go out after the stop. */
        std::lock_guard<std::mutex> lock(device->pendingMovesMutex);
        device->pendingMoves.clear();

        /* Let the listener wait only briefly for the final position */
        if (device->listenerRunning)
        {
            device->stopRequestedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch()).count();
        }

        /* Not SendCommand: its input flush could discard the reply the
         * listener is reading right now. */
        const char *cmd = "stop";
        device->port->Drain();
        if (!device->port->Write((const unsigned char *)cmd, strlen(cmd)))
        {
            return WR_ERROR_COMMUNICATION;
        }
    }

    SetMoving(device.get(), false);

    return WR_SUCCESS;
}
WR_API_CATCH
