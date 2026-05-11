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
#include <cmath>
#include <cctype>
#include <mutex>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <dirent.h>
#include <libudev.h>

#define SDK_VERSION "1.3.3"

/* Import internal implementation for use in public C API */
using namespace WandererRotator;

/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */

static WR_ERROR_TYPE MoveInternal(std::shared_ptr<Device> device, float angle)
{
    /* Check if overshoot applies for this movement
     * Overshoot is only applied in one direction based on overshootDirection flag
     * overshootDirection: 0 = apply overshoot for positive angles (CCW)
     *                    1 = apply overshoot for negative angles (CW)
     */
    int shouldApplyOvershoot = 0;
    if (device->overshoot && device->overshootAngle > 0.0f)
    {
        if ((device->overshootDirection == 0 && angle > 0.0f) ||
            (device->overshootDirection == 1 && angle < 0.0f))
        {
            shouldApplyOvershoot = 1;
        }
    }

    /* Phase 1: Move to the desired angle (+ overshoot if applicable) */
    float moveAngle = angle;
    if (shouldApplyOvershoot)
    {
        /* Add overshoot in the direction of movement */
        if (angle > 0.0f)
        {
            moveAngle = angle + device->overshootAngle;
        }
        else
        {
            moveAngle = angle - device->overshootAngle;
        }
        WR_INFO("Applying overshoot: moving to %.2f (target: %.2f, overshoot: %.2f)", 
                moveAngle, angle, device->overshootAngle);

        /* Mark that we're in overshoot mode - waiting for first phase to complete */
        device->overshooting = 1;
        device->targetAngle = angle;
    }
    else
    {
        /* Ensure overshoot flag is cleared if not applying */
        device->overshooting = 0;
    }

    /* Relative movement by angle in degrees
     * Positive angle = counterclockwise
     * Negative angle = clockwise
     * Command: 1000000 + (angle * stepsPerDegree)
     */
    int command_value = 1000000 + (int)(moveAngle * device->stepsPerDegree);
    char cmd[8];
    snprintf(cmd, sizeof(cmd), "%d", command_value);

    WR_DEBUG("MoveInternal: angle=%.2f, command=%s", moveAngle, cmd);

    /* Drain any leftover data in the buffer before sending move command */
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    device->port->Flush();

    if (SendCommand(device, cmd) != WR_SUCCESS)
    {
        device->overshooting = 0;
        return WR_ERROR_COMMUNICATION;
    }

    /* Mark device as moving - status will be updated when response arrives */
    device->status.moving = 1;

    /* Listener will get the rotation feedback */
    StartMoveListener(device);

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
{
    auto port = std::make_shared<SerialPort>();
    
    /* Allow up to ~3 seconds for a competing SDK scan to release the port.
     * Workers run in parallel so this does not multiply scan duration —
     * total scan time = max(all worker times), not sum. A permanently
     * connected device (TIOCEXCL held indefinitely) will still be skipped
     * after timeout, which is correct. */
    port->SetRetryParams(60, 50);  /* 60 * 50ms = 3000ms total timeout, 50ms poll interval */
    
    if (!port->Open(task.portName.c_str()))
    {
        WR_DEBUG("ScanWorkerThread: Failed to open port %s (skipped, in use by another app)", task.portName.c_str());
        return;
    }

    auto tempDevice = std::make_shared<Device>();
    tempDevice->port = port;
    tempDevice->portName = task.portName;

    /* Perform status handshake with retry mechanism */
    WR_ERROR_TYPE stat = QueryStatus(tempDevice);

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

/* ============================================================================
 * PUBLIC SDK API IMPLEMENTATION
 * ============================================================================ */

WRAPI WR_ERROR_TYPE WRGetSDKVersion(char *version)
{
    if (!version)
    {
        return WR_ERROR_NULL_POINTER;
    }

    strncpy(version, SDK_VERSION, WR_VERSION_LEN - 1);
    version[WR_VERSION_LEN - 1] = '\0';
    return WR_SUCCESS;
}

WRAPI WR_ERROR_TYPE WRRotatorScan(int *number, int *ids)
{
    WR_INFO("Scanning for devices");
    if (!number || !ids)
    {
        WR_ERROR("Invalid pointer parameters");
        return WR_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    int count = 0;

    /* Create udev context */
    struct udev *udev = udev_new();
    if (!udev)
    {
        return WR_ERROR_COMMUNICATION;
    }

    /* Create enumeration for tty devices */
    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    if (!enumerate)
    {
        udev_unref(udev);
        return WR_ERROR_COMMUNICATION;
    }

    /* Filter for tty subsystem */
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry *entry;

    /* Step 1: Collect all candidate CH340 devices */
    std::vector<std::string> candidatePorts;
    udev_list_entry_foreach(entry, devices)
    {
        const char *path = udev_list_entry_get_name(entry);
        struct udev_device *device = udev_device_new_from_syspath(udev, path);
        if (!device)
        {
            continue;
        }

        /* Get the parent USB device */
        struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(
            device, "usb", "usb_device");

        if (!parent)
        {
            udev_device_unref(device);
            continue;
        }

        /* Check VID and PID for CH340 (1a86:7523) */
        const char *vid = udev_device_get_sysattr_value(parent, "idVendor");
        const char *pid = udev_device_get_sysattr_value(parent, "idProduct");

        if (!vid || !pid)
        {
            udev_device_unref(device);
            continue;
        }

        if (strcmp(vid, "1a86") != 0 || strcmp(pid, "7523") != 0)
        {
            udev_device_unref(device);
            continue;
        }

        /* Get the device node (e.g., /dev/ttyUSB0) */
        const char *deviceNode = udev_device_get_devnode(device);
        if (deviceNode)
        {
            WR_DEBUG("Found CH340 device: %s", deviceNode);
            candidatePorts.push_back(std::string(deviceNode));
        }

        udev_device_unref(device);
    }

    /* Step 2: Scan candidate devices in parallel */
    std::vector<ScanWorkerTask> tasks;
    std::vector<std::thread> workerThreads;

    for (const auto &port : candidatePorts)
    {
        if (count >= WR_MAX_NUM)
            break;
        tasks.emplace_back(port.c_str());
        count++;
    }

    /* Spawn worker threads for each candidate port */
    for (auto &task : tasks)
    {
        workerThreads.emplace_back(ScanWorkerThread, std::ref(task));
    }

    /* Wait for all threads to complete */
    for (auto &thread : workerThreads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }

    /* Step 3: Collect valid devices */
    count = 0;
    for (auto &task : tasks)
    {
        if (task.isValid && count < WR_MAX_NUM)
        {
            int id = count;
            g_devices[id] = task.device;
            ids[count] = id;
            count++;
        }
    }

    /* Clean up udev resources */
    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    *number = count;

    WR_INFO("Scan complete: found %d device(s)", count);
    return WR_SUCCESS;
}

WRAPI WR_ERROR_TYPE WRRotatorOpen(int id)
{
    std::lock_guard<std::mutex> lock(g_globalMutex);
    WR_DEBUG("WRRotatorOpen: Opening device id=%d", id);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        WR_ERROR("WRRotatorOpen: Device id=%d not found", id);
        return WR_ERROR_INVALID_ID;
    }

    auto device = it->second;
    WR_DEBUG("WRRotatorOpen: Found device, portName=%s", device->portName.c_str());

    /* Create a new SerialPort instance and open it */
    if (!device->port)
    {
        WR_DEBUG("WRRotatorOpen: Creating new SerialPort instance");
        device->port = std::make_shared<SerialPort>();
        /* Use aggressive retry parameters for normal device open.
         * More tolerant than scan to handle other SDKs scanning concurrently.
         * 10 retries with 300ms delay + 1.5x exponential backoff = ~5+ seconds wait
         * This ensures we don't fail if another scanner briefly holds the port. */
        device->port->SetRetryParams(10, 300);
    }

    WR_DEBUG("WRRotatorOpen: Attempting to open port %s", device->portName.c_str());
    if (!device->port->Open(device->portName.c_str()))
    {
        WR_ERROR("WRRotatorOpen: Failed to open port");
        return WR_ERROR_COMMUNICATION;
    }

    WR_DEBUG("WRRotatorOpen: Port opened successfully, performing handshake");

    /* Perform status handshake with retry mechanism */
    WR_ERROR_TYPE stat = QueryStatus(device);

    if (stat != WR_SUCCESS)
    {
        WR_ERROR("WRRotatorOpen: Querying for status handshake failed");
        device->port->Close();
        return WR_ERROR_COMMUNICATION;
    }

    WR_INFO("[OK] Rotator opened");
    return WR_SUCCESS;
}

WRAPI WR_ERROR_TYPE WRRotatorClose(int id)
{
    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        return WR_ERROR_INVALID_ID;
    }

    auto device = it->second;

    /* Stop any running listener thread first */
    StopMoveListener(device);

    if (device->port)
    {
        device->port->Close();
    }

    WR_INFO("[OK] Rotator closed");
    return WR_SUCCESS;
}

WRAPI WR_ERROR_TYPE WRRotatorGetConfig(int id, WR_ROTATOR_CONFIG *config)
{
    if (!config)
    {
        return WR_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        return WR_ERROR_INVALID_ID;
    }

    auto device = it->second;
    config->reverseDirection = device->rotator.reverseDirection;
    config->backlash = device->backlash / 10.0f; /* Convert from internal format */
    config->overshoot = device->overshoot;
    config->overshootAngle = device->overshootAngle;
    config->overshootDirection = device->overshootDirection;

    return WR_SUCCESS;
}

WRAPI WR_ERROR_TYPE WRRotatorSetConfig(int id, WR_ROTATOR_CONFIG *config)
{
    if (!config)
    {
        return WR_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        return WR_ERROR_INVALID_ID;
    }

    auto device = it->second;

    if (config->mask & MASK_ROTATOR_REVERSE_DIRECTION)
    {
        /* Send reverse direction command: 1700000 or 1700001 */
        const char *cmd = ReverseDirectionToCommand(config->reverseDirection);
        if (SendCommand(device, cmd) != WR_SUCCESS)
        {
            return WR_ERROR_COMMUNICATION;
        }

        device->rotator.reverseDirection = config->reverseDirection;
        device->reverseDirection = config->reverseDirection;
    }

    if (config->mask & MASK_ROTATOR_BACKLASH)
    {
        if (config->backlash < 0.0f)
        {
            return WR_ERROR_INVALID_PARAMETER;
        }

        /* Send backlash command: 10*x + 1600000 */
        int command_value = BacklashToCommand(config->backlash);
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "%d\n", command_value);

        if (SendCommand(device, cmd) != WR_SUCCESS)
        {
            return WR_ERROR_COMMUNICATION;
        }

        device->backlash = (int)(config->backlash * 10.0f);
    }

    if (config->mask & MASK_ROTATOR_OVERSHOOT)
    {
        device->overshoot = config->overshoot != 0;
        WR_DEBUG("Set overshoot to %d", config->overshoot);
    }

    if (config->mask & MASK_ROTATOR_OVERSHOOT_ANGLE)
    {
        if (config->overshootAngle < 0.0f)
        {
            return WR_ERROR_INVALID_PARAMETER;
        }

        device->overshootAngle = config->overshootAngle;
        WR_DEBUG("Set backlash overshoot to %.2f degrees", config->overshootAngle);
    }

    if (config->mask & MASK_ROTATOR_OVERSHOOT_DIRECTION)
    {
        if (config->overshootDirection < 0)
        {
            return WR_ERROR_INVALID_PARAMETER;
        }

        device->overshootDirection = config->overshootDirection != 0;
        WR_DEBUG("Set backlash overshoot direction to %d", device->overshootDirection);
    }

    return WR_SUCCESS;
}

WRAPI WR_ERROR_TYPE WRRotatorGetStatus(int id, WR_ROTATOR_STATUS *status)
{
    if (!status)
    {
        return WR_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        return WR_ERROR_INVALID_ID;
    }

    auto device = it->second;

    /* If currently moving, hardware does not support fetching latest status */

    status->position = device->status.position;
    status->moving = device->status.moving;
    status->stepsPerRevolution = device->status.stepsPerRevolution;
    status->stepSize = device->status.stepSize;

    return WR_SUCCESS;
}

WRAPI WR_ERROR_TYPE WRRotatorGetVersion(int id, WR_VERSION *version)
{
    if (!version)
    {
        return WR_ERROR_NULL_POINTER;
    }

    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        return WR_ERROR_INVALID_ID;
    }

    auto device = it->second;
    version->firmware = device->firmwareVersion;
    strncpy(version->model, device->modelType.c_str(), sizeof(version->model) - 1);
    version->model[sizeof(version->model) - 1] = '\0';

    return WR_SUCCESS;
}

WRAPI WR_ERROR_TYPE WRRotatorFindHome(int id)
{
    return WRRotatorMoveTo(id, 0.0f);
}

WRAPI WR_ERROR_TYPE WRRotatorSyncPosition(int id, float angle)
{
    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        return WR_ERROR_INVALID_ID;
    }

    auto device = it->second;

    if (!device->port || !device->port->IsOpen())
    {
        return WR_ERROR_COMMUNICATION;
    }

    // So far, we only support setting to zero
    if (angle != 0.0f)
    {
        return WR_ERROR_INVALID_PARAMETER;
    }

    if (angle < 0.0f || angle >= 360.0f)
    {
        return WR_ERROR_INVALID_PARAMETER;
    }

    /* Set the current mechanical position as zero (home)
     * Command: 1500002
     */
    if (SendCommand(device, "1500002") != WR_SUCCESS)
    {
        return WR_ERROR_COMMUNICATION;
    }

    /* Update the status position to reflect the sync */
    device->status.position = angle;

    return WR_SUCCESS;
}

WRAPI WR_ERROR_TYPE WRRotatorMove(int id, float angle)
{
    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        return WR_ERROR_INVALID_ID;
    }

    auto device = it->second;

    if (!device->port || !device->port->IsOpen())
    {
        return WR_ERROR_COMMUNICATION;
    }

    return MoveInternal(device, angle);
}

WRAPI WR_ERROR_TYPE WRRotatorMoveTo(int id, float angle)
{
    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        return WR_ERROR_INVALID_ID;
    }

    auto device = it->second;

    if (!device->port || !device->port->IsOpen())
    {
        return WR_ERROR_COMMUNICATION;
    }

    if (angle < 0.0f || angle >= 360.0f)
    {
        return WR_ERROR_INVALID_PARAMETER;
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

    return MoveInternal(device, delta);
}

WRAPI WR_ERROR_TYPE WRRotatorStopMove(int id)
{
    std::lock_guard<std::mutex> lock(g_globalMutex);

    auto it = g_devices.find(id);
    if (it == g_devices.end())
    {
        return WR_ERROR_INVALID_ID;
    }

    auto device = it->second;

    if (!device->port || !device->port->IsOpen())
    {
        return WR_ERROR_COMMUNICATION;
    }

    /* Send stop command */
    if (SendCommand(device, "stop") != WR_SUCCESS)
    {
        return WR_ERROR_COMMUNICATION;
    }

    device->status.moving = 0;

    return WR_SUCCESS;
}
