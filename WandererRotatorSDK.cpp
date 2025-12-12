/* *******************************************************************************
 * MIT License
 *
 * Copyright (c) 2025 Nico Trost
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
#include <memory>
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cctype>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <dirent.h>
#include <chrono>
#include <libudev.h>

#define SDK_VERSION "1.0.0"

/* Import internal implementation for use in public C API */
using namespace WandererRotator;

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
	if (!number || !ids)
	{
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

	char response[64];

	/* Iterate through all tty devices */
	udev_list_entry_foreach(entry, devices)
	{
		if (count >= WR_MAX_NUM)
			break;

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

		WR_DEBUG("Found device with VID:%s PID:%s", vid, pid);

		if (strcmp(vid, "1a86") != 0 || strcmp(pid, "7523") != 0)
		{
			udev_device_unref(device);
			continue;
		}

		/* Get the device node (e.g., /dev/ttyUSB0) */
		const char *deviceNode = udev_device_get_devnode(device);
		if (!deviceNode)
		{
			udev_device_unref(device);
			continue;
		}

		WR_DEBUG("Trying to open device: %s", deviceNode);

		/* Try to open the port */
		auto port = std::make_shared<SerialPort>();
		if (port->Open(deviceNode))
		{
			WR_DEBUG("Port opened, flushing and sending command...");

			auto tempDevice = std::make_shared<Device>();
			tempDevice->port = port;
			tempDevice->portName = deviceNode;

			if (QueryHandshake(tempDevice))
			{
				WR_DEBUG("Valid Wanderer Rotator found!");
				/* Valid Wanderer Rotator found - close port, will be reopened in WRRotatorOpen */
				port->Close();
				int id = count;
				g_devices[id] = tempDevice;
				ids[count] = id;
				count++;
			}
			else
			{
				WR_DEBUG("No response from device");
				/* Not a valid Wanderer Rotator, close port */
				port->Close();
			}
		}
		else
		{
			WR_DEBUG("Failed to open port %s", deviceNode);
		}

		udev_device_unref(device);
	}

	/* Clean up udev resources */
	udev_enumerate_unref(enumerate);
	udev_unref(udev);

	*number = count;
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
	}

	WR_DEBUG("WRRotatorOpen: Attempting to open port %s", device->portName.c_str());
	if (!device->port->Open(device->portName.c_str()))
	{
		WR_ERROR("WRRotatorOpen: Failed to open port");
		return WR_ERROR_COMMUNICATION;
	}

	WR_DEBUG("WRRotatorOpen: Port opened successfully, performing handshake");

	/* Perform handshake */
	if (!QueryHandshake(device))
	{
		WR_ERROR("WRRotatorOpen: Handshake failed");
		device->port->Close();
		return WR_ERROR_COMMUNICATION;
	}

	if (!QueryStatus(device))
	{
		WR_ERROR("WRRotatorOpen: Querying for status failed");
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
		if (!SendCommand(device, cmd))
			return WR_ERROR_COMMUNICATION;

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

		if (!SendCommand(device, cmd))
		{
			return WR_ERROR_COMMUNICATION;
		}

		device->backlash = (int)(config->backlash * 10.0f);
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
	if (!SendCommand(device, "1500002"))
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

	/* Relative movement by angle in degrees
	 * Positive angle = counterclockwise
	 * Negative angle = clockwise
	 * Command: 1000000 + (angle * stepsPerDegree)
	 */

	int command_value = 1000000 + (int)(angle * device->stepsPerDegree);
	char cmd[8];
	snprintf(cmd, sizeof(cmd), "%d", command_value);

	WR_DEBUG("WRRotatorMove: angle=%.2f, command=%s", angle, cmd);

	// Store last target
	device->targetAngle = angle;

	/* Drain any leftover data in the buffer before sending move command */
	usleep(50000);
	tcflush(device->port->GetFD(), TCIFLUSH); /* Flush input buffer */

	if (!SendCommand(device, cmd))
	{
		return WR_ERROR_COMMUNICATION;
	}

	/* Mark device as moving - status will be updated when response arrives */
	device->status.moving = 1;

	/* Listener will get the rotation feedback */
	StartMoveListener(device);

	return WR_SUCCESS;
}

WRAPI WR_ERROR_TYPE WRRotatorMoveTo(int id, float angle)
{
	float delta;

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

		// Fetch current position
		if (!QueryStatus(device))
		{
			return WR_ERROR_COMMUNICATION;
		}

		float currentAngle = (float)device->mechanicalAngle / 1000.0f;

		/* Absolute positioning
		 * Calculate relative movement needed from current position
		 */
		delta = angle - currentAngle;

		/* Normalize delta to shortest path */
		delta = fmodf(delta + 180.0f, 360.0f);
		delta = (delta < 0.0f) ? delta + 180.0f : delta - 180.0f;

		// Skip 0 delta
		if (delta == 0.0f)
		{
			return WR_SUCCESS;
		}

		WR_DEBUG("Moving from %f by %f to %f\n", currentAngle, delta, angle);
	}

	return WRRotatorMove(id, delta);
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
	if (!SendCommand(device, "stop"))
	{
		return WR_ERROR_COMMUNICATION;
	}

	device->status.moving = 0;

	return WR_SUCCESS;
}