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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

bool WaitForRotatorReady(int deviceId, int maxWaitSeconds = 120)
{
	WR_ROTATOR_STATUS status;
	int elapsed = 0;
	int noResponseCount = 0;

	while (elapsed < maxWaitSeconds)
	{
		usleep(500000); /* 0.5 second */
		elapsed++;

		WR_ERROR_TYPE result = WRRotatorGetStatus(deviceId, &status);
		if (result != WR_SUCCESS)
		{
			printf("    [FAIL] Failed to get status (Error: %d)\n", result);
			return false;
		}

		printf("    Position: %.2f°, Moving: %s\n", status.position, status.moving ? "Yes" : "No");

		if (!status.moving)
		{
			printf("    Movement completed after %d.%d seconds\n", elapsed / 2, (elapsed % 2) * 5);
			return true;
		}

		noResponseCount++;
	}

	printf("    [TIMEOUT] Movement did not complete within %d seconds\n", maxWaitSeconds);
	return false;
}

int main(int argc, char *argv[])
{
	printf("=== Wanderer Rotator Interactive Movement Test ===\n\n");

	/* Scan for devices */
	printf("Scanning for devices...\n");
	int deviceCount = 32;
	int deviceIds[32] = {0};

	WR_ERROR_TYPE result = WRRotatorScan(&deviceCount, deviceIds);
	if (result != WR_SUCCESS)
	{
		printf("Scan failed (Error: %d)\n", result);
		return 1;
	}

	printf("Found %d device(s)\n\n", deviceCount);

	if (deviceCount == 0)
	{
		printf("No devices found. Please connect a Wanderer Rotator.\n");
		return 0;
	}

	/* Use specified device ID or first device found */
	int deviceId = deviceIds[0];
	if (argc > 1)
	{
		int requestedId = atoi(argv[1]);
		bool found = false;
		for (int i = 0; i < deviceCount; i++)
		{
			if (deviceIds[i] == requestedId)
			{
				deviceId = requestedId;
				found = true;
				break;
			}
		}
		if (!found)
		{
			printf("Device %d not found in scan results. Using first device: %d\n\n", requestedId, deviceId);
		}
	}

	printf("Testing device with ID: %d\n\n", deviceId);

	/* Open rotator */
	result = WRRotatorOpen(deviceId);
	if (result != WR_SUCCESS)
	{
		printf("Failed to open rotator (Error: %d)\n", result);
		return 1;
	}
	printf("[OK] Rotator opened\n\n");

	/* Get initial status */
	WR_ROTATOR_STATUS status;
	result = WRRotatorGetStatus(deviceId, &status);

	WR_VERSION version;
	WRRotatorGetVersion(deviceId, &version);

	WR_ROTATOR_CONFIG config;
	WRRotatorGetConfig(deviceId, &config);

	if (result == WR_SUCCESS)
	{
		printf("Initial Status:\n");
		printf("===============\n");
		printf("Model: %s\n", version.model);
		printf("Firmware: %u\n", version.firmware);
		printf("Backlash: %.2f°\n", config.backlash);
		printf("Reverse: %s\n", config.reverseDirection ? "Yes" : "No");
		printf("Position: %.2f°\n", status.position);
		printf("Moving: %s\n", status.moving ? "Yes" : "No");
		printf("Steps per revolution: %d\n", status.stepsPerRevolution);
		printf("Step size: %.4f°/step\n\n", status.stepSize);
	}

	/* Interactive menu */
	char input[256];
	bool running = true;

	while (running)
	{
		/* Get current config to show reverse status */
		WR_ROTATOR_CONFIG config;
		WRRotatorGetConfig(deviceId, &config);

		printf("\n--- Rotator Movement Commands ---\n");
		printf("m <angle>   - Move to angle (0-360°)\n");
		printf("r <angle>   - Move relative by angle\n");
		printf("s           - Stop movement\n");
		printf("h           - Find home (sync to 0°)\n");
		printf("g           - Get current status\n");
		printf("d           - Toggle reverse direction (currently %s)\n", config.reverseDirection ? "ON" : "OFF");
		printf("b <angle>   - Set backlash in degrees\n");
		printf("q           - Quit\n");
		printf("> ");
		fflush(stdout);

		if (fgets(input, sizeof(input), stdin) == NULL)
		{
			break;
		}

		/* Remove newline */
		input[strcspn(input, "\n")] = 0;

		if (strlen(input) == 0)
		{
			continue;
		}

		char cmd = input[0];
		float angle = 0;

		if (cmd == 'm' || cmd == 'r' || cmd == 'b')
		{
			if (sscanf(input, "%c %f", &cmd, &angle) != 2)
			{
				printf("[FAIL] Invalid format. Use: %c <angle>\n", input[0]);
				continue;
			}
		}

		switch (cmd)
		{
		case 'm':
		{
			if (angle < 0.0f || angle >= 360.0f)
			{
				printf("[FAIL] Angle must be in range [0, 360)\n");
				break;
			}
			printf("Moving rotator to %.2f°...\n", angle);
			result = WRRotatorMoveTo(deviceId, angle);
			if (result != WR_SUCCESS)
			{
				printf("[FAIL] Movement failed (Error: %d)\n", result);
			}
			else
			{
				printf("[OK] Movement command sent\n");
				if (WaitForRotatorReady(deviceId))
				{
					printf("[OK] Movement completed\n");
				}
			}
			break;
		}
		case 'r':
		{
			printf("Moving rotator by %.2f°...\n", angle);
			result = WRRotatorMove(deviceId, angle);
			if (result != WR_SUCCESS)
			{
				printf("[FAIL] Movement failed (Error: %d)\n", result);
			}
			else
			{
				printf("[OK] Movement command sent\n");
				while (!WaitForRotatorReady(deviceId))
				{
					sleep(1);
				}

				printf("[OK] Movement completed\n");
			}
			break;
		}
		case 's':
		{
			printf("Stopping rotator...\n");
			result = WRRotatorStopMove(deviceId);
			if (result != WR_SUCCESS)
			{
				printf("[FAIL] Stop failed (Error: %d)\n", result);
			}
			else
			{
				printf("[OK] Stop command sent\n");
			}
			break;
		}
		case 'h':
		{
			printf("Syncing position to 0° (home)...\n");
			result = WRRotatorSyncPosition(deviceId, 0.0f);
			if (result != WR_SUCCESS)
			{
				printf("[FAIL] Sync failed (Error: %d)\n", result);
			}
			else
			{
				printf("[OK] Position synced to 0°\n");
			}
			break;
		}
		case 'g':
		{
			result = WRRotatorGetStatus(deviceId, &status);
			WR_ROTATOR_CONFIG currentConfig;
			WRRotatorGetConfig(deviceId, &currentConfig);

			if (result == WR_SUCCESS)
			{
				printf("\nCurrent Status:\n");
				printf("===============\n");
				printf("Position: %.2f°\n", status.position);
				printf("Moving: %s\n", status.moving ? "Yes" : "No");
				printf("Backlash: %.2f°\n", currentConfig.backlash);
				printf("Reverse: %s\n", currentConfig.reverseDirection ? "Yes" : "No");
				printf("Steps per revolution: %d\n", status.stepsPerRevolution);
				printf("Step size: %.4f°/step\n", status.stepSize);
			}
			else
			{
				printf("[FAIL] Failed to get status (Error: %d)\n", result);
			}
			break;
		}
		case 'd':
		{
			/* Toggle reverse direction */
			WR_ROTATOR_CONFIG config;
			result = WRRotatorGetConfig(deviceId, &config);
			if (result != WR_SUCCESS)
			{
				printf("[FAIL] Failed to get config (Error: %d)\n", result);
				break;
			}

			config.reverseDirection = !config.reverseDirection;
			config.mask = MASK_ROTATOR_REVERSE_DIRECTION;

			result = WRRotatorSetConfig(deviceId, &config);
			if (result != WR_SUCCESS)
			{
				printf("[FAIL] Failed to set reverse direction (Error: %d)\n", result);
			}
			else
			{
				printf("[OK] Reverse direction toggled to: %s\n", config.reverseDirection ? "ON" : "OFF");
			}
			break;
		}
		case 'b':
		{
			/* Set backlash */
			if (angle < 0.0f)
			{
				printf("[FAIL] Backlash must be >= 0\n");
				break;
			}
			printf("Setting backlash to %.2f°...\n", angle);
			WR_ROTATOR_CONFIG config;
			config.backlash = angle;
			config.mask = MASK_ROTATOR_BACKLASH;

			result = WRRotatorSetConfig(deviceId, &config);
			if (result != WR_SUCCESS)
			{
				printf("[FAIL] Failed to set backlash (Error: %d)\n", result);
			}
			else
			{
				printf("[OK] Backlash set to %.2f°\n", angle);
			}
			break;
		}
		case 'q':
		{
			running = false;
			break;
		}
		default:
		{
			printf("[FAIL] Unknown command: %c\n", cmd);
			break;
		}
		}
	}

	/* Close rotator */
	printf("\nClosing rotator...\n");
	result = WRRotatorClose(deviceId);
	if (result == WR_SUCCESS)
	{
		printf("[OK] Rotator closed\n");
	}
	else
	{
		printf("[FAIL] Failed to close rotator (Error: %d)\n", result);
	}

	printf("\n=== Test Complete ===\n");
	return 0;
}
