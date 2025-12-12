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

#include "WandererRotatorProtocol.h"
#include "WandererRotatorLogging.h"
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <atomic>
#include <memory>

namespace WandererRotator
{
    bool SendCommand(std::shared_ptr<Device> device, const char *command, int timeoutMs)
    {
        if (!device || !device->port || !device->port->IsOpen())
        {
            WR_DEBUG("SendCommand: device=%p, port=%p, isOpen=%d",
                     device.get(), device ? device->port.get() : nullptr,
                     device && device->port ? device->port->IsOpen() : 0);
            return false;
        }

        // 100 ms delay
        usleep(100000);

        WR_DEBUG("SendCommand: Writing '%s'", command);
        if (!device->port->Write((const unsigned char *)command, strlen(command)))
        {
            WR_DEBUG("SendCommand: Write failed");
            return false;
        }

        return true;
    }

    bool QueryHandshake(std::shared_ptr<Device> device)
    {
        if (!device || !device->port)
        {
            return false;
        }

        WR_DEBUG("QueryHandshake: started for device %s", device->portName.c_str());

        if (!device->port->IsOpen())
        {
            WR_DEBUG("QueryHandshake: Port not open");
            return false;
        }

        // 100 ms delay
        usleep(100000);

        int retries = 0;
        char response[32];

        while (retries++ < 5)
        {
            tcflush(device->port->GetFD(), TCIOFLUSH);
            if (!device->port->Write((const unsigned char *)"1500001\n", 8))
            {
                WR_DEBUG("Handshake: Writing to serial failed");
                return false;
            }

            if (device->port->Read((unsigned char *)response, 32, 'A', 3000))
            {
                if (strstr(response, "WandererRotator") != NULL)
                {
                    printf("Found after %d retries", retries);
                    return true;
                }
            }

            // 200 ms delay
            usleep(200000);
        }

        WR_DEBUG("Handshake: Handshaking timed out after %d retries", retries);
        return false;
    }

    bool QueryStatus(std::shared_ptr<Device> device)
    {
        if (!device || !device->port)
        {
            WR_DEBUG("QueryStatus: invalid device");
            return false;
        }

        WR_DEBUG("QueryStatus: started for device %s", device->portName.c_str());

        if (!device->port->IsOpen())
        {
            WR_DEBUG("QueryStatus: Port not open");
            return false;
        }

        // 100 ms delay
        usleep(100000);

        char response[32];

        tcflush(device->port->GetFD(), TCIOFLUSH);
        if (!device->port->Write((const unsigned char *)"1500001\n", 8))
        {
            WR_DEBUG("QueryStatus: Writing to serial failed");
            return false;
        }

        // Read handshake tag and model
        if (device->port->Read((unsigned char *)response, 32, 'A', 3000))
        {
            char model[8];
            if (sscanf(response, "WandererRotator%7[^A]A", model) != 1)
            {
                WR_DEBUG("QueryStatus: invalid message %s", response);
                return false;
            }

            device->modelType = std::string(model);
        }
        else
        {
            WR_DEBUG("QueryStatus: timeout reading model from serial");
            return false;
        }

        // Read firmware
        if (device->port->Read((unsigned char *)response, 32, 'A', 3000))
        {
            if (sscanf(response, "%dA", &device->firmwareVersion) != 1)
            {
                WR_DEBUG("QueryStatus: invalid message %s", response);
                return false;
            }
        }
        else
        {
            WR_DEBUG("QueryStatus: timeout reading firmware from serial");
            return false;
        }

        // Read mechanical position
        if (device->port->Read((unsigned char *)response, 32, 'A', 3000))
        {
            if (sscanf(response, "%dA", &device->mechanicalAngle) != 1)
            {
                WR_DEBUG("QueryStatus: invalid message %s", response);
                return false;
            }
        }
        else
        {
            WR_DEBUG("QueryStatus: timeout reading position from serial");
            return false;
        }

        // Read backlash
        if (device->port->Read((unsigned char *)response, 32, 'A', 3000))
        {
            float backlash;
            if (sscanf(response, "%fA", &backlash) != 1)
            {
                WR_DEBUG("QueryStatus: invalid message %s", response);
                return false;
            }
            device->backlash = backlash * 10.0f;
        }
        else
        {
            WR_DEBUG("QueryStatus: timeout reading backlash from serial");
            return false;
        }

        // Read reverse state
        if (device->port->Read((unsigned char *)response, 32, 'A', 3000))
        {
            if (sscanf(response, "%dA", &device->reverseDirection) != 1)
            {
                WR_DEBUG("QueryStatus: invalid message %s", response);
                return false;
            }
        }
        else
        {
            WR_DEBUG("QueryStatus: timeout reading reverse state from serial");
            return false;
        }

        /* Set steps per degree based on model type */
        if (device->modelType.find("Mini") != std::string::npos)
        {
            device->stepsPerDegree = 1142;
        }
        else if (device->modelType.find("Lite") != std::string::npos)
        {
            if (device->modelType.find("V2") != std::string::npos)
            {
                device->stepsPerDegree = 1199;
            }
            else
            {
                device->stepsPerDegree = 1155;
            }
        }

        device->status.stepsPerRevolution = device->stepsPerDegree * 360;
        device->status.stepSize = 1.0f / device->stepsPerDegree;

        /* Set initial position from mechanical angle */
        device->status.position = device->mechanicalAngle / 1000.0f;

        WR_DEBUG("QueryStatus: Successfully parsed, model=%s steps=%d",
                 device->modelType.c_str(), device->stepsPerDegree);
        return true;
    }

    int BacklashToCommand(float backlash)
    {
        return (int)(backlash * 10.0f) + 1600000;
    }

    const char *ReverseDirectionToCommand(int reverse)
    {
        return reverse ? "1700001\n" : "1700000\n";
    }

    /* Background listener thread function for movement completion */
    static void MoveListenerThreadFunc(std::shared_ptr<Device> device)
    {
        if (!device || !device->port)
        {
            return;
        }

        WR_DEBUG("MoveListener: Started for device %s", device->portName.c_str());

        if (!device->port->IsOpen())
        {
            WR_DEBUG("MoveListener: Port not open, exiting");
            device->listenerRunning = false;
            return;
        }

        char buffer[32];

        // Read the actual angle moved
        if (device->port->Read((unsigned char *)buffer, 32, 'A', 90000))
        {
            if (sscanf(buffer, "%fA", &device->lastRotated) != 1)
            {
                WR_DEBUG("MoveListener: Invalid message");
                device->listenerRunning = false;
                return;
            }
        }
        else
        {
            WR_DEBUG("MoveListener: Timeout reading from port");
            device->listenerRunning = false;
            return;
        }

        // Read the new position
        if (device->port->Read((unsigned char *)buffer, 32, 'A', 3000))
        {
            if (sscanf(buffer, "%dA", &device->mechanicalAngle) != 1)
            {
                WR_DEBUG("MoveListener: Invalid message");
                device->listenerRunning = false;
                return;
            }
            device->status.position = device->mechanicalAngle / 1000.0f; /* Convert from *1000 format to degrees */

            /* Check if we need to perform second phase of overshoot compensation */
            if (device->overshooting == 1)
            {
                device->overshooting = 2; /* Mark that first phase is done, ready for return */
                /* Keep moving = 1 since we have a second phase to do */

                WR_INFO("Backlash compensation: returning from overshoot by %.2f degrees", device->overshootAngle);

                /* Small delay before returning */
                usleep(100000);

                /* Move back by the overshoot amount to land on the actual target */
                float returnAngle = (device->targetAngle > 0.0f) ? -device->overshootAngle : device->overshootAngle;
                int command_value = 1000000 + (int)(returnAngle * device->stepsPerDegree);
                char cmd[16];
                snprintf(cmd, sizeof(cmd), "%d", command_value);

                WR_DEBUG("Return move command: %s", cmd);

                tcflush(device->port->GetFD(), TCIFLUSH); /* Flush input buffer */

                if (SendCommand(device, cmd))
                {
                    device->status.moving = 1;

                    /* Recursively call this function to handle the return movement */
                    device->listenerRunning = false; /* Will be reset by StartMoveListener */
                    StartMoveListener(device);
                    return;
                }
                else
                {
                    WR_ERROR("Failed to send return movement command");
                    device->overshooting = 0;
                    device->status.moving = 0;
                }
            }
            else if (device->overshooting == 2)
            {
                /* Second phase complete */
                device->overshooting = 0;
                device->status.moving = 0;
                WR_INFO("Backlash compensation complete, at target %.2f degrees", device->targetAngle);
            }
            else
            {
                /* No overshoot, just regular movement complete */
                device->status.moving = 0;
            }
        }
        else
        {
            WR_DEBUG("MoveListener: Timeout reading from port");
            device->listenerRunning = false;
            return;
        }

        /* Mark listener as stopped before exiting */
        device->listenerRunning = false;
        WR_DEBUG("MoveListener: Stopped for device %s", device->portName.c_str());
    }

    void StartMoveListener(std::shared_ptr<Device> device)
    {
        if (!device)
        {
            return;
        }

        /* Stop any existing listener by setting the flag */
        device->listenerRunning = false;

        /* Small delay to let old thread exit if it's still running */
        usleep(50000);

        /* Start new listener thread */
        device->listenerRunning = true;
        std::thread listenerThread(MoveListenerThreadFunc, device);
        listenerThread.detach(); /* Detach immediately - let it run independently */
        WR_DEBUG("StartMoveListener: Listener thread started");
    }

    void StopMoveListener(std::shared_ptr<Device> device)
    {
        if (!device)
        {
            return;
        }

        /* Signal listener thread to stop */
        device->listenerRunning = false;
        WR_DEBUG("StopMoveListener: Listener stop requested");
    }
} /* namespace WandererRotator */
