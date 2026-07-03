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
#include "WandererRotatorProtocol.h"
#include "WandererRotatorLogging.h"
#include <cstring>
#include <cstdio>
#include <memory>
#include <chrono>
#include <thread>

namespace WandererRotator
{
    WR_ERROR_TYPE SendCommand(std::shared_ptr<Device> device, const char *command, int timeoutMs)
    {
        if (!device)
        {
            return WR_ERROR_NULL_POINTER;
        }

        if (!device->port || !device->port->IsOpen())
        {
            WR_DEBUG("SendCommand: device=%p, port=%p, isOpen=%d",
                     device.get(), device ? device->port.get() : nullptr,
                     device && device->port ? device->port->IsOpen() : 0);
            return WR_ERROR_COMMUNICATION;
        }

        device->port->Drain();
        device->port->Flush();
        device->port->ClearRxBuffer();

        if (!device->port->Write((const unsigned char *)command, strlen(command)))
        {
            WR_DEBUG("SendCommand: Write failed");
            return WR_ERROR_COMMUNICATION;
        }

        return WR_SUCCESS;
    }

    int ReadResponse(std::shared_ptr<Device> device, char *response, int maxLen, int timeoutMs)
    {
        if(!device || !device->port || !device->port->IsOpen())
        {
            WR_DEBUG("ReadResponse: device=%p, port=%p, isOpen=%d",
                     device.get(), device ? device->port.get() : nullptr,
                     device && device->port ? device->port->IsOpen() : 0);
            return false;
        }

        WR_DEBUG("ReadResponse: Reading...");
        int len = device->port->Read((unsigned char *)response, maxLen, '\r', 500);
        if(len == 0)
        {
            WR_DEBUG("ReadResponse: Read failed");
            return 0;
        }

        return len;
    }

    WR_ERROR_TYPE SendAndWaitForReply(std::shared_ptr<Device> device, const char* cmd, char* buffer, int maxLen, int sendTimeoutMs, int recvTimeoutMs)
    {
        // Send command
        RETURN_IF_ERROR(SendCommand(device, cmd, sendTimeoutMs));

        // Read response
        if (!ReadResponse(device, buffer, maxLen, recvTimeoutMs))
        {
            WR_DEBUG("Failed to receive response");
            return WR_ERROR_COMMUNICATION;
        }

        return WR_SUCCESS;
    }

    WR_ERROR_TYPE SendAndWaitForReplyWithRetry(std::shared_ptr<Device> device,
                                         const char *cmd,
                                         char* buffer,
                                         int maxLen,
                                         int sendTimeoutMs,
                                         int recvTimeoutMs,
                                         int maxRetries,
                                         int retryDelayMs,
                                         const char *timeoutMsg)
    {
        for (int attempt = 1; attempt <= maxRetries; ++attempt)
        {
            WR_DEBUG("SendAndWaitForReplyWithRetry: Attempt %d/%d for %s (timeout=%dms)", attempt, maxRetries, timeoutMsg, recvTimeoutMs);

            if (SendAndWaitForReply(device, cmd, buffer, maxLen, sendTimeoutMs, recvTimeoutMs) == WR_SUCCESS)
            {
                WR_DEBUG("SendAndWaitForReplyWithRetry: Success on attempt %d for %s", attempt, timeoutMsg);
                return WR_SUCCESS;
            }

            if (attempt < maxRetries)
            {
                WR_DEBUG("SendAndWaitForReplyWithRetry: Failed on attempt %d, retrying after %d ms", attempt, retryDelayMs);
                std::this_thread::sleep_for(std::chrono::milliseconds(retryDelayMs));
            }
        }

        WR_DEBUG("SendAndWaitForReplyWithRetry: All %d attempts failed for %s", maxRetries, timeoutMsg);
        return WR_ERROR_COMMUNICATION;
    }

    WR_ERROR_TYPE QueryStatus(std::shared_ptr<Device> device)
    {
        if (!device)
        {
            return WR_ERROR_NULL_POINTER;
        }

        if (!device->port || !device->port->IsOpen())
        {
            return WR_ERROR_COMMUNICATION;
        }

        char response[64];
        RETURN_IF_ERROR(SendAndWaitForReplyWithRetry(device, "1500001\n", response, 64,
                                                     200, 500, 3, 300,
                                                     "QueryStatus handshake"));

        WR_INFO("Response: '%s'", response);

        char model[8];
        int firmware;
        int angle;
        float backlash;
        int reverse;
        if (sscanf(response,
                   "WandererRotator%7[^A]A%dA%dA%fA%dA",
                   model,
                   &firmware,
                   &angle,
                   &backlash,
                   &reverse) != 5)
        {
            return WR_ERROR_COMMUNICATION;
        }

        device->modelType = std::string(model);
        device->firmwareVersion = firmware;
        device->mechanicalAngle = angle;
        device->backlash = backlash * 10.0f;
        device->reverseDirection = reverse;

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
        else
        {
            /* Unknown model - fall back to the most common value rather than
             * leaving stepsPerDegree at 0, which would make stepSize = 1/0 (inf)
             * and every move command compute to exactly zero steps. */
            WR_ERROR("QueryStatus: Unknown model '%s', defaulting stepsPerDegree to 1155",
                     device->modelType.c_str());
            device->stepsPerDegree = 1155;
        }

        device->status.stepsPerRevolution = device->stepsPerDegree * 360;
        device->status.stepSize = 1.0f / device->stepsPerDegree;

        /* Set initial position from mechanical angle */
        device->status.position = device->mechanicalAngle / 1000.0f;

        WR_DEBUG("QueryStatus: Successfully parsed, model=%s steps=%d",
                 device->modelType.c_str(), device->stepsPerDegree);

        return WR_SUCCESS;
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
    static void MoveListenerThreadFunc(Device* device)
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
        bool continueLoop = true;

        while (continueLoop && device->listenerRunning)
        {
            continueLoop = false;

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
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    /* Move back by the overshoot amount to land on the actual target */
                    float returnAngle = (device->targetAngle > 0.0f) ? -device->overshootAngle : device->overshootAngle;
                    int command_value = 1000000 + (int)(returnAngle * device->stepsPerDegree);
                    char cmd[16];
                    snprintf(cmd, sizeof(cmd), "%d", command_value);

                    WR_DEBUG("Return move command: %s", cmd);

                    device->port->Flush();
                    device->port->Drain();
                    device->port->ClearRxBuffer();

                    if (device->port->Write((const unsigned char *)cmd, strlen(cmd)))
                    {
                        device->status.moving = 1;
                        continueLoop = true; /* Loop for second phase */
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

        /* Caller must hold device->moveMutex (never g_globalMutex) - the join below
         * can legitimately block for up to ~90s while a prior move completes. */

        /* Stop any existing listener and join it before starting a new one */
        device->listenerRunning = false;
        if (device->moveListenerThread.joinable())
            device->moveListenerThread.join();

        /* Start new listener thread */
        device->listenerRunning = true;
        device->moveListenerThread = std::thread(MoveListenerThreadFunc, device.get());
        WR_DEBUG("StartMoveListener: Listener thread started");
    }

    void StopMoveListener(std::shared_ptr<Device> device)
    {
        if (!device)
        {
            return;
        }

        /* Caller must hold device->moveMutex (never g_globalMutex) - the join below
         * can legitimately block for up to ~90s while a prior move completes. */

        /* Signal listener thread to stop and join it */
        device->listenerRunning = false;
        if (device->moveListenerThread.joinable())
            device->moveListenerThread.join();
        WR_DEBUG("StopMoveListener: Listener stopped");
    }
} /* namespace WandererRotator */
