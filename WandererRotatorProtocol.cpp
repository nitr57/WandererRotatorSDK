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
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <system_error>
#include <thread>

namespace WandererRotator
{
    static long long SteadyNowMs()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    float MechanicalAngleToPosition(int mechanicalAngle)
    {
        float position = fmodf(mechanicalAngle / 1000.0f, 360.0f);
        return position < 0.0f ? position + 360.0f : position;
    }

    void StopAndDrain(Device *device)
    {
        const char *cmd = "stop";
        device->port->Drain();
        device->port->Write((const unsigned char *)cmd, strlen(cmd));

        /* A moving device answers within ~50ms with the rotation so far */
        unsigned char reply[64];
        if (device->port->Read(reply, sizeof(reply), '\r', 300) > 0)
        {
            WR_DEBUG("StopAndDrain: Device was moving, stop reply: %s", (const char *)reply);
        }
        device->port->Flush();
        device->port->ClearRxBuffer();
    }

    void SetMoving(Device *device, bool moving)
    {
        std::lock_guard<std::mutex> lock(device->stateMutex);
        device->status.moving = moving ? 1 : 0;
        device->moveDoneCv.notify_all();
    }

    WR_ERROR_TYPE SendCommand(std::shared_ptr<Device> device, const char *command)
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

    static int ReadResponse(std::shared_ptr<Device> device, char *response, int maxLen, int timeoutMs)
    {
        if(!device || !device->port || !device->port->IsOpen())
        {
            WR_DEBUG("ReadResponse: device=%p, port=%p, isOpen=%d",
                     device.get(), device ? device->port.get() : nullptr,
                     device && device->port ? device->port->IsOpen() : 0);
            return 0;
        }

        WR_DEBUG("ReadResponse: Reading...");
        int len = device->port->Read((unsigned char *)response, maxLen, '\r', timeoutMs);
        if(len == 0)
        {
            WR_DEBUG("ReadResponse: Read failed");
            return 0;
        }

        return len;
    }

    WR_ERROR_TYPE SendAndWaitForReply(std::shared_ptr<Device> device, const char* cmd, char* buffer, int maxLen, int recvTimeoutMs)
    {
        // Send command
        RETURN_IF_ERROR(SendCommand(device, cmd));

        // Read response
        if (!ReadResponse(device, buffer, maxLen, recvTimeoutMs))
        {
            WR_DEBUG("Failed to receive response");
            return WR_ERROR_COMMUNICATION;
        }

        return WR_SUCCESS;
    }

    WR_ERROR_TYPE ProbeStatus(std::shared_ptr<Device> device)
    {
        return QueryStatus(device, 4, 250, 0);
    }

    WR_ERROR_TYPE QueryStatus(std::shared_ptr<Device> device, int maxAttempts, int recvTimeoutMs, int retryDelayMs)
    {
        if (!device)
        {
            return WR_ERROR_NULL_POINTER;
        }

        if (!device->port || !device->port->IsOpen())
        {
            return WR_ERROR_COMMUNICATION;
        }

        /* Retry both missing and unparseable (e.g. truncated) replies */
        for (int attempt = 1; attempt <= maxAttempts; ++attempt)
        {
            if (attempt > 1 && retryDelayMs > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(retryDelayMs));
            }

            char response[64];
            if (SendAndWaitForReply(device, "1500001\n", response, sizeof(response), recvTimeoutMs) != WR_SUCCESS)
            {
                WR_DEBUG("QueryStatus: No reply on attempt %d/%d", attempt, maxAttempts);
                continue;
            }

            WR_INFO("Response: '%s'", response);

            char model[32];
            int firmware;
            int angle;
            float backlash;
            int reverse;
            ClassicNumericLocale numericLocale;
            if (sscanf(response,
                       "WandererRotator%31[^A]A%dA%dA%fA%dA",
                       model,
                       &firmware,
                       &angle,
                       &backlash,
                       &reverse) != 5)
            {
                float rotated;
                int mech;
                if (sscanf(response, "%fA%dA", &rotated, &mech) == 2)
                {
                    /* The device was still moving: the firmware ends a move when
                     * any command arrives and answers with the move result - and
                     * a numeric command received mid-move starts a new, unwanted
                     * move. Stop that right away before asking again. */
                    WR_ERROR("QueryStatus: Device was moving, stopping it");
                    StopAndDrain(device.get());
                }
                else
                {
                    WR_DEBUG("QueryStatus: Unparseable reply on attempt %d/%d", attempt, maxAttempts);
                }
                continue;
            }

            /* Set steps per degree based on model type */
            std::string modelType(model);
            int stepsPerDegree;
            if (modelType.find("Mini") != std::string::npos)
            {
                stepsPerDegree = 1142;
            }
            else if (modelType.find("Lite") != std::string::npos)
            {
                stepsPerDegree = modelType.find("V2") != std::string::npos ? 1199 : 1155;
            }
            else
            {
                /* Unknown model - fall back to the most common value rather than
                 * leaving stepsPerDegree at 0, which would make stepSize = 1/0 (inf)
                 * and every move command compute to exactly zero steps. */
                WR_ERROR("QueryStatus: Unknown model '%s', defaulting stepsPerDegree to 1155",
                         modelType.c_str());
                stepsPerDegree = 1155;
            }

            {
                std::lock_guard<std::mutex> lock(device->stateMutex);
                device->modelType = modelType;
                device->firmwareVersion = firmware;
                device->backlash = (int)lroundf(backlash * 10.0f);
                device->reverseDirection = reverse;
                device->stepsPerDegree = stepsPerDegree;
                device->status.stepsPerRevolution = stepsPerDegree * 360;
                device->status.stepSize = 1.0f / stepsPerDegree;
            }

            device->mechanicalAngle = angle;
            device->status.position = MechanicalAngleToPosition(angle);

            WR_DEBUG("QueryStatus: Successfully parsed, model=%s steps=%d",
                     modelType.c_str(), stepsPerDegree);

            return WR_SUCCESS;
        }

        WR_DEBUG("QueryStatus: All %d attempts failed", maxAttempts);
        return WR_ERROR_COMMUNICATION;
    }

    int BacklashToCommand(float backlash)
    {
        return (int)lroundf(backlash * 10.0f) + 1600000;
    }

    const char *ReverseDirectionToCommand(int reverse)
    {
        return reverse ? "1700001\n" : "1700000\n";
    }

    /* Longest a blocked listener read can delay StopMoveListener (Close, a new
     * Move, process exit) from noticing listenerRunning was cleared. */
    static const int kListenerSliceMs = 200;

    /* How long the listener keeps waiting for the final position after a
     * StopMove before falling back to querying the device. */
    static const int kStopReplyTimeoutMs = 3000;

    /* Time to wait for a move segment to complete: a Lite moves ~4300 steps/s
     * (measured: 90 degrees in 24.3s), so allow half that speed plus some
     * slack, and never less than the original 90s. The largest segment
     * (499999 steps, ~433 degrees) takes ~2 minutes. */
    static int MoveReplyTimeoutMs(int steps)
    {
        long long ms = 10000 + std::llabs((long long)steps) * 1000 / 2000;
        return (int)std::max<long long>(90000, ms);
    }

    /* Read one 'A'-terminated message, polling in short slices so a stop request
     * is noticed promptly instead of after the full timeout. */
    static bool ListenerReadMessage(Device *device, char *buffer, int len, int timeoutMs, bool &ioError)
    {
        long long deadlineMs = SteadyNowMs() + timeoutMs;
        ioError = false;

        while (device->listenerRunning)
        {
            long long stopMs = device->stopRequestedMs;
            long long effectiveDeadlineMs = stopMs ? std::min(deadlineMs, stopMs + kStopReplyTimeoutMs) : deadlineMs;

            long long remainingMs = effectiveDeadlineMs - SteadyNowMs();
            if (remainingMs <= 0)
                return false;

            int sliceMs = (int)std::min<long long>(remainingMs, kListenerSliceMs);
            if (device->port->ReadMessage((unsigned char *)buffer, len, 'A', sliceMs, ioError) > 0)
                return true;
            if (ioError)
                return false;
        }
        return false;
    }

    /* Send the next queued move segment (remaining chunk of a large move or the
     * overshoot return move). Returns false when there is nothing left to send. */
    static bool SendNextPendingMove(Device *device)
    {
        {
            std::lock_guard<std::mutex> lock(device->pendingMovesMutex);
            if (device->pendingMoves.empty())
                return false;
        }

        /* Small delay before the next segment */
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        /* Held while writing, so a concurrent StopMove either cancels this segment
         * before it is sent or sends "stop" after it. */
        std::lock_guard<std::mutex> lock(device->pendingMovesMutex);
        if (device->pendingMoves.empty() || !device->listenerRunning)
            return false;

        int steps = device->pendingMoves.front();
        device->pendingMoves.pop_front();
        device->currentSegmentSteps = steps;

        /* Newline-terminated so the device starts at once instead of waiting
         * ~1s for more input */
        char cmd[16];
        snprintf(cmd, sizeof(cmd), "%d\n", 1000000 + steps);
        WR_DEBUG("MoveListener: Sending next move segment: %s", cmd);

        device->port->Flush();
        device->port->Drain();
        device->port->ClearRxBuffer();

        if (!device->port->Write((const unsigned char *)cmd, strlen(cmd)))
        {
            WR_ERROR("Failed to send next movement segment");
            device->pendingMoves.clear();
            return false;
        }
        return true;
    }

    /* The listener lost track of the move (no reply, garbled reply or port error).
     * Re-read the position if the device still answers, then clear the moving
     * state so callers polling WRRotatorGetStatus() don't wait forever. */
    static void RecoverFromListenerFailure(Device *device, bool ioError)
    {
        {
            std::lock_guard<std::mutex> lock(device->pendingMovesMutex);
            device->pendingMoves.clear();
        }

        if (!ioError)
        {
            /* The device may still be moving (a very long move, or a garbled
             * reply mid-move). A status query would end that move and start a
             * new, unwanted one, so stop it first. No ioMutex needed - the
             * listener owns the port until it exits (see Device::ioMutex). */
            StopAndDrain(device);

            /* Non-owning reference: the listener must not extend the Device's
             * lifetime, since ~Device joins this thread. */
            std::shared_ptr<Device> ref(device, [](Device *) {});
            if (QueryStatus(ref) != WR_SUCCESS)
            {
                WR_DEBUG("MoveListener: Status query after failed move did not succeed");
            }
        }

        SetMoving(device, false);
    }

    /* Background listener thread body for movement completion */
    static void MoveListenerThreadBody(Device* device)
    {
        ClassicNumericLocale numericLocale;

        WR_DEBUG("MoveListener: Started for device %s", device->portName.c_str());

        char buffer[32];

        if (!device->port || !device->port->IsOpen())
        {
            WR_DEBUG("MoveListener: Port not open, exiting");
            SetMoving(device, false);
        }

        while (device->listenerRunning && device->port && device->port->IsOpen())
        {
            bool ioError = false;
            float rotated;
            int angle;

            /* Read the actual angle moved, then the new position */
            if (!ListenerReadMessage(device, buffer, sizeof(buffer), MoveReplyTimeoutMs(device->currentSegmentSteps), ioError) ||
                sscanf(buffer, "%fA", &rotated) != 1 ||
                !ListenerReadMessage(device, buffer, sizeof(buffer), 3000, ioError) ||
                sscanf(buffer, "%dA", &angle) != 1)
            {
                /* A cleared listenerRunning means we were asked to stop - the caller
                 * owns the device state from here on. */
                if (device->listenerRunning)
                {
                    WR_DEBUG("MoveListener: %s", ioError ? "Port error" : "Timeout or invalid message");
                    RecoverFromListenerFailure(device, ioError);
                }
                break;
            }

            device->lastRotated = rotated;
            device->mechanicalAngle = angle;
            device->status.position = MechanicalAngleToPosition(angle);

            if (!SendNextPendingMove(device))
            {
                if (device->listenerRunning)
                {
                    SetMoving(device, false);
                }
                break;
            }
        }

        /* Mark listener as stopped before exiting and wake callers waiting for
         * the device to become idle */
        {
            std::lock_guard<std::mutex> lock(device->stateMutex);
            device->listenerRunning = false;
            device->moveDoneCv.notify_all();
        }
        WR_DEBUG("MoveListener: Stopped for device %s", device->portName.c_str());
    }

    static void MoveListenerThreadFunc(Device* device)
    {
        try
        {
            MoveListenerThreadBody(device);
        }
        catch (...)
        {
            /* An exception escaping a std::thread would call std::terminate.
             * Release callers waiting for the move to finish. */
            WR_ERROR("MoveListener: unexpected exception, stopping");
            std::lock_guard<std::mutex> lock(device->stateMutex);
            device->status.moving = 0;
            device->listenerRunning = false;
            device->moveDoneCv.notify_all();
        }
    }

    bool StartMoveListener(std::shared_ptr<Device> device)
    {
        if (!device)
        {
            return false;
        }

        /* Caller must hold device->ioMutex (never g_globalMutex). */

        /* Stop any existing listener and join it before starting a new one */
        StopMoveListener(device);

        /* Start new listener thread */
        device->stopRequestedMs = 0;
        device->listenerRunning = true;
        try
        {
            device->moveListenerThread = std::thread(MoveListenerThreadFunc, device.get());
        }
        catch (const std::system_error &e)
        {
            WR_ERROR("StartMoveListener: Failed to start listener thread: %s", e.what());
            device->listenerRunning = false;
            return false;
        }
        WR_DEBUG("StartMoveListener: Listener thread started");
        return true;
    }

    void StopMoveListener(std::shared_ptr<Device> device)
    {
        if (!device)
        {
            return;
        }

        /* Caller must hold device->ioMutex (never g_globalMutex). The listener
         * checks listenerRunning between short read slices, so the join is quick. */

        /* Signal listener thread to stop and join it */
        device->listenerRunning = false;
        if (device->moveListenerThread.joinable())
            device->moveListenerThread.join();
        WR_DEBUG("StopMoveListener: Listener stopped");
    }
} /* namespace WandererRotator */
