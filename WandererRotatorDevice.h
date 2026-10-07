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

#ifndef WANDERER_ROTATOR_DEVICE_H
#define WANDERER_ROTATOR_DEVICE_H

#include "WandererRotatorSerialPort.h"
#include <memory>
#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <deque>

#define RETURN_IF_ERROR(fct)        \
    {                               \
        WR_ERROR_TYPE stat = fct;   \
        if (stat != WR_SUCCESS)     \
        {                           \
            return stat;            \
        }                           \
    }

namespace WandererRotator
{
    /**
     * Device represents a Wanderer Rotator device with its current state.
     *
     * Locking (acquire in this order, never the reverse):
     *   g_globalMutex -> ioMutex -> pendingMovesMutex -> stateMutex
     * g_globalMutex only guards g_devices lookups and is never held while
     * taking a device lock. stateMutex is a leaf lock and is never held during
     * serial I/O.
     */
    struct Device
    {
        std::shared_ptr<SerialPort> port;
        std::string portName;
        std::atomic<bool> isOpen{false};

        /* Device info and configuration - guarded by stateMutex */
        std::string modelType;
        int firmwareVersion = 0;
        int backlash = 0;               /* Backlash in tenths of a degree */
        int reverseDirection = 0;
        int stepsPerDegree = 0;
        int overshoot = 0;
        float overshootAngle = 0.0f;    /* Backlash overshoot angle in degrees */
        int overshootDirection = 0;     /* 0 - normal, 1 - reverse */

        std::atomic<int> mechanicalAngle{0}; /* Written by the listener thread */
        float lastRotated = 0.0f;            /* Only used by the listener thread */

        struct RotatorStatus
        {
            std::atomic<float> position{0.0f}; /* Written by the listener thread */
            std::atomic<int> moving{0};        /* Cleared through SetMoving() so waiters are notified */
            int stepsPerRevolution = 0;        /* Guarded by stateMutex */
            float stepSize = 0.0f;             /* Guarded by stateMutex */
        } status;

        /* Guards the device info/config fields above. Also the mutex for
         * moveDoneCv, which is notified whenever status.moving is cleared or
         * the listener thread exits. */
        std::mutex stateMutex;
        std::condition_variable moveDoneCv;

        /* Owns the serial port: held for every command exchange (Open, Close,
         * Move, MoveTo, SetConfig, SyncPosition, StopMove). While a move is in
         * progress the listener thread reads the port without it; calls that
         * need the port wait for the move to finish first (see LockIdle in
         * WandererRotatorSDK.cpp), so only StopMove's "stop" write and Close
         * ever overlap with the listener. */
        std::mutex ioMutex;

        /* Listener thread state */
        std::atomic<bool> listenerRunning{false};
        std::thread moveListenerThread;

        /* Steps of the move segment currently running, for the reply timeout */
        std::atomic<int> currentSegmentSteps{0};

        /* Steady-clock time (ms) of a StopMove issued during the current move,
         * 0 if none. The listener then waits only briefly for the final reply. */
        std::atomic<long long> stopRequestedMs{0};

        /* Move segments (in steps) still to be sent by the listener once the
         * current segment completes: the remaining chunks of a move too large
         * for a single command, followed by the overshoot return move. Guarded
         * by pendingMovesMutex, which is also held while a segment or "stop" is
         * written so StopMove can never race a segment out after clearing it. */
        std::deque<int> pendingMoves;
        std::mutex pendingMovesMutex;

        ~Device()
        {
            listenerRunning = false;
            if (moveListenerThread.joinable())
                moveListenerThread.join();
        }
    };

    /**
     * Global device registry mapping device IDs to Device objects.
     */
    extern std::map<int, std::shared_ptr<Device>> g_devices;

    /**
     * Global mutex protecting access to g_devices.
     */
    extern std::mutex g_globalMutex;

} /* namespace WandererRotator */

#endif /* WANDERER_ROTATOR_DEVICE_H */
