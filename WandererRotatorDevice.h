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
     */
    struct Device
    {
        std::shared_ptr<SerialPort> port;
        std::string portName;
        std::string modelType;
        std::atomic<bool> isOpen{false}; /* Written outside g_globalMutex by Close, so must be atomic */
        int firmwareVersion = 0;
        int mechanicalAngle = 0;
        int backlash = 0;
        int reverseDirection = 0;
        int stepsPerDegree = 0;
        float lastRotated = 0.0f;
        int overshoot = 0;
        float overshootAngle = 0.0f;    /* Backlash overshoot angle in degrees */
        int overshootDirection = 0;     /* 0 - normal, 1 - reverse */
        int overshooting = 0;           /* 0 - not in overshoot, 1 - in first phase, 2 - awaiting return */
        float targetAngle = 0.0f;       /* Target angle for second phase of overshoot */

        struct RotatorConfig
        {
            int reverseDirection = 0;
            int stepRate = 50;
        } rotator;

        struct RotatorStatus
        {
            float position = 0.0f;
            int moving = 0;
            int stepsPerRevolution = 0;
            float stepSize = 0.0f;
        } status;

        /* Listener thread state */
        std::atomic<bool> listenerRunning{false};
        std::mutex listenerMutex;
        std::thread moveListenerThread;

        /* Serializes Move/MoveTo/Close's handling of moveListenerThread (i.e. the
         * SendCommand + StartMoveListener/StopMoveListener sequence) on this device.
         * The rotator can take up to ~90s to complete a move and stays silent on
         * the wire the whole time, so StartMoveListener's join on the previous
         * listener can legitimately take that long - callers must hold this
         * per-device lock (never g_globalMutex) while it does, so other devices
         * and quick per-device calls (StopMove, GetStatus, ...) are never blocked
         * by it. */
        std::mutex moveMutex;

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
