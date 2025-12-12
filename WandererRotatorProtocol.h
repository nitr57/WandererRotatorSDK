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

#ifndef WANDERER_ROTATOR_PROTOCOL_H
#define WANDERER_ROTATOR_PROTOCOL_H

#include "WandererRotatorDevice.h"

namespace WandererRotator
{
    /**
     * Send a command to the device and ignore the response.
     *
     * @param device Device to send command to
     * @param command Command string (no newline)
     * @param timeoutMs Timeout in milliseconds (default 3000ms)
     * @return true if command succeeded
     */
    bool SendCommand(std::shared_ptr<Device> device, const char *command, int timeoutMs = 3000);

    bool QueryStatus(std::shared_ptr<Device> device);

    /**
     * Convert backlash value to command value.
     * Command format: 10*x + 1600000
     *
     * @param backlash Backlash in degrees
     * @return Command value
     */
    int BacklashToCommand(float backlash);

    /**
     * Get reverse direction command string.
     *
     * @param reverse 0 for normal, 1 for reversed
     * @return Command string ("1700000" or "1700001")
     */
    const char *ReverseDirectionToCommand(int reverse);

    /**
     * Start listening for movement completion messages.
     * Spawns a background thread that reads serial data until movement finishes.
     * Should be called before triggering a move command.
     *
     * @param device Device to listen on
     */
    void StartMoveListener(std::shared_ptr<Device> device);

    /**
     * Stop listening for movement completion messages.
     * Terminates the background listener thread.
     * Should be called after movement finishes.
     *
     * @param device Device to stop listening on
     */
    void StopMoveListener(std::shared_ptr<Device> device);
    bool QueryHandshake(std::shared_ptr<Device> device);

} /* namespace WandererRotator */

#endif /* WANDERER_ROTATOR_PROTOCOL_H */
