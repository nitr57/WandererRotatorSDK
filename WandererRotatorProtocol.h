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

#include "WandererRotatorSDK.h"
#include "WandererRotatorDevice.h"
#include <locale.h>

namespace WandererRotator
{
    /**
     * While in scope, numeric conversions on this thread (sscanf/snprintf %f)
     * use '.' as the decimal separator, like the device does - whatever
     * LC_NUMERIC the host application has set (e.g. "," with a German locale).
     */
    class ClassicNumericLocale
    {
    public:
        ClassicNumericLocale() : previous(uselocale(Classic())) {}
        ~ClassicNumericLocale() { uselocale(previous); }
        ClassicNumericLocale(const ClassicNumericLocale &) = delete;
        ClassicNumericLocale &operator=(const ClassicNumericLocale &) = delete;

    private:
        static locale_t Classic()
        {
            static locale_t classic = newlocale(LC_NUMERIC_MASK, "C", (locale_t)0);
            return classic;
        }
        locale_t previous;
    };

    /**
     * Send a command to the device, discarding any unread input first.
     * Caller must hold device->ioMutex (or own the port as the listener).
     *
     * @param device Device to send command to
     * @param command Command string
     * @return WR_SUCCESS if the command was written
     */
    WR_ERROR_TYPE SendCommand(std::shared_ptr<Device> device, const char *command);
    WR_ERROR_TYPE SendAndWaitForReply(std::shared_ptr<Device> device, const char* cmd, char* buffer, int maxLen, int recvTimeoutMs = 500);

    /**
     * Query model, firmware, position, backlash and reverse direction from the
     * device and store them in the Device. Retries missing or unparseable replies.
     */
    WR_ERROR_TYPE QueryStatus(std::shared_ptr<Device> device, int maxAttempts = 3,
                              int recvTimeoutMs = 500, int retryDelayMs = 300);

    /**
     * Handshake used by WRRotatorScan to identify a freshly opened port: asks
     * for the status every 250ms until 1.5s after the port was opened (incl.
     * SerialPort::Open's 500ms settle). A rotator that was just plugged in
     * answers ~450ms after open (it ignores queries while booting); ports that
     * stay silent are released quickly so other programs waiting on them can
     * proceed.
     */
    WR_ERROR_TYPE ProbeStatus(std::shared_ptr<Device> device);

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
     * Send "stop" and wait briefly for the reply it produces if a move was in
     * progress (an idle device does not answer). Safe in any state: unlike
     * numeric commands, "stop" never starts a move. Caller must own the port.
     */
    void StopAndDrain(Device *device);

    /**
     * Convert a mechanical angle in thousandths of a degree (which the device
     * does not wrap) to a position in [0, 360) degrees.
     */
    float MechanicalAngleToPosition(int mechanicalAngle);

    /**
     * Set or clear device->status.moving and wake callers waiting for the
     * device to become idle. Must not be called with stateMutex held.
     */
    void SetMoving(Device *device, bool moving);

    /**
     * Start listening for movement completion messages.
     * Spawns a background thread that reads serial data until movement finishes.
     * Should be called before triggering a move command.
     *
     * @param device Device to listen on
     * @return false if the listener thread could not be started
     */
    bool StartMoveListener(std::shared_ptr<Device> device);

    /**
     * Stop listening for movement completion messages.
     * Terminates the background listener thread.
     * Should be called after movement finishes.
     *
     * @param device Device to stop listening on
     */
    void StopMoveListener(std::shared_ptr<Device> device);

} /* namespace WandererRotator */

#endif /* WANDERER_ROTATOR_PROTOCOL_H */
