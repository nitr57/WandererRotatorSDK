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

#ifndef WANDERER_ROTATOR_SERIAL_PORT_H
#define WANDERER_ROTATOR_SERIAL_PORT_H

#include <atomic>
#include <cstdint>
#include <string>
#include <mutex>
#ifndef _WIN32
#include <termios.h>
#endif

/* ============================================================================
 * WANDERER ROTATOR SDK - SERIAL PORT MODULE
 *
 * Low-level serial port communication with select()-based timeout handling.
 * ============================================================================ */

namespace WandererRotator
{
    class SerialPort
    {
    private:
        std::atomic<intptr_t> fd{-1}; /* atomic: IsOpen() is called without the device's ioMutex */
        std::string rxBuffer; /* leftover bytes read from device */
        std::mutex rxMutex;   /* guards rxBuffer */
        int maxRetries = 3;   /* number of retries when port is busy */
        int retryDelayMs = 200; /* delay between retry attempts in milliseconds */

        int ReadImpl(unsigned char *buf, int maxlen, char stop_char, int timeoutMs,
                     bool returnPartial, bool *ioError);

#ifndef _WIN32
        struct termios originalSettings; /* port settings before Open changed them */
        bool haveOriginalSettings = false;
        int inUseLockFd = -1;            /* held lock on the port's in-use marker, see MarkInUse */
#endif

    public:
        SerialPort() {}
        ~SerialPort() { Close(); }

        /**
         * Open a serial port device.
         * 
         * Polls every 50ms while the port is busy (e.g. another tool is scanning
         * it), for up to maxRetries * retryDelayMs in total. Permission errors are
         * only retried briefly, in case udev has not applied its rules yet.
         * 
         * @param portName Device path (e.g., "/dev/ttyUSB0" or "COM3")
         * @return true if successfully opened and configured
         */
        bool Open(const char *portName);

        /**
         * Set the total time budget for opening a busy port: maxRetries * retryDelayMs.
         * @param maxRetries Multiplier for the budget (default: 3)
         * @param retryDelayMs Milliseconds per unit of the budget (default: 200)
         */
        void SetRetryParams(int maxRetries, int retryDelayMs)
        {
            this->maxRetries = maxRetries > 0 ? maxRetries : 1;
            this->retryDelayMs = retryDelayMs > 0 ? retryDelayMs : 1;
        }

        /**
         * Close the serial port.
         */
        void Close();

        /**
         * Mark the port as in use (opened for a device, not just probed) until it
         * is closed: holds an advisory lock on /tmp/ch34x-<port name>.lock (e.g.
         * /tmp/ch34x-ttyUSB0.lock). Open() fails at once instead of waiting for a
         * busy port whose marker is held. The convention is shared with other
         * SDKs for CH34x-based devices, so their scans skip ports in use as well.
         * The lock is released by Close() or when the process exits.
         * @param portName Device path the port was opened with
         */
        void MarkInUse(const char *portName);

        /**
         * Put back the port settings found when it was opened (baud rate, mode),
         * except that hang-up-on-close stays off so closing does not toggle DTR.
         * Used after probing a port that turned out to belong to another device,
         * which may be in use by another program.
         */
        void RestoreOriginalSettings();

        /**
         * Check whether the port has been hung up, e.g. because the USB device
         * was unplugged. Does not consume any data.
         * @return true if the port is open but no longer usable
         */
        bool IsHungUp();

        /**
         * Write data to the serial port.
         * @param data Buffer containing data to write
         * @param len Number of bytes to write
         * @return true if all bytes were successfully written
         */
        bool Write(const unsigned char *data, int len);

        /**
         * Read data from the serial port with timeout.
         *
         * Uses select() for timeout-based reading on POSIX. Reads byte-by-byte and
         * stops when stop_char is found.
         *
         * @param data Buffer to read data into
         * @param maxLen Maximum number of bytes to read
         * @param stop_char Character that terminates a message (e.g. '#')
         * @param timeoutMs Timeout in milliseconds
         * @return Number of bytes read (0 on timeout or error)
         */
        int Read(unsigned char *buf, int maxlen, char stop_char, int timeoutMs);

        /**
         * Read one complete message (terminated by stop_char) with timeout.
         *
         * Unlike Read(), a timeout never returns partial data - incomplete bytes
         * stay buffered for the next call - so callers can poll in short slices.
         *
         * @param ioError Set to true if the port reported an error or hangup
         *                (e.g. USB device unplugged); false on a plain timeout
         * @return Number of bytes read including stop_char, 0 on timeout or error
         */
        int ReadMessage(unsigned char *buf, int maxlen, char stop_char, int timeoutMs, bool &ioError);

        /**
         * Flush any pending input/output on the port (cross-platform)
         */
        void Flush();

        /**
         * Ensure transmitted data has been physically sent (drain transmit buffer)
         */
        void Drain();

        /**
         * Clear the receive buffer of any stale data
         */
        void ClearRxBuffer();

        /**
         * Check if the serial port is open.
         * @return true if port is open
         */
        bool IsOpen() { return fd != -1; }

        /**
         * Get the file descriptor for the serial port.
         * @return File descriptor or -1 if closed
         */
        intptr_t GetFD() { return fd; }
    };

} /* namespace WandererRotator */

#endif /* WANDERER_ROTATOR_SERIAL_PORT_H */
