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

#include <string>
#include <mutex>

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
        intptr_t fd = -1;
        std::string rxBuffer; /* leftover bytes read from device */
        std::mutex rxMutex;   /* guards rxBuffer */
        int maxRetries = 3;   /* number of retries when port is busy */
        int retryDelayMs = 200; /* delay between retry attempts in milliseconds */

    public:
        SerialPort() {}
        ~SerialPort() { Close(); }

        /**
         * Open a serial port device.
         * 
         * Implements retry logic for handling busy ports (e.g., when other tools
         * are scanning the port). Uses exponential backoff between attempts.
         * 
         * @param portName Device path (e.g., "/dev/ttyUSB0" or "COM3")
         * @return true if successfully opened and configured
         */
        bool Open(const char *portName);

        /**
         * Set retry parameters for opening the serial port.
         * @param maxRetries Maximum number of retry attempts (default: 3)
         * @param retryDelayMs Initial delay between attempts in milliseconds (default: 100)
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
