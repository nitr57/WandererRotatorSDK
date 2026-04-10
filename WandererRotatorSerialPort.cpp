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

#include "WandererRotatorSerialPort.h"
#include "WandererRotatorLogging.h"
#ifdef _WIN32
#include <windows.h>
#include <string>
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#endif
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>

namespace WandererRotator
{
#ifdef _WIN32
    static std::string ToDevicePath(const char *portName)
    {
        // Accept either "COM3" or "\\.\\COM3" style input
        std::string s(portName);
        if (s.rfind("\\\\.", 0) == 0)
            return s;
        if (s.rfind("COM", 0) == 0 && s.size() > 3)
        {
            // COM10+ require \\.\\ prefix
            return std::string("\\\\.\\") + s;
        }
        if (s.rfind("COM", 0) == 0)
            return s;
        return s; // fallback
    }
#endif

    bool SerialPort::Open(const char *portName)
    {
        WR_DEBUG("SerialPort::Open: Attempting to open %s", portName);
#ifdef _WIN32
        std::string device = ToDevicePath(portName);

        HANDLE h = CreateFileA(device.c_str(),
                               GENERIC_READ | GENERIC_WRITE,
                               0,
                               NULL,
                               OPEN_EXISTING,
                               FILE_ATTRIBUTE_NORMAL,
                               NULL);
        if (h == INVALID_HANDLE_VALUE)
        {
            WR_ERROR("SerialPort::Open: Failed to open port %s (errno=%lu)", device.c_str(), GetLastError());
            return false;
        }

        DCB dcb = {0};
        dcb.DCBlength = sizeof(DCB);
        if (!GetCommState(h, &dcb))
        {
            WR_ERROR("SerialPort::Open: GetCommState failed (err=%lu)", GetLastError());
            CloseHandle(h);
            return false;
        }

        // Configure 19200 8N1 NOFLOW
        dcb.BaudRate = CBR_19200;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fNull = FALSE;
        dcb.fBinary = TRUE;
        dcb.fAbortOnError = FALSE;

        if (!SetCommState(h, &dcb))
        {
            WR_ERROR("SerialPort::Open: SetCommState failed (err=%lu)", GetLastError());
            CloseHandle(h);
            return false;
        }

        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = 0;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = 5000;
        SetCommTimeouts(h, &timeouts);

        /* Save handle in fd variable (support 64-bit handles on Win64) */
        fd = (intptr_t)h;

        WR_DEBUG("SerialPort::Open: Opened %s (handle=%p)", device.c_str(), h);
#else
        /* Try to open with retry logic for busy ports */
        int open_fd = -1;
        int attempt = 0;
        int delayMs = retryDelayMs;

        while (attempt < maxRetries && open_fd < 0)
        {
            /* Open without O_NONBLOCK to allow blocking I/O */
            open_fd = open(portName, O_RDWR | O_NOCTTY);
            WR_DEBUG("SerialPort::Open: Attempt %d/%d, open() returned fd=%d", 
                     attempt + 1, maxRetries, open_fd);

            if (open_fd < 0)
            {
                int last_errno = errno;
                /* EBUSY means resource is busy, worth retrying */
                /* EAGAIN means resource temporarily unavailable, worth retrying */
                if ((last_errno == EBUSY || last_errno == EAGAIN || last_errno == EACCES) 
                    && attempt < maxRetries - 1)
                {
                    WR_DEBUG("SerialPort::Open: Port busy/unavailable (errno=%d), retrying in %dms...",
                            last_errno, delayMs);
                    std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
                    delayMs = (delayMs * 3) / 2; /* 1.5x exponential backoff */
                    attempt++;
                    continue;
                }
                else
                {
                    WR_ERROR("SerialPort::Open: Failed to open port %s (errno=%d, attempt %d/%d)", 
                             portName, last_errno, attempt + 1, maxRetries);
                    return false;
                }
            }
        }

        if (open_fd < 0)
        {
            WR_ERROR("SerialPort::Open: Failed to open port %s after %d retries", portName, maxRetries);
            return false;
        }

        fd = (intptr_t)open_fd;

        struct termios tty;
        if (tcgetattr(open_fd, &tty) != 0)
        {
            WR_ERROR("SerialPort::Open: tcgetattr failed (errno=%d)", errno);
            close(open_fd);
            fd = -1;
            return false;
        }

        /* Set 19200 baud rate */
        cfsetispeed(&tty, B19200);
        cfsetospeed(&tty, B19200);

        /* Control Mode Flags (c_cflag) */
        tty.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | HUPCL | CRTSCTS);
        /* Clear: 
         *   CSIZE    - Character size mask
         *   CSTOPB   - Two stop bits (we want one)
         *   PARENB   - Parity enable (we want no parity)
         *   PARODD   - Odd parity (we want even/none)
         *   HUPCL    - Hang up on last close
         *   CRTSCTS  - RTS/CTS flow control (we don't need it)
         */

        tty.c_cflag |= (CLOCAL | CREAD);
        /* Set:
         *   CLOCAL   - Ignore modem control lines, local connection
         *   CREAD    - Enable receiving characters
         */

        tty.c_cflag |= CS8;     /* 8 bit data width */
        tty.c_cc[VMIN] = 0;     /* Minimum characters to read (non-blocking) */
        tty.c_cc[VTIME] = 0;    /* Read timeout in deciseconds (0 = none, we use select()) */

        /* Input Mode Flags (c_iflag) - Process incoming data */
        tty.c_iflag &= ~(PARMRK | ISTRIP | IGNCR | ICRNL | INLCR | IXOFF | IXON | IXANY);
        /* Clear:
         *   PARMRK   - Mark parity errors with special sequence
         *   ISTRIP   - Strip input to 7 bits
         *   IGNCR    - Ignore carriage return
         *   ICRNL    - Map CR to NL on input
         *   INLCR    - Map NL to CR on input
         *   IXOFF    - Enable software flow control (input)
         *   IXON     - Enable software flow control (output)
         *   IXANY    - Allow any character to restart output
         */

        tty.c_iflag |= INPCK | IGNPAR | IGNBRK;
        /* Set:
         *   INPCK    - Enable parity checking
         *   IGNPAR   - Ignore characters with parity errors
         *   IGNBRK   - Ignore break characters
         */

        /* Output Mode Flags (c_oflag) - Process outgoing data */
        tty.c_oflag &= ~(OPOST | ONLCR);
        /* Clear:
         *   OPOST    - Enable post-processing of output
         *   ONLCR    - Map NL to CR-NL on output
         * This makes output raw, no character translation
         */

        /* Local Mode Flags (c_lflag) - Control terminal behavior */
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN | TOSTOP);
        /* Clear:
         *   ICANON   - Canonical (line-buffered) input mode
         *   ECHO     - Echo input characters
         *   ECHOE    - Echo erase characters
         *   ISIG     - Enable signal generation (Ctrl+C, etc.)
         *   IEXTEN   - Enable implementation-defined extensions
         *   TOSTOP   - Send SIGSTOP when background process writes to terminal
         */

        tty.c_lflag |= NOFLSH;
        /* Set:
         *   NOFLSH   - Don't flush I/O buffers on signal
         */

        tcflush((int)fd, TCIOFLUSH);

        if (tcsetattr((int)fd, TCSANOW, &tty) != 0)
        {
            WR_ERROR("SerialPort::Open: tcsetattr failed (errno=%d)", errno);
            close((int)fd);
            fd = -1;
            return false;
        }
        WR_DEBUG("SerialPort::Open: tcsetattr succeeded");

        tcflush(fd, TCIOFLUSH);

        /* Allow the USB-serial adapter and MCU to settle after port open.
         * CH340-based devices may trigger a microcontroller reset on the first
         * open after a fresh USB plug-in. The bootloader typically needs
         * ~200-500ms before the application firmware starts responding. */
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        tcflush(fd, TCIOFLUSH);

        WR_DEBUG("SerialPort::Open: Successfully opened %s (fd=%d)", portName, (int)fd);
#endif
        return true;
    }

    void SerialPort::Close()
    {
        if (fd != -1)
        {
#ifdef _WIN32
            HANDLE h = (HANDLE)fd;
            CloseHandle(h);
#else
            close((int)fd);
#endif
            fd = -1;
        }

        /* Clear the receive buffer when port is closed to prevent stale data from
           affecting subsequent reads if the same port is reopened */
        {
            std::lock_guard<std::mutex> lock(rxMutex);
            rxBuffer.clear();
        }
    }

    bool SerialPort::Write(const unsigned char *data, int len)
    {
        if (fd == -1)
        {
            return false;
        }

#ifdef _WIN32
        HANDLE h = (HANDLE)fd;
        DWORD written = 0;
        if (!WriteFile(h, data, (DWORD)len, &written, NULL))
        {
            WR_ERROR("SerialPort::Write (win): WriteFile failed (err=%lu)", GetLastError());
            return false;
        }
#else
        int written = write((int)fd, data, len);
        WR_DEBUG("Write: fd=%d, wrote %d/%d bytes", (int)fd, written, len);
#endif

        /* Wait for all data to be sent */
        Drain();
        return (int)written == len;
    }

    int SerialPort::Read(unsigned char *buf, int maxlen, char stop_char, int timeoutMs)
    {
        if (fd == -1 || maxlen <= 1)
            return 0;

#ifdef _WIN32
        HANDLE h = (HANDLE)fd;
#endif
        auto start = std::chrono::high_resolution_clock::now();

        /* Temporary read buffer */
        unsigned char tmp[512];

        while (true)
        {
            /* Check remaining time */
            auto elapsed = std::chrono::high_resolution_clock::now() - start;
            int elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
            int remainingMs = timeoutMs - elapsedMs;
            if (remainingMs <= 0)
                break;

            /* First, check if we already have a complete message in rxBuffer */
            {
                std::lock_guard<std::mutex> lock(rxMutex);
                auto pos = rxBuffer.find(stop_char);
                if (pos != std::string::npos)
                {
                    /* Found a complete message */
                    size_t copyLen = std::min((size_t)maxlen - 1, pos + 1);
                    memcpy(buf, rxBuffer.data(), copyLen);
                    buf[copyLen] = '\0';

                    /* Remove the message from rxBuffer, keep anything after the stop_char */
                    rxBuffer.erase(0, copyLen);
                    return (int)copyLen;
                }
            }
#ifdef _WIN32
            /* Try to read more data from the port */
            DWORD toRead = 0;
            COMSTAT status;
            DWORD errors = 0;
            if (ClearCommError(h, &errors, &status))
            {
                toRead = status.cbInQue;
            }

            if (toRead == 0)
            {
                // No data available, wait a bit and retry
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            DWORD chunk = std::min((DWORD)sizeof(tmp), toRead);
            DWORD n = 0;
            if (!ReadFile(h, tmp, chunk, &n, NULL))
            {
                DWORD err = GetLastError();
                if (err == ERROR_IO_PENDING)
                    continue;
                WR_ERROR("SerialPort::Read: ReadFile failed (err=%lu)", err);
                break;
            }
#else
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET((int)fd, &readfds);

            struct timeval tv;
            tv.tv_sec = remainingMs / 1000;
            tv.tv_usec = (remainingMs % 1000) * 1000;

            int selectResult = select((int)fd + 1, &readfds, NULL, NULL, &tv);
            if (selectResult <= 0)
                break; /* timeout or error */

            /* Find how many bytes are available to read */
            int avail = 0;
            if (ioctl((int)fd, FIONREAD, &avail) < 0)
            {
                avail = 0;
            }

            int toRead = std::min(avail > 0 ? avail : 1, (int)sizeof(tmp));
            toRead = std::min(toRead, maxlen - 1); /* avoid exceeding caller buffer */

            ssize_t n = ::read((int)fd, tmp, toRead);
            if (n < 0)
            {
                if (errno == EINTR || errno == EAGAIN)
                    continue;
                break;
            }
#endif
            if (n == 0)
                continue;

            /* Append to rxBuffer and check for stop_char */
            {
                std::lock_guard<std::mutex> lock(rxMutex);
                rxBuffer.append((const char *)tmp, n);
            }

            /* Keep looping until we find stop_char or timeout */
        }

        /* On timeout, return any buffered partial data (if available) */
        {
            std::lock_guard<std::mutex> lock(rxMutex);
            size_t copyLen = std::min((size_t)maxlen - 1, rxBuffer.size());
            if (copyLen > 0)
            {
                memcpy(buf, rxBuffer.data(), copyLen);
                buf[copyLen] = '\0';
                rxBuffer.erase(0, copyLen);
                return (int)copyLen;
            }
        }

        buf[0] = '\0';
        return 0;
    }

    void SerialPort::Flush()
    {
        if (fd != -1)
        {
#ifdef _WIN32
            HANDLE h = (HANDLE)fd;
            PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
#else
            tcflush((int)fd, TCIOFLUSH);
#endif
        }
    }

    void SerialPort::Drain()
    {
        if (fd != -1)
        {
#ifdef _WIN32
            HANDLE h = (HANDLE)fd;
            // Wait briefly; FlushFileBuffers already used after write
            FlushFileBuffers(h);
#else
            tcdrain((int)fd);
#endif
        }
    }

    void SerialPort::ClearRxBuffer()
    {
        std::lock_guard<std::mutex> lock(rxMutex);
        rxBuffer.clear();
    }

} /* namespace WandererRotator */
