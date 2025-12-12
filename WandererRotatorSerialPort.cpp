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

#include "WandererRotatorSerialPort.h"
#include "WandererRotatorLogging.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstring>

namespace WandererRotator
{
    bool SerialPort::Open(const char *portName)
    {
        WR_DEBUG("SerialPort::Open: Attempting to open %s", portName);

        /* Open without O_NONBLOCK to allow blocking I/O */
        fd = open(portName, O_RDWR | O_NOCTTY);
        WR_DEBUG("SerialPort::Open: open() returned fd=%d", fd);

        if (fd < 0)
        {
            WR_ERROR("SerialPort::Open: Failed to open port %s (errno=%d)", portName, errno);
            return false;
        }

        struct termios tty;
        if (tcgetattr(fd, &tty) != 0)
        {
            WR_ERROR("SerialPort::Open: tcgetattr failed (errno=%d)", errno);
            close(fd);
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

        tcflush(fd, TCIOFLUSH);

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            WR_ERROR("SerialPort::Open: tcsetattr failed (errno=%d)", errno);
            close(fd);
            fd = -1;
            return false;
        }
        WR_DEBUG("SerialPort::Open: tcsetattr succeeded");

        tcflush(fd, TCIOFLUSH);
        WR_DEBUG("SerialPort::Open: Successfully opened %s (fd=%d)", portName, fd);
        return true;
    }

    void SerialPort::Close()
    {
        if (fd >= 0)
        {
            close(fd);
            fd = -1;
        }
    }

    bool SerialPort::Write(const unsigned char *data, int len)
    {
        if (fd < 0)
        {
            return false;
        }
        int written = write(fd, data, len);
        WR_DEBUG("Write: fd=%d, wrote %d/%d bytes", fd, written, len);
        /* Wait for all data to be sent */
        tcdrain(fd);
        return written == len;
    }

    int SerialPort::Read(unsigned char *buf, int maxlen, char stop_char, int timeoutMs)
    {
        int bytesRead = 0;
        auto startTime = std::chrono::high_resolution_clock::now();

        while (bytesRead < maxlen - 1)
        {
            /* Check timeout */
            auto elapsed = std::chrono::high_resolution_clock::now() - startTime;
            int elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
            if (elapsedMs >= timeoutMs)
                break;

            /* Set up select with remaining timeout */
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(fd, &readfds);

            int remainingMs = timeoutMs - elapsedMs;
            struct timeval tv;
            tv.tv_sec = remainingMs / 1000;
            tv.tv_usec = (remainingMs % 1000) * 1000;

            int selectResult = select(fd + 1, &readfds, NULL, NULL, &tv);
            if (selectResult <= 0)
                break;

            /* Read one byte at a time to avoid reading past stop character */
            ssize_t n = read(fd, buf + bytesRead, 1);
            if (n <= 0)
                break;

            if (buf[bytesRead] == stop_char)
            {
                bytesRead++;
                buf[bytesRead] = '\0';
                return bytesRead;
            }

            bytesRead++;
        }

        buf[bytesRead] = '\0';
        return bytesRead;
    }

} /* namespace WandererRotator */
