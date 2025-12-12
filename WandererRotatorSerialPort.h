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

#ifndef WANDERER_ROTATOR_SERIAL_PORT_H
#define WANDERER_ROTATOR_SERIAL_PORT_H

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
		int fd = -1;

	public:
		SerialPort() {}
		~SerialPort() { Close(); }

		/**
		 * Open a serial port device.
		 * @param portName Device path (e.g., "/dev/ttyUSB0")
		 * @return true if successfully opened and configured
		 */
		bool Open(const char *portName);

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
		 * Uses select() for timeout-based reading. Reads byte-by-byte and
		 * stops when 5 'A' delimiters are found (device response format).
		 *
		 * @param data Buffer to read data into
		 * @param maxLen Maximum number of bytes to read
		 * @param timeoutMs Timeout in milliseconds
		 * @return Number of bytes read (0 on timeout or error)
		 */
		// int Read(unsigned char *data, int maxLen, int timeoutMs);
		int Read(unsigned char *buf, int maxlen, char stop_char, int timeoutMs);

		/**
		 * Check if the serial port is open.
		 * @return true if port is open
		 */
		bool IsOpen() { return fd >= 0; }

		/**
		 * Get the file descriptor for the serial port.
		 * @return File descriptor or -1 if closed
		 */
		int GetFD() { return fd; }
	};

} /* namespace WandererRotator */

#endif /* WANDERER_ROTATOR_SERIAL_PORT_H */
