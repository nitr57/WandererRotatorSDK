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

#include "WandererRotatorLogging.h"
#include <cstdio>
#include <cstdarg>
#include <ctime>

namespace WandererRotator
{
	/* ============================================================================
	 * LOGGING IMPLEMENTATION
	 * ============================================================================ */

	const char *WRGetTimestamp()
	{
		static char timestamp[20];
		time_t now = time(nullptr);
		struct tm *timeinfo = localtime(&now);
		strftime(timestamp, sizeof(timestamp), "%H:%M:%S", timeinfo);
		return timestamp;
	}

	void WRLogDebug(const char *fmt, ...)
	{
		va_list args;
		va_start(args, fmt);
		if (WandererRotator::WR_TIMESTAMP_ENABLED)
		{
			fprintf(stderr, "[%s] [WR_DEBUG] ", WRGetTimestamp());
		}
		else
		{
			fprintf(stderr, "[WR_DEBUG] ");
		}
		vfprintf(stderr, fmt, args);
		fprintf(stderr, "\n");
		va_end(args);
	}

	void WRLogInfo(const char *fmt, ...)
	{
		va_list args;
		va_start(args, fmt);
		if (WandererRotator::WR_TIMESTAMP_ENABLED)
		{
			fprintf(stderr, "[%s] [WR_INFO] ", WRGetTimestamp());
		}
		else
		{
			fprintf(stderr, "[WR_INFO] ");
		}
		vfprintf(stderr, fmt, args);
		fprintf(stderr, "\n");
		va_end(args);
	}

	void WRLogError(const char *fmt, ...)
	{
		va_list args;
		va_start(args, fmt);
		if (WandererRotator::WR_TIMESTAMP_ENABLED)
		{
			fprintf(stderr, "[%s] [WR_ERROR] ", WRGetTimestamp());
		}
		else
		{
			fprintf(stderr, "[WR_ERROR] ");
		}
		vfprintf(stderr, fmt, args);
		fprintf(stderr, "\n");
		va_end(args);
	}

} /* namespace WandererRotator */
