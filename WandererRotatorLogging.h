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

#ifndef WANDERER_ROTATOR_LOGGING_H
#define WANDERER_ROTATOR_LOGGING_H

/* ============================================================================
 * WANDERER ROTATOR SDK - LOGGING MODULE
 *
 * Compile-time controlled logging system for developers.
 * All logging is disabled in release builds unless explicitly enabled.
 * ============================================================================ */

namespace WandererRotator
{
	/* Compile-time logging configuration */
	static constexpr bool WR_DEBUG_ENABLED = false; /* Disable debug logging by default */
	static constexpr bool WR_INFO_ENABLED = false;	/* Enable info logging */
	static constexpr bool WR_ERROR_ENABLED = true;	/* Enable error logging */
	static constexpr bool WR_TIMESTAMP_ENABLED = true; /* Enable timestamps in logs */

/* Logging macros - use these throughout the SDK */
#define WR_DEBUG(fmt, ...)                                   \
	do                                                       \
	{                                                        \
		if (WandererRotator::WR_DEBUG_ENABLED)               \
		{                                                    \
			WandererRotator::WRLogDebug(fmt, ##__VA_ARGS__); \
		}                                                    \
	} while (0)

#define WR_INFO(fmt, ...)                                   \
	do                                                      \
	{                                                       \
		if (WandererRotator::WR_INFO_ENABLED)               \
		{                                                   \
			WandererRotator::WRLogInfo(fmt, ##__VA_ARGS__); \
		}                                                   \
	} while (0)

#define WR_ERROR(fmt, ...)                                   \
	do                                                       \
	{                                                        \
		if (WandererRotator::WR_ERROR_ENABLED)               \
		{                                                    \
			WandererRotator::WRLogError(fmt, ##__VA_ARGS__); \
		}                                                    \
	} while (0)

	/* Logging functions - called by macros */
	void WRLogDebug(const char *fmt, ...);
	void WRLogInfo(const char *fmt, ...);
	void WRLogError(const char *fmt, ...);
	const char *WRGetTimestamp();

} /* namespace WandererRotator */

#endif /* WANDERER_ROTATOR_LOGGING_H */
