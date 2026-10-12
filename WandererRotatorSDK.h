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

#ifndef WANDERER_ROTATOR_SDK_H
#define WANDERER_ROTATOR_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#if defined(WANDERER_ROTATOR_SDK_BUILD)
#define WRAPI __declspec(dllexport)
#else
#define WRAPI __declspec(dllimport)
#endif
#elif defined(__GNUC__)
#define WRAPI __attribute__((visibility("default")))
#else
#define WRAPI
#endif

#define WR_MAX_NUM          32      /* Maximum rotator numbers supported by this SDK */
#define WR_VERSION_LEN      32      /* Buffer length for version strings */
#define WR_PORT_LEN         256     /* Buffer length for port names */

typedef enum _WR_ERROR_TYPE {
    WR_SUCCESS = 0,                     /* Success */
    WR_ERROR_INVALID_ID,                /* Device ID is invalid */
    WR_ERROR_INVALID_PARAMETER,         /* One or more parameters are invalid */
    WR_ERROR_INVALID_STATE,             /* Device is not in correct state for specific API call */
    WR_ERROR_COMMUNICATION,             /* Data communication error such as device has been removed from USB port */
    WR_ERROR_NULL_POINTER,              /* Caller passes null-pointer parameter which is not expected */
} WR_ERROR_TYPE;

/*
 * Used by WRxxxSetConfig() to indicate which field wants to be set
 */
#define MASK_ROTATOR_REVERSE_DIRECTION          0x01
#define MASK_ROTATOR_BACKLASH                   0x02
#define MASK_ROTATOR_OVERSHOOT                  0x04
#define MASK_ROTATOR_OVERSHOOT_ANGLE            0x08
#define MASK_ROTATOR_OVERSHOOT_DIRECTION        0x10
#define MASK_ROTATOR_ALL                        0x1F

typedef struct _WR_VERSION
{
    unsigned int firmware;              /* Rotator firmware version */
    char model[8];                      /* Model type (e.g., "Lite", "Mini") */
} WR_VERSION;

typedef struct _WR_ROTATOR_CONFIG
{
    unsigned int mask;          /* Used by WRRotatorSetConfig() to indicate which field wants to be set */
    int reverseDirection;       /* 0 - Not reverse motor moving direction, others - Reverse motor moving direction */
    float backlash;             /* Backlash in degrees */
    int overshoot;              /* Backlash overshoot: 0 - disabled, others - enabled */
    float overshootAngle;       /* Backlash overshoot angle in degrees(move past target, then return) */
    int overshootDirection;     /* Backlash overshoot direction: 0 - normal, others - reverse */
} WR_ROTATOR_CONFIG;

typedef struct _WR_ROTATOR_STATUS {
    float position;                     /* Current motor position in degrees */
    int moving;                         /* 0 - motor is not moving, others - Motor is moving */
    int stepsPerRevolution;             /* Steps per full revolution (hardware dependent) */
    float stepSize;                     /* Step size in degrees per step */
} WR_ROTATOR_STATUS;

/* Device scanning and management */
/* ids must have room for WR_MAX_NUM entries; *number receives the count written */
WRAPI WR_ERROR_TYPE WRRotatorScan(int *number, int *ids);
/* Probes only the given port (as returned by WRRotatorGetPort, or a device
 * node such as /dev/ttyUSB0) instead of all CH340 ports, so no other device
 * is touched. *id receives the device's id; a device already registered on
 * that port keeps its id and is not probed again. Returns
 * WR_ERROR_COMMUNICATION if no rotator answers there - fall back to
 * WRRotatorScan then. */
WRAPI WR_ERROR_TYPE WRRotatorScanPort(const char *port, int *id);
/* port must have room for WR_PORT_LEN bytes. Receives the device's
 * /dev/serial/by-path/... link, which stays the same across reconnects and
 * reboots as long as the rotator is plugged into the same USB socket, or its
 * device node if there is no such link. The CH340 has no USB serial number,
 * so /dev/serial/by-id/... cannot tell two such devices apart. */
WRAPI WR_ERROR_TYPE WRRotatorGetPort(int id, char *port);
WRAPI WR_ERROR_TYPE WRRotatorOpen(int id);
WRAPI WR_ERROR_TYPE WRRotatorClose(int id);

/* Configuration
 * WRRotatorGetConfig sets config->mask to 0; set the bits of the fields to
 * change before passing the struct to WRRotatorSetConfig. */
WRAPI WR_ERROR_TYPE WRRotatorGetConfig(int id, WR_ROTATOR_CONFIG *config);
WRAPI WR_ERROR_TYPE WRRotatorSetConfig(int id, const WR_ROTATOR_CONFIG *config);

/* Status and information */
WRAPI WR_ERROR_TYPE WRRotatorGetStatus(int id, WR_ROTATOR_STATUS *status);
WRAPI WR_ERROR_TYPE WRRotatorGetVersion(int id, WR_VERSION *version);

/* Motion control
 * Move, MoveTo, FindHome, SyncPosition and SetConfig (reverse direction or
 * backlash) wait for a move already in progress on the device to finish.
 * StopMove, Close and the Get* functions never wait. Calls on a device that
 * is not open return WR_ERROR_INVALID_STATE; calls on an open device whose
 * USB connection was lost return WR_ERROR_COMMUNICATION until it is reopened
 * (WRRotatorOpen reconnects). */
WRAPI WR_ERROR_TYPE WRRotatorFindHome(int id);
WRAPI WR_ERROR_TYPE WRRotatorSyncPosition(int id, float angle);
WRAPI WR_ERROR_TYPE WRRotatorMove(int id, float angle);
WRAPI WR_ERROR_TYPE WRRotatorMoveTo(int id, float angle);
WRAPI WR_ERROR_TYPE WRRotatorStopMove(int id);

/* Utility */
/* version must have room for WR_VERSION_LEN bytes */
WRAPI WR_ERROR_TYPE WRGetSDKVersion(char *version);

#ifdef __cplusplus
}
#endif

#endif    /* WANDERER_ROTATOR_SDK_H */
