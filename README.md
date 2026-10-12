# WandererRotator SDK

A C/C++ SDK for controlling Wanderer Rotator devices via serial communication. Provides an easy-to-use API for device discovery, movement control, and status queries.

## Features

- **Device Discovery**: Automatic scanning and enumeration of connected Wanderer Rotator devices
- **Movement Control**: Relative and absolute rotation commands with async feedback
- **Async Movement Detection**: Background threads for non-blocking movement monitoring

## Requirements

- GCC/Clang with C++17 support
- CMake 3.22 or higher
- libudev library

### Install Dependencies (Ubuntu/Debian)

```bash
sudo apt-get install build-essential cmake libudev-dev
```

## Building from Source

### Clone and Build

```bash
cd WandererRotatorSDK
mkdir build && cd build
cmake ..
make
```

## Usage

### Basic Example

```c
#include "WandererRotatorSDK.h"
#include <stdio.h>
#include <unistd.h>

int main(void) {
    // Scan for devices - ids must have room for WR_MAX_NUM entries
    int ids[WR_MAX_NUM];
    int count = 0;
    if (WRRotatorScan(&count, ids) != WR_SUCCESS || count == 0) {
        printf("No devices found\n");
        return 1;
    }

    // Open first device
    int id = ids[0];
    if (WRRotatorOpen(id) != WR_SUCCESS) {
        printf("Failed to open device\n");
        return 1;
    }

    // Get device status
    WR_ROTATOR_STATUS status;
    WRRotatorGetStatus(id, &status);
    printf("Position: %.2f degrees\n", status.position);

    // Move rotator 45 degrees and wait for the move to finish
    if (WRRotatorMove(id, 45.0f) == WR_SUCCESS) {
        do {
            usleep(500000);
            WRRotatorGetStatus(id, &status);
        } while (status.moving);
        printf("Now at %.2f degrees\n", status.position);
    }

    // Close device
    WRRotatorClose(id);

    return 0;
}
```

### Run the Tests

```bash
cd build && ctest --output-on-failure
```

`test_fake_rotator` exercises the SDK against a simulated rotator on a
pseudo-terminal - no hardware needed.

Two programs test against a real device and are not run by `ctest`:
`test_hardware` runs automated checks (moves, overshoot, stop, calls during a
move, close/reopen mid-move, a 450 degree move) and returns the rotator to its
starting position - **it rotates the device by up to 450 degrees, so make sure
it can turn freely**. `test_wanderer_rotator` is an interactive test.

```bash
./test_hardware      # asks for confirmation; -y to skip
```

### Compile with SDK

```bash
gcc -o my_app my_app.c -lWandererRotatorSDK
```

## API Reference

Every function returns a `WR_ERROR_TYPE`: `WR_SUCCESS` (0) on success, otherwise
`WR_ERROR_INVALID_ID`, `WR_ERROR_INVALID_PARAMETER`, `WR_ERROR_INVALID_STATE`,
`WR_ERROR_COMMUNICATION` or `WR_ERROR_NULL_POINTER`.

### Device Management

#### `WRRotatorScan(int *number, int *ids)`
Scan for Wanderer Rotator devices (CH340 USB-serial adapters answering the rotator
handshake). Devices already known to the SDK keep their id across rescans.

- `ids` - array that receives the device ids; must hold `WR_MAX_NUM` entries
- `number` - receives the number of ids written

Registered devices that have been unplugged and are not open are dropped from
the list (their id becomes invalid). Devices are registered under their
`/dev/serial/by-path/...` link where there is one, so `WRRotatorOpen` reconnects
after the rotator is unplugged and plugged back into the same socket, even if it
comes back as a different `/dev/ttyUSBn`.

#### `WRRotatorGetPort(int id, char *port)`
Get a port name for a scanned device that can be saved and passed to
`WRRotatorScanPort` later. `port` must hold `WR_PORT_LEN` bytes. This is the
device's `/dev/serial/by-path/...` link, which stays the same across reconnects
and reboots as long as the rotator is plugged into the same USB socket (or its
device node if there is no such link). The CH340 has no USB serial number, so
`/dev/serial/by-id/...` is the same for every CH340 device and cannot be used.

#### `WRRotatorScanPort(const char *port, int *id)`
Probe only the given port instead of every CH340 port, so devices of other
programs are not touched. `id` receives the device id. A port that already
belongs to a registered device returns that device's id without probing it again.
The device keeps the port name it was found on, so with a by-path link
`WRRotatorOpen` reconnects after a replug the same way as for `WRRotatorScan`.
If nothing on the port is a CH340 or no rotator answers there, it returns
`WR_ERROR_COMMUNICATION`; fall back to `WRRotatorScan`:

```c
int id, count, ids[WR_MAX_NUM];
if (WRRotatorScanPort(savedPort, &id) != WR_SUCCESS) {
    if (WRRotatorScan(&count, ids) != WR_SUCCESS || count == 0)
        return -1;
    id = ids[0];
}
WRRotatorGetPort(id, savedPort);   /* remember for next time */
```

#### `WRRotatorOpen(int id)` / `WRRotatorClose(int id)`
Open or close the serial connection to a scanned device. Closing does not stop a
move that is in progress. Opening fails quickly with an error logged if the user
lacks permission for the serial port (on most distributions: not in the `dialout`
group).

### Moves in Progress

Moves run in the background. While one is in progress, `WRRotatorMove`,
`WRRotatorMoveTo`, `WRRotatorFindHome`, `WRRotatorSyncPosition` and
`WRRotatorSetConfig` (reverse direction or backlash) wait for it to finish
before running. `WRRotatorStopMove`, `WRRotatorClose` and the `Get` functions
never wait. Calls that need an open device return `WR_ERROR_INVALID_STATE`
when it is not open. If the USB connection of an open device is lost, calls
return `WR_ERROR_COMMUNICATION` until `WRRotatorOpen` reconnects it.

Numbers are exchanged with the device in a locale-independent way, so the SDK
works in applications that use a decimal comma (e.g. German locales).

Scanning probes every CH340 serial port. Ports that another program holds
exclusively (TIOCEXCL or an advisory lock) are skipped once they stay busy;
ports that turn out not to be a rotator get their previous settings back.

While a device is open, the SDK holds an advisory lock (`flock`) on
`/tmp/ch34x-<port>.lock` (e.g. `/tmp/ch34x-ttyUSB0.lock`). Scans that find a
port busy check this marker: if it is held the port is in use and is skipped at
once, otherwise it is assumed to be briefly probed by another scan and waited
for (up to 6 s). Other SDKs for CH34x-based devices can follow the same
convention so that scans of all of them skip ports in use.

### Configuration

#### `WRRotatorGetConfig(int id, WR_ROTATOR_CONFIG *config)`
#### `WRRotatorSetConfig(int id, const WR_ROTATOR_CONFIG *config)`
`WRRotatorGetConfig` sets `config->mask` to 0. `config->mask` selects which fields `WRRotatorSetConfig` applies
(`MASK_ROTATOR_REVERSE_DIRECTION`, `MASK_ROTATOR_BACKLASH`, `MASK_ROTATOR_OVERSHOOT`,
`MASK_ROTATOR_OVERSHOOT_ANGLE`, `MASK_ROTATOR_OVERSHOOT_DIRECTION`, or
`MASK_ROTATOR_ALL`). All selected fields are validated before anything is sent to
the device.

- `reverseDirection` - reverse the motor direction (stored on the device)
- `backlash` - backlash in degrees (stored on the device)
- `overshoot`, `overshootAngle`, `overshootDirection` - software backlash
  compensation: moves in the selected direction (`overshootDirection` 0 -
  positive/counterclockwise, non-zero - negative/clockwise) go `overshootAngle`
  past the target and then return to it

### Status and Information

#### `WRRotatorGetStatus(int id, WR_ROTATOR_STATUS *status)`
Current `position` in degrees, `moving` flag, `stepsPerRevolution` and `stepSize`.
Moves run in the background; poll `moving` to wait for completion.

#### `WRRotatorGetVersion(int id, WR_VERSION *version)`
Firmware version and model name.

#### `WRGetSDKVersion(char *version)`
SDK version string; `version` must hold `WR_VERSION_LEN` bytes.

### Motion Control

#### `WRRotatorMove(int id, float angle)`
Rotate by `angle` degrees relative to the current position (-3600 to 3600;
positive is counterclockwise). Returns as soon as the move has started. Moves
larger than one device command can carry (about 417 degrees) are sent as
consecutive segments.

#### `WRRotatorMoveTo(int id, float angle)`
Rotate to the absolute position `angle` (0 to <360) along the shortest path.

#### `WRRotatorFindHome(int id)`
Move to position 0.

#### `WRRotatorSyncPosition(int id, float angle)`
Define the current position as `angle`. Only 0 is currently supported.

#### `WRRotatorStopMove(int id)`
Stop the current move, including any pending overshoot return.

## License

MIT License - See [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please ensure:
- Code follows existing style and conventions
- All changes are tested
- License header is included in new files
- Commit messages are clear and descriptive

## Support

For issues, questions, or suggestions, please open an issue on the project repository.

---

**Version:** 1.6.0
