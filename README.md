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

int main() {
    // Scan for devices
    WRRotatorInfo devices[10];
    int count = WRRotatorScan(devices, 10);
    
    if (count <= 0) {
        printf("No devices found\n");
        return 1;
    }
    
    // Open first device
    int device_id = WRRotatorOpen(devices[0].port);
    if (device_id < 0) {
        printf("Failed to open device\n");
        return 1;
    }
    
    // Get device status
    WRRotatorStatus status;
    WRRotatorGetStatus(device_id, &status);
    printf("Status: %s\n", status.status);
    printf("Mechanical Position: %.1f degrees\n", status.mechanical_angle);
    
    // Move rotator 45 degrees
    WRRotatorMove(device_id, 45.0);
    
    // Close device
    WRRotatorClose(device_id);
    
    return 0;
}
```

### Compile with SDK

```bash
gcc -o my_app my_app.c -lWandererRotatorSDK
```

## API Reference

### Device Management

#### `WRRotatorScan(devices, max_devices)`
Scan for available Wanderer Rotator devices on the system.

**Parameters:**
- `WRRotatorInfo* devices` - Array to store discovered devices
- `int max_devices` - Maximum number of devices to return

**Returns:** Number of devices found (>= 0)

#### `WRRotatorOpen(port)`
Open a connection to a Wanderer Rotator device.

**Parameters:**
- `const char* port` - Serial port path (e.g., "/dev/ttyUSB0")

**Returns:** Device ID (>= 0 on success, < 0 on error)

#### `WRRotatorClose(device_id)`
Close connection to a device and clean up resources.

**Parameters:**
- `int device_id` - Device ID from WRRotatorOpen

**Returns:** 0 on success, < 0 on error

### Movement Control

#### `WRRotatorMove(device_id, degrees)`
Rotate the device by the specified number of degrees (relative movement).

**Parameters:**
- `int device_id` - Device ID
- `double degrees` - Degrees to rotate (-360 to 360)

**Returns:** 0 on success, < 0 on error

#### `WRRotatorMoveTo(device_id, degrees)`
Rotate the device to an absolute position.

**Parameters:**
- `int device_id` - Device ID
- `double degrees` - Target angle in degrees (0-360)

**Returns:** 0 on success, < 0 on error

### Status and Monitoring

#### `WRRotatorGetStatus(device_id, status)`
Get current device status and position.

**Parameters:**
- `int device_id` - Device ID
- `WRRotatorStatus* status` - Pointer to status structure

**Returns:** 0 on success, < 0 on error

#### `WRRotatorIsMoving(device_id)`
Check if device is currently moving.

**Parameters:**
- `int device_id` - Device ID

**Returns:** 1 if moving, 0 if idle, < 0 on error

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

**Version:** 1.0.0  
**Last Updated:** December 2025