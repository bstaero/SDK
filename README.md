# BST SDK

Official SDK for the Black Swift Technologies SwiftCore flight management system.

**Version:** 3.23.0
**Website:** https://bst.aero
**License:** GNU General Public License v2

## Overview

The BST SDK enables real-time communication with SwiftCore autopilots, custom payload development, ground station integration, and hardware-in-the-loop simulation.

- **C/C++ SDK** - Primary implementation for embedded systems, real-time applications, payload development, and ground stations
- **Python SDK** - Wrapper for log parsing, data analysis, and web-based applications

## Documentation

Full documentation: [GitLab Wiki](https://gitlab.com/bstaero/sdk/-/wikis/home)

### C/C++ SDK (Primary)

| Guide | Description |
|-------|-------------|
| [Installation](https://gitlab.com/bstaero/sdk/-/wikis/Cpp-Installation) | Build setup and dependencies |
| [Quick Start](https://gitlab.com/bstaero/sdk/-/wikis/Cpp-Quick-Start) | First application walkthrough |
| [API Overview](https://gitlab.com/bstaero/sdk/-/wikis/Cpp-API-Overview) | Classes, headers, and patterns |

### Python SDK (Data Processing)

| Guide | Description |
|-------|-------------|
| [Installation](https://gitlab.com/bstaero/sdk/-/wikis/Python-Installation) | pip install and prerequisites |
| [Quick Start](https://gitlab.com/bstaero/sdk/-/wikis/Python-Quick-Start) | Log parsing basics |
| [API Overview](https://gitlab.com/bstaero/sdk/-/wikis/Python-API-Overview) | Module structure and usage |

### Core References

- [BST Protocol](https://gitlab.com/bstaero/sdk/-/wikis/BST-Protocol) - Packet structure, addressing, checksums
- [Data Structures](https://gitlab.com/bstaero/sdk/-/wikis/Data-Structures-Reference) - Complete struct documentation (State, GPS, IMU, IAS, TAS, etc.)
- [Packet Types](https://gitlab.com/bstaero/sdk/-/wikis/Packet-Types) - All packet types and enumerations
- [Communication Interfaces](https://gitlab.com/bstaero/sdk/-/wikis/Communication-Interfaces) - Serial, socket, CAN, file

### Example Applications

- [Examples Overview](https://gitlab.com/bstaero/sdk/-/wikis/Examples-Overview) - Architecture and file structure
- [CAN Communication](https://gitlab.com/bstaero/sdk/-/wikis/Example-CAN-Test) - CAN bus testing

---

## C/C++ SDK

### Directory Structure

```
sdk/
├── include/
│   ├── bst_protocol/       # Protocol headers
│   │   ├── bst_protocol.h  # Main handler
│   │   ├── bst_packet.h    # Packet class
│   │   ├── bst_module.h    # Module base
│   │   └── messages/       # Data structures
│   ├── bst_core/           # Utilities
│   └── bst_can/            # CAN bridge
├── src/                    # Implementation
└── examples/
    ├── common/             # Shared utility code and Makefile
    ├── template_bst/       # Copy-ready BST protocol starting point
    ├── template_can/       # Copy-ready CAN protocol starting point
    └── <example>/          # Individual example applications
```

### Quick Start

```cpp
#include "bst_protocol.h"
#include "bst_module_basic.h"
#include "bst_socket.h"

void receive(uint8_t type, void* data, uint16_t size, const void* param) {
    if (type == STATE_STATE) {
        State_t* state = (State_t*)data;
        printf("IAS: %.1f m/s\n", state->ias);
        printf("TAS: %.1f m/s\n", state->tas);
        printf("Alt: %.1f m\n", state->altitude);
    }
}

int main() {
    BSTProtocol* protocol = new BSTProtocol();
    BSTSocket* socket = new BSTSocket();
    socket->initialize("localhost", "55555", "udp");
    protocol->setInterface(socket);

    BSTModuleBasic basic;
    basic.registerReceive(receive);
    protocol->registerModule(&basic);

    socket->open();
    while (true) {
        protocol->update();
        usleep(1000);
    }
}
```

### Building Examples

```bash
cd examples/can_test
make
./test -i localhost -p 55555
```

---

## Python SDK

### Installation

**Prerequisites:**
```bash
# Ubuntu/Debian
sudo apt-get install swig python3-dev

# macOS
brew install swig
```

**Install:**
```bash
pip install BSTPythonSDK
```

**From source:**
```bash
git clone https://gitlab.com/bstaero/sdk.git
cd sdk
pip install -e .
```

### Quick Start

```python
from bst_python_sdk.logparse import Parser

# Parse a flight log
parser = Parser()
data = parser.parse_log("flight.bin")

# Access data by aircraft and packet type
for aircraft, packets in data.items():
    print(f"Aircraft: {aircraft}")

    # GPS data
    if 'SENSORS_GPS' in packets:
        for gps in packets['SENSORS_GPS']:
            print(f"  Position: {gps.latitude:.6f}, {gps.longitude:.6f}")

    # State data (includes IAS, TAS)
    if 'STATE_STATE' in packets:
        for state in packets['STATE_STATE']:
            print(f"  IAS: {state.ias:.1f} m/s")
            print(f"  TAS: {state.tas:.1f} m/s")
            print(f"  Altitude: {state.altitude:.1f} m")
```

### Convert to NetCDF

```python
from bst_python_sdk.log_to_nc import convert_to_nc

output_files = convert_to_nc("flight.bin", out_dir="./output")
```

---

## Example Applications

Located in `examples/`:

### Creating a New Example

Copy a template directory and start building:

```bash
# BST protocol example (serial/socket communication)
cp -r examples/template_bst/ examples/my_example/
cd examples/my_example/ && make

# CAN protocol example (simulated CAN bus)
cp -r examples/template_can/ examples/my_can_example/
cd examples/my_can_example/ && make
```

### Shared Code

All examples share common utility code from `examples/common/`:
- `example_common.cpp/.h` - Time management, terminal I/O, CLI argument parsing, endianness detection
- `Makefile.common` - Shared build rules with `EXAMPLE_TYPE` support (`bst`, `can`, `raw`)

Each example Makefile is just a few lines:
```makefile
EXAMPLE_TYPE = bst
include ../common/Makefile.common
```

### Available Examples

| Example | Type | Purpose |
|---------|------|---------|
| `template_bst` | BST | Copy-ready BST protocol starting point |
| `template_can` | CAN | Copy-ready CAN protocol starting point |
| `payload` | BST | Generic payload integration |
| `smm` | BST | SMM serial sensor |
| `gazebo` | BST | Gazebo SITL simulation (multirotor) |
| `emass` | BST | EMASS ECS-DoT payload node interface |
| `can_test` | CAN | CAN bus testing and visualization |
| `mhp` | Raw | Multi-hole probe meteorological |

### Gazebo Simulation Workflow

The `gazebo` example provides a complete software-in-the-loop (SITL) simulation environment for multirotor testing. It supports both Gazebo Classic (9-11) and Gazebo Jetty/Harmonic.

**Setup:**
```bash
cd examples/gazebo
make              # Build the GCS test binary
cd ../emass
make              # Build the EMASS payload binary (optional)
```

**Running a simulation:**

1. **Start the autopilot** (terminal 1):
   ```bash
   cd examples/gazebo
   ./run_autopilot
   ```

2. **Start Gazebo** (terminal 2):
   ```bash
   cd examples/gazebo
   ./run_gazebo              # Default: multirotor
   ./run_gazebo <world_name> # Other world (e.g., complex_multirotor)
   ```

3. **Connect the GCS test binary** (terminal 3) - used for flight control:
   ```bash
   cd examples/gazebo
   ./test -i localhost -p 55555
   ```
   Use the test binary to ready the vehicle for flight, take off, and fly.

4. **Connect the EMASS payload** (terminal 4, optional) - for external actuator control:
   ```bash
   cd examples/emass
   ./emass -i localhost -p 55551
   ```
   Once the vehicle is airborne and in FLYING mode, the payload node can take over actuator control.

5. **Landing** - use the test binary (terminal 3) to land and complete the flight.

**Ports:**
- `55554` - Internal AP comms (gcsDaemon)
- `55555` - GCS client connection (test binary)
- `55551` - Payload serial interface (EMASS / external controller)

### Example File Structure

All C++ examples follow this pattern:

| File | Purpose |
|------|---------|
| `main.cpp` | Entry point, CLI parsing, comms setup |
| `main.h` | Configuration, includes `example_common.h` |
| `test.cpp` | User interaction, telemetry display, file output |
| `test.h` | Test data structures |
| `test_handler.cpp` | **Primary customization point**: incoming packet handlers |
| `Makefile` | Sets `EXAMPLE_TYPE` and includes `Makefile.common` |

---

## Supported Vehicles

- Fixed-wing aircraft
- Multirotor (quadcopter, hexacopter, etc.)
- VTOL (tilt-rotor, quad-plane)
- Tail-sitter

## Key Data Fields

Common fields in `State_t`:

| Field | Unit | Description |
|-------|------|-------------|
| `ias` | m/s | Indicated airspeed |
| `tas` | m/s | True airspeed |
| `altitude` | m | Barometric altitude |
| `q[4]` | - | Attitude quaternion |

See [Data Structures Reference](https://gitlab.com/bstaero/sdk/-/wikis/Data-Structures-Reference) for complete documentation.

## Protocol Versions

Supports protocol versions 3.11.0 through 3.23.0. Version is automatically detected from log files.

## Dependencies

**C++:** C++11 compiler, libbst_lib
**Python:** numpy, scipy, h5netcdf, lxml, swig

## Support

- [GitLab Issues](https://gitlab.com/bstaero/sdk/-/issues)
- [Documentation Wiki](https://gitlab.com/bstaero/sdk/-/wikis/home)
