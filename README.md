# BST SDK

Official SDK for the Black Swift Technologies SwiftCore flight management system.

**Version:** 3.23.0
**Website:** https://bst.aero
**License:** GNU General Public License v2

## Overview

The BST SDK provides both **Python** and **C/C++** interfaces for:
- Parsing binary telemetry logs
- Real-time communication with aircraft
- Custom payload integration
- Ground station development
- Hardware-in-the-loop simulation

## Documentation

Full documentation: [GitLab Wiki](https://gitlab.com/bstaero/sdk/-/wikis/home)

### Getting Started
| Python | C/C++ |
|--------|-------|
| [Installation](https://gitlab.com/bstaero/sdk/-/wikis/Python-Installation) | [Installation](https://gitlab.com/bstaero/sdk/-/wikis/Cpp-Installation) |
| [Quick Start](https://gitlab.com/bstaero/sdk/-/wikis/Python-Quick-Start) | [Quick Start](https://gitlab.com/bstaero/sdk/-/wikis/Cpp-Quick-Start) |
| [API Overview](https://gitlab.com/bstaero/sdk/-/wikis/Python-API-Overview) | [API Overview](https://gitlab.com/bstaero/sdk/-/wikis/Cpp-API-Overview) |

### Core References
- [BST Protocol](https://gitlab.com/bstaero/sdk/-/wikis/BST-Protocol) - Packet structure, addressing, checksums
- [Packet Types](https://gitlab.com/bstaero/sdk/-/wikis/Packet-Types) - All packet types and enumerations
- [Data Structures](https://gitlab.com/bstaero/sdk/-/wikis/Data-Structures-Reference) - Complete struct documentation (State, GPS, IMU, IAS, TAS, etc.)
- [Communication Interfaces](https://gitlab.com/bstaero/sdk/-/wikis/Communication-Interfaces) - Serial, socket, CAN, file

### Example Applications
- [Examples Overview](https://gitlab.com/bstaero/sdk/-/wikis/Examples-Overview) - File structure and architecture
- [CAN Test](https://gitlab.com/bstaero/sdk/-/wikis/Example-CAN-Test) - CAN bus communication
- [Gazebo Simulation](https://gitlab.com/bstaero/sdk/-/wikis/Example-Gazebo) - HITL simulation
- [Payload Integration](https://gitlab.com/bstaero/sdk/-/wikis/Example-Payload) - Custom sensor template

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
└── examples/               # Example applications
```

### Quick Start

```cpp
#include "bst_protocol.h"
#include "bst_module_basic.h"
#include "netuas_socket.h"

void receive(uint8_t type, void* data, uint16_t size, const void* param) {
    if (type == STATE_STATE) {
        State_t* state = (State_t*)data;
        printf("IAS: %.1f m/s\n", state->ias);
        printf("TAS: %.1f m/s\n", state->tas);
    }
}

int main() {
    BSTProtocol* protocol = new BSTProtocol();
    NetuasSocket* socket = new NetuasSocket();
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

## Example Applications

Located in `examples/`:

| Example | Purpose |
|---------|---------|
| `can_test` | CAN bus testing and visualization |
| `gazebo` | Gazebo HITL simulation |
| `payload` | Generic payload template |
| `mhp` | Multi-hole probe meteorological |
| `python_payload` | Python real-time visualization |
| `ch4` | Methane sensor integration |
| `s0` | Ground state testing |
| `psns_test` | Pressure sensor network |

### Example File Structure

All C++ examples follow this pattern:

| File | Purpose |
|------|---------|
| `main.cpp` | CLI parsing, interface setup, main loop |
| `main.h` | Configuration, timing functions |
| `test.cpp` | Display, file output, user interaction |
| `test.h` | Data structures |
| `test_handler.cpp` | Incoming packet handlers |

---

## Supported Vehicles

- Fixed-wing aircraft
- Multirotor (quadcopter, hexacopter, etc.)
- VTOL (tilt-rotor, quad-plane)
- Tail-sitter

## Key Data Fields

Common fields you'll access:

| Field | Location | Unit | Description |
|-------|----------|------|-------------|
| `ias` | State_t | m/s | Indicated airspeed |
| `tas` | State_t | m/s | True airspeed |
| `altitude` | State_t | m | Barometric altitude |
| `latitude` | GPS_t | deg | Latitude |
| `longitude` | GPS_t | deg | Longitude |
| `q[4]` | State_t | - | Attitude quaternion |
| `usec[16]` | Actuators_t | µs | Servo PWM values |

See [Data Structures Reference](https://gitlab.com/bstaero/sdk/-/wikis/Data-Structures-Reference) for complete documentation.

## Protocol Versions

The SDK supports protocol versions 3.11.0 through 3.23.0. Version is automatically detected from log files.

## Dependencies

**Python:** numpy, scipy, h5netcdf, lxml, swig
**C++:** C++11 compiler, libnetuas_lib

## Support

- [GitLab Issues](https://gitlab.com/bstaero/sdk/-/issues)
- [Documentation Wiki](https://gitlab.com/bstaero/sdk/-/wikis/home)
