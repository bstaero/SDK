# BST Python SDK

This is the public SDK for the Black Swift Technologies SwiftCore flight management system. More information on the products this works with can be found on our website https://bst.aero

**Package:** `BSTPythonSDK`
**Version:** 3.23.0
**License:** GNU General Public License v2

## Documentation

Full API documentation is available on the [GitLab Wiki](https://gitlab.com/bstaero/sdk/-/wikis/home):

### Getting Started
- [Installation](https://gitlab.com/bstaero/sdk/-/wikis/Installation)
- [Quick Start](https://gitlab.com/bstaero/sdk/-/wikis/Quick-Start)

### Core API
- [API Overview](https://gitlab.com/bstaero/sdk/-/wikis/API-Overview)
- [Log Parsing](https://gitlab.com/bstaero/sdk/-/wikis/Log-Parsing)
- [Packet Types](https://gitlab.com/bstaero/sdk/-/wikis/Packet-Types)
- [Vehicle-Specific Packets](https://gitlab.com/bstaero/sdk/-/wikis/Vehicle-Specific-Packets)

### Advanced Topics
- [NetCDF Export](https://gitlab.com/bstaero/sdk/-/wikis/NetCDF-Export)
- [Custom Payloads](https://gitlab.com/bstaero/sdk/-/wikis/Custom-Payloads)
- [Protocol Versions](https://gitlab.com/bstaero/sdk/-/wikis/Protocol-Versions)
- [SWIG Parser](https://gitlab.com/bstaero/sdk/-/wikis/SWIG-Parser)

### Integration Guides
- [Gazebo Setup](https://gitlab.com/bstaero/sdk/-/wikis/Gazebo-Setup)
- [SwiftFlow Interface](https://gitlab.com/bstaero/sdk/-/wikis/SwiftFlow-Interface)
- [Payload Interface](https://gitlab.com/bstaero/sdk/-/wikis/Payload-Interface)

## Python SDK

### Prerequisites

Installing the Python SDK requires the following to be installed on your machine:

- swig
- python3-dev

**Ubuntu/Debian:**
```bash
sudo apt-get install swig python3-dev
```

**macOS (with Homebrew):**
```bash
brew install swig
```

### Install

```bash
pip install BSTPythonSDK
```

Or install from source:

```bash
git clone https://gitlab.com/bstaero/sdk.git
cd sdk
pip install -e .
```

### Import

```python
import bst_python_sdk
```

## Quick Start

### Parse Log

```python
from bst_python_sdk.logparse import Parser

parser = Parser()
parsed_log = parser.parse_log("path/to/log.bin")

# Access data by aircraft and packet type
for aircraft, packets in parsed_log.items():
    print(f"Aircraft: {aircraft}")

    if 'SENSORS_GPS' in packets:
        for gps in packets['SENSORS_GPS']:
            print(f"  GPS: {gps.latitude:.6f}, {gps.longitude:.6f}")
```

### Convert to NetCDF

```python
from bst_python_sdk.log_to_nc import convert_to_nc

# Convert and export to current directory
output_files = convert_to_nc("path/to/log.bin")
# Returns: ["log_010_FW0001.nc", "log_010_SwiftStation.nc"]
```

### Access State Data

```python
from bst_python_sdk.logparse import Parser

parser = Parser()
data = parser.parse_log("flight.bin")

for aircraft, packets in data.items():
    if 'STATE_STATE' in packets:
        for state in packets['STATE_STATE']:
            print(f"Time: {state.system_time:.2f}s")
            print(f"Altitude: {state.altitude:.1f}m")
            print(f"Airspeed: {state.ias:.1f}m/s")
```

### Parser Options

```python
parser = Parser(
    has_addr=True,      # Log uses packet addressing (default: True)
    quick_mode=False,   # Fast mode, essential packets only (default: False)
    verbose=False,      # Print debug info (default: False)
    xml_payload_path="" # Path to custom payload XML
)
```

## Supported Vehicles

- Fixed-wing aircraft
- Multirotor (quadcopter, hexacopter, etc.)
- VTOL (tilt-rotor, quad-plane)
- Tail-sitter

## Packet Categories

The SDK parses the following packet types:

| Category | Examples |
|----------|----------|
| Sensors | GPS, IMU, Pressure, Magnetometer |
| State | Attitude, Altitude, Airspeed |
| Control | Commands, PID gains, Flight parameters |
| Actuators | Servo PWM values, Calibration |
| Navigation | Waypoints, Flight plans |
| System | Health, Errors, Initialization |
| Telemetry | Position, Orientation, System status |
| Payload | Trigger events, Custom data channels |

See [Packet Types](https://gitlab.com/bstaero/sdk/-/wikis/Packet-Types) for complete reference.

## Protocol Versions

The SDK supports protocol versions 3.11.0 through 3.23.0. Version is automatically detected from log files.

See [Protocol Versions](https://gitlab.com/bstaero/sdk/-/wikis/Protocol-Versions) for details.

## Dependencies

- numpy
- scipy
- h5netcdf
- lxml

## Support

For issues and feature requests, please use the [GitLab issue tracker](https://gitlab.com/bstaero/sdk/-/issues).

## License

GNU General Public License v2
