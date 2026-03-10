# SDK Examples

## Creating a New Example

Copy a template directory and start building:

```bash
# BST protocol (serial/socket communication with autopilot)
cp -r template_bst/ my_example/
cd my_example/ && make

# CAN protocol (simulated CAN bus communication)
cp -r template_can/ my_can_example/
cd my_can_example/ && make
```

See `template_bst/README.md` and `template_can/README.md` for customization details.

## Directory Structure

```
examples/
├── common/              # Shared utility code and build rules
│   ├── example_common.h # Shared declarations (time, terminal, CLI)
│   ├── example_common.cpp
│   └── Makefile.common  # Shared build rules
├── template_bst/        # Copy-ready BST protocol starting point
├── template_can/        # Copy-ready CAN protocol starting point
│
│   BST Protocol Examples:
├── payload/             # Generic payload integration
├── smm/                 # Soil moisture radiometer
├── gazebo/              # Gazebo SITL simulation (multirotor)
├── emass/               # EMASS ECS-DoT payload node interface
│
│   CAN Protocol Examples:
├── can_test/            # CAN bus testing and visualization
│
│   Raw Serial Examples:
└── mhp/                 # Multi-hole probe meteorological
```

## Example File Structure

All C++ examples follow the same pattern:

| File | Purpose |
|------|---------|
| `main.cpp` | Entry point, CLI parsing, comms setup |
| `main.h` | Configuration, includes `example_common.h` |
| `test.cpp` | User interaction, telemetry display, file output |
| `test.h` | Test data structures |
| `test_handler.cpp` | **Primary customization point**: incoming packet handlers |
| `Makefile` | Sets `EXAMPLE_TYPE` and includes `Makefile.common` |

## Makefile Configuration

Each example Makefile is just a few lines:

```makefile
EXAMPLE_TYPE = bst          # bst, can, or raw
EXTRA_OBJ = my_module.o     # Optional extra objects
EXTRA_FLAGS = -DBOARD_PSNS  # Optional extra compiler flags
VEHICLE_FLAG = VEHICLE_MULTIROTOR  # Optional (default: VEHICLE_FIXEDWING)

include ../common/Makefile.common
```

## Running

```bash
# Socket (default: localhost:55555)
./test

# Custom socket
./test -i 192.168.1.100 -p 55555

# Serial
./test -d /dev/ttyUSB0 -b 57600
```
