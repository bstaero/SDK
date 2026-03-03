# CAN Protocol Example Template

Copy this entire directory to create a new CAN example:

```bash
cp -r template_can/ my_can_example/
cd my_can_example/
make
```

## File Overview

| File | Purpose |
|------|---------|
| `main.cpp` | Entry point: arg parsing, SimulatedCAN setup, main loop |
| `main.h` | Globals: comm_handler, comm_interface, writeFile/writeBytes |
| `test.cpp` | User interaction: key handlers, telemetry display |
| `test.h` | Test interface declarations |
| `test_handler.cpp` | **Primary customization point**: CAN callback stubs |
| `test_handler.h` | Handler declarations |
| `Makefile` | Build config (includes `../common/Makefile.common`) |

## Customization

1. Edit `test_handler.cpp` to fill in the CAN callbacks you need
2. Edit `test.cpp` to add custom key commands and display
3. Add extra source files or board flags in the Makefile:
   ```makefile
   EXTRA_OBJ = my_sensor.o
   EXTRA_FLAGS = -DBOARD_PSNS
   ```

## Connection

```bash
# Socket (default: localhost:55555)
./test

# Custom socket
./test -i 192.168.1.100 -p 55555

# Serial
./test -d /dev/ttyUSB0 -b 57600

# With output file
./test -o data.csv
```
