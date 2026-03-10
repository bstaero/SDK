# BST Protocol Example Template

Copy this entire directory to create a new BST protocol example:

```bash
cp -r template_bst/ my_example/
cd my_example/
make
```

## File Overview

| File | Purpose |
|------|---------|
| `main.cpp` | Entry point: arg parsing, BSTProtocol setup, main loop |
| `main.h` | Globals: comm_handler, comm_interface |
| `test.cpp` | User interaction: key handlers, telemetry display |
| `test.h` | Test interface declarations |
| `test_handler.cpp` | **Primary customization point**: receive/command/reply handlers |
| `test_handler.h` | Handler function declarations |
| `Makefile` | Build config (includes `../common/Makefile.common`) |

## Customization

1. Edit `test_handler.cpp` to handle the message types you need
2. Edit `test.cpp` to add custom key commands and telemetry display
3. Add extra source files by setting `EXTRA_OBJ` in the Makefile:
   ```makefile
   EXTRA_OBJ = my_module.o
   ```

## Connection

```bash
# Socket (default: localhost:55555)
./test

# Custom socket
./test -i 192.168.1.100 -p 55555

# Serial
./test -d /dev/ttyUSB0 -b 57600
```
