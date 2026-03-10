# parameter_load

Load and upload app parameters from XML to a BST autopilot via the GCS daemon socket.

## Usage

```bash
# Validate only (no connection required)
python parameter_load.py --validate-only example_app.xml

# Upload to autopilot (GCS daemon must be running)
python parameter_load.py example_app.xml

# Custom host/port
python parameter_load.py example_app.xml --host 192.168.1.1 --port 55555

# Skip read-back verification
python parameter_load.py example_app.xml --no-verify

# Use a specific comm protocol version (default: symlinked version)
python parameter_load.py --comm-version 3200 example_app.xml

# Verbose packet-level output
python parameter_load.py example_app.xml -v
```

## XML Format

Uses the same XML format as the SwiftTab tablet application. See `example_app.xml` for a reference.

Supported sections:
- `<Payloads>` - Payload channel configurations (type, signal, state, trigger distance, pulse, power timing)
- `<Actuators>` - Actuator/servo calibrations (function, min/center/max usec)
- `<PayloadSerial>` - Payload serial interface (baud rate, interface type)

## Upload Sequence

1. Parse and validate XML locally
2. Connect to GCS daemon
3. Request current parameters from autopilot
4. Upload in order: payloads -> actuators -> serial
5. Read back and verify parameters were accepted

## Validation

Checks performed beyond what the tablet does:
- Channel range and duplicate detection
- Enum string validation for all types
- Actuator calibration pulse width bounds (500-2500 usec)
- Min <= max ordering for actuator calibration
- Center within [min, max] range
- Negative value detection for timing parameters
- Channel name length (max 31 bytes UTF-8)
- Baud rate validation
- Camera-specific warnings (zero pulse, unknown signal type)
- Actuator overwrite protection (won't overwrite control surfaces)

## Comm Protocol Versions

Supports versions 3150+ (use `--comm-version`). Defaults to the version symlinked
in `bst_python_sdk/comm_packets/`. Versions 3110-3140 use a different PayloadParam
layout and are not compatible with the app XML format.
