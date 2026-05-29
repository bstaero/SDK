# payload_monitor

A terminal **bus monitor** for the BST protocol — a UAVCAN-GUI-Tool-style view,
in the terminal. It connects to an autopilot's **payload interface** (or a SWIL
sim) and listens to every packet, showing:

- a live table of every packet type seen, with its receive **rate (Hz)**, total
  count, last payload size, and age, and
- a **detail pane** that decodes the most recent packet of the selected type
  (known telemetry structs) or shows a hex + ASCII dump of anything else.

It is read-only: it never transmits, it just watches the link.

This is the *diagnostic* companion to the [`payload`](../payload) example. The
`payload` example is the clean skeleton to copy when building your own payload
(streaming data in, sending commands/waypoints, requesting settings); this tool
keeps all of the monitoring/TUI code separate so that skeleton stays simple.

## Build

```bash
cd examples/payload_monitor
make
```

## Run

```bash
# payload serial interface (typical: autopilot payload UART)
./test -d /dev/ttyUSB1 -b 460800

# SWIL / socket
./test -i localhost -p 55551
```

Default with no arguments is `-i localhost -p 55555`.

> **Note — this is the payload interface, not the GCS link.** Point it at the
> autopilot's payload serial channel (BST_PROTOCOL), e.g. `/dev/ttyUSB1 @
> 460800`. Port `55555` is the GCS-facing socket; in SWIL the payload interface
> is `55551`.

## Keys

| Key            | Action                                              |
|----------------|-----------------------------------------------------|
| `j` / `k`, ↓/↑ | move the selection in the type table                |
| `g` / `G`      | jump to first / last type                           |
| `x`            | toggle decoded fields vs. raw hex dump              |
| `p` / space    | pause (freeze the display; counts keep accumulating)|
| `c`            | clear all statistics                                |
| `q` / Ctrl-C   | quit (Esc is reserved for arrow sequences)          |

## Decoders

Decoded field views are provided for `TELEMETRY_POSITION`, `TELEMETRY_ORIENTATION`,
`TELEMETRY_PRESSURE`, `TELEMETRY_SYSTEM`, and `TELEMETRY_CONTROL`. Every other
packet type is shown as a hex + ASCII dump (and `x` forces the hex view for the
decoded types too). Adding a decoder is just another `case` in `decodeDetail()`
in `monitor.cpp`.

Fields render as `---` when they carry the firmware's **no-data sentinel** — a
field set to the maximum value of its integer type means NaN (see
`getTelemetryValue()` in the firmware's `comm_handler_base.cpp`). This is why,
with no GPS fix, latitude/longitude show `---` rather than a bogus `+922.33 deg`.

## How it receives everything (addressing)

BST packets on the wire carry a destination address, and `BSTProtocol::update()`
drops any addressed packet whose destination doesn't match this node's
`system_initialize.serial_num`. A `serial_num` of `0` (`NO_ID`) matches *nothing*
— not even `ALL_NODES` broadcasts — so `main.cpp` sets our address to
`ALL_NODES`, which makes `Packet::isToID()` match every packet. That turns the
monitor into a promiscuous sniffer that sees all traffic regardless of who it is
addressed to.

## Files

| File              | Role                                                       |
|-------------------|------------------------------------------------------------|
| `main.cpp`        | CLI / transport setup, promiscuous-address setup, main loop|
| `test_handler.cpp`| receive hooks → forward every packet to `monitorPacket()`  |
| `test.cpp`        | per-loop tick → `monitorUpdate()`                          |
| `monitor.{h,cpp}` | all of the monitoring + TUI logic (self-contained)         |
