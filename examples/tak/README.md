# TAK Example: Aircraft Positions on a TAK Server

This example connects a computer to a Black Swift Technologies (BST) ground station over a network socket. It reads each aircraft's position telemetry and sends it to a TAK server as Cursor-on-Target (CoT) messages. Each aircraft then shows up as a live track in ATAK, WinTAK, iTAK or WebTAK.

```
 Aircraft ──radio──► BST Ground Station ──TCP 55555/55556──► this program ──CoT (TCP/UDP)──► TAK Server ──► TAK clients
```

The program only listens. It never sends commands to the ground station or to the aircraft.

## Platform support

| Platform | Status |
|----------|--------|
| **Linux** | Recommended. Developed and tested on Ubuntu 24.04. |
| macOS | Might work. The SDK has some macOS handling, but this example is not tested there. |
| Windows | Not supported. The code uses POSIX sockets and terminal APIs - plus the os is a security hazard. |

## 1. Install build tools (Linux)

```bash
sudo apt update
sudo apt install build-essential git unzip
```

## 2. Get the SDK

Clone the BST SDK from GitLab. **You need the `develop` branch.** The `master` branch is older and does not include this example.

```bash
git clone -b develop https://gitlab.com/bstaero/sdk.git
cd sdk
git branch --show-current   # should print: develop
```

(Or download a zip from https://gitlab.com/bstaero/sdk: select the **develop** branch in the branch dropdown first, then **Code → Download source code → zip**, and unzip it.)

This example is included in the SDK at `examples/tak/`. Its Makefile uses the relative paths `../common` and `../..` to find the rest of the SDK, so build it from that location.

## 3. Build

```bash
cd examples/tak
make
```

The first build also compiles the SDK library (`lib/libbst_lib.a`). The output is an executable called `test`.

If you update the SDK later, rebuild the library as well, because `make` here only builds it when it is missing:

```bash
(cd ../../lib && make clean && make) && make clean && make
```

## 4. Run

You need:

- **Ground station address**: set its IP with `-i` and its port with `-p` (see below).
- **TAK server address**: set the TAK server's IP with `-T` and a **plain-text (non-TLS)** CoT input port with `-P`. The TAK Server default is `8087`.

```bash
./test -i <ground-station-ip> -p <ground-station-port> -T <tak-server-ip> -P 8087
```

### Connecting to the ground station

Each radio channel on the ground station has its own port:

| Radio channel | Port |
|---------------|------|
| Channel A | `55555` |
| Channel B | `55556` |

**Over wireless (most ground stations):** join the ground station's Wi-Fi network. The ground station is at `192.168.1.1`.

```bash
./test -i 192.168.1.1 -p 55555 -T <tak-server-ip> -P 8087   # Channel A
./test -i 192.168.1.1 -p 55556 -T <tak-server-ip> -P 8087   # Channel B
```

**Over Ethernet (ground stations with the MTS option):** connect to the ground station by Ethernet. It has a static IP of `10.10.16.119`, so give your computer's wired interface an address on the same subnet (for example `10.10.16.100`, netmask `255.255.255.0`).

```bash
./test -i 10.10.16.119 -p 55555 -T <tak-server-ip> -P 8087  # Channel A
./test -i 10.10.16.119 -p 55556 -T <tak-server-ip> -P 8087  # Channel B
```

To show aircraft on both channels, run two copies of the program: one on port `55555` and one on `55556`.

Once the program is running, it prints one line per aircraft each second:

```
TAK: connected to 10.0.0.20:8087 (tcp)
New aircraft: 0x41001234
0x41001234 S2-TEST          lla: +40.0149856 -105.2705456  1650.0 m | age  1.0 s | TAK up
```

Keys: `t` toggles the display, `p` shows help, `q` quits.

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `-i <ip>` | `localhost` | Ground station IP (`192.168.1.1` over Wi-Fi, `10.10.16.119` over Ethernet with MTS) |
| `-p <port>` | `55555` | Ground station port (`55555` Channel A, `55556` Channel B) |
| `-d <device>` / `-b <baud>` | | Use a serial port instead of a socket (for example a radio plugged in directly) |
| `-T <host>` | `localhost` | TAK server, or multicast address |
| `-P <port>` | `8087` | TAK port |
| `-U` | off | Send CoT over UDP instead of TCP |
| `-R <s>` | `1.0` | Minimum seconds between updates for each aircraft |
| `-S <s>` | `30` | CoT stale time: how long TAK keeps a track after its last update |

### Without a TAK server (ATAK/WinTAK on the same LAN)

TAK clients listen for situational-awareness multicast. You can send straight to them with no server:

```bash
./test -i 192.168.1.1 -p 55555 -U -T 239.2.3.1 -P 6969
```

## What gets sent

For each aircraft, the program sends one CoT event per update:

| CoT field | Source |
|-----------|--------|
| `uid` | `BST-<address>`, the aircraft's BST packet address (for example `BST-41001234`) |
| `callsign` | Aircraft name from `SYSTEM_INITIALIZE`, or the uid if no name has been received |
| `type` | `a-f-A-M-F-Q`: friendly, air, UAV |
| `point lat/lon` | `TELEMETRY_POSITION` latitude and longitude |
| `point hae` | `TELEMETRY_POSITION` altitude. **This is MSL, not height above ellipsoid**, so TAK may show the altitude off by the local geoid separation (tens of meters). |
| `track course/speed` | Computed from the north/east velocity |
| `remarks` | Height above ground (AGL), battery %, GPS satellite count |

Positions of `0,0` (no GPS fix yet) are not sent. Telemetry from the ground station itself is ignored.

## Troubleshooting

- **`waiting for aircraft telemetry ...` never goes away**: check the ground station IP and port, and make sure an aircraft is connected and sending telemetry.
- **`TAK: connect ... failed`**: the TAK server is not reachable, or that port is not a plain-TCP input. The program retries every 5 seconds.
- **TAK Server only has TLS (port 8089)**: this example does not do TLS. Either enable a plain-TCP input on the server (`<input _name="stdtcp" protocol="tcp" port="8087"/>` in `CoreConfig.xml`), or wrap the connection with `stunnel` using a client certificate from the TAK server.
- **The track appears and then fades out**: updates have stopped for longer than the stale time (`-S`).

## Files

| File | Purpose |
|------|---------|
| `main.cpp` | Parses command-line options and sets up the ground station and TAK connections |
| `test_handler.cpp` | Handles incoming BST packets and keeps a table of aircraft by source address |
| `test.cpp` | Main loop: rate-limits and sends CoT for each aircraft, and prints the display |
| `tak_client.cpp/.h` | Small CoT/XML builder and TCP/UDP sender, with no dependencies on the rest of the SDK |
