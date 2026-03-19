#!/usr/bin/env python3
#*=+--+=#=+--            SwiftPilot Autopilot Software            --+=#=+--+=#*#
#               Copyright (C) 2026 Black Swift Technologies LLC.               #
#                             All Rights Reserved.                             #
#                                                                              #
#    This program is free software: you can redistribute it and/or modify      #
#    it under the terms of the GNU General Public License version 2 as         #
#    published by the Free Software Foundation.                                #
#                                                                              #
#    This program is distributed in the hope that it will be useful,           #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of            #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
#    GNU General Public License for more details.                              #
#                                                                              #
#    You should have received a copy of the GNU General Public License         #
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.     #
#                                                                              #
#                                 Jack Elston                                  #
#                          elstonj@blackswifttech.com                          #
#                                                                              #
#*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*#

"""
parameter_load - Load and validate app parameters from XML, then upload to
a connected autopilot via the GCS daemon socket.

Usage:
    python parameter_load.py <app.xml> [--host HOST] [--port PORT] [--validate-only] [--verbose]

This replicates (and improves on) the parameter upload flow used by the
SwiftTab tablet application:
    1. Parse app XML
    2. Validate all parameter values locally
    3. Connect to GCS daemon
    4. Request current parameters from autopilot
    5. Compare and display changes
    6. Upload changed parameters (payloads -> actuators -> serial)
    7. Verify parameters were accepted by reading them back
"""

import argparse
import importlib
import os
import socket
import struct
import sys
import time
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from bst_python_sdk.bstpacket import BSTPacket
from bst_python_sdk.listener import parse_stream

# These globals are set by load_comm_version() to support multiple protocol versions.
PacketTypes = None
ActuatorFunction = None
ActuatorCalibration = None
PayloadParam = None
PayloadSerial = None
PayloadType = None
PayloadSignal = None
PayloadState = None
PayloadInterface = None
standard_handler = None


def get_available_versions():
    """Discover available comm protocol versions from the SDK."""
    versions_dir = os.path.join(os.path.dirname(__file__),
                                '..', '..', 'bst_python_sdk', 'comm_versions')
    versions = []
    if os.path.isdir(versions_dir):
        for entry in os.listdir(versions_dir):
            if entry.startswith('ver_') and os.path.isdir(os.path.join(versions_dir, entry)):
                versions.append(int(entry[4:]))
    versions.sort()
    return versions


def get_default_version():
    """Get the version that the comm_packets symlink points to."""
    link = os.path.join(os.path.dirname(__file__),
                        '..', '..', 'bst_python_sdk', 'comm_packets')
    try:
        target = os.readlink(link)
        # e.g. './comm_versions/ver_3210'
        basename = os.path.basename(target)
        if basename.startswith('ver_'):
            return int(basename[4:])
    except OSError:
        pass
    # Fallback: use highest available version
    versions = get_available_versions()
    return versions[-1] if versions else 3210


MIN_SUPPORTED_VERSION = 3150

def load_comm_version(version):
    """Dynamically import a specific comm protocol version.

    The app XML format (with channelName, payloadType, payloadSignal,
    payloadState, PayloadSerial, etc.) requires comm version >= 3150.
    Versions 3110-3140 have a completely different PayloadParam layout
    (17 bytes, no enums) and are not compatible with this tool.
    """
    global PacketTypes, ActuatorFunction, ActuatorCalibration
    global PayloadParam, PayloadSerial, PayloadType, PayloadSignal
    global PayloadState, PayloadInterface, standard_handler

    if version < MIN_SUPPORTED_VERSION:
        print(f"ERROR: Comm version {version} is not supported. "
              f"The app XML format requires version >= {MIN_SUPPORTED_VERSION}.",
              file=sys.stderr)
        print(f"  Versions 3110-3140 use a different PayloadParam structure "
              f"(17 bytes, no channel name or type enums).", file=sys.stderr)
        sys.exit(1)

    ver_str = f'ver_{version}'
    comm_mod = importlib.import_module(
        f'.comm_versions.{ver_str}.comm_packets', package='bst_python_sdk')
    payload_mod = importlib.import_module(
        f'.comm_versions.{ver_str}.payload', package='bst_python_sdk')
    handler_mod = importlib.import_module(
        f'.comm_versions.{ver_str}.handler', package='bst_python_sdk')

    PacketTypes = comm_mod.PacketTypes
    ActuatorFunction = comm_mod.ActuatorFunction
    ActuatorCalibration = comm_mod.ActuatorCalibration
    PayloadType = payload_mod.PayloadType
    PayloadSignal = payload_mod.PayloadSignal
    PayloadState = payload_mod.PayloadState
    PayloadInterface = payload_mod.PayloadInterface
    PayloadParam = payload_mod.PayloadParam
    standard_handler = handler_mod.standard_handler

    # PayloadSerial exists in all supported versions (>= 3150)
    PayloadSerial = payload_mod.PayloadSerial

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PKT_ACTION_STATUS  = 0
PKT_ACTION_REQUEST = 1
PKT_ACTION_COMMAND = 2
PKT_ACTION_ACK     = 3
PKT_ACTION_NACK    = 4

MAX_PAYLOAD_CHANNELS = 8
MAX_ACTUATOR_CHANNELS = 16

ACTUATOR_USEC_MIN = 500
ACTUATOR_USEC_MAX = 2500

VALID_BAUD_RATES = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200,
                    230400, 460800, 921600]

# SwiftTab XML uses some enum names that differ from the comm protocol.
# Map them to the canonical names used by the Python SDK enums.
ENUM_ALIASES = {
    'INTERFACE_BST_SERIAL': 'INTERFACE_BST_PROTOCOL',
}

# ---------------------------------------------------------------------------
# XML Parsing
# ---------------------------------------------------------------------------

class PayloadConfig:
    def __init__(self, channel, channel_name, delta_t, pulse, power_up,
                 power_down, payload_type, payload_signal, payload_state,
                 payload_interface):
        self.channel = channel
        self.channel_name = channel_name
        self.delta_t = delta_t
        self.pulse = pulse
        self.power_up = power_up
        self.power_down = power_down
        self.payload_type = payload_type
        self.payload_signal = payload_signal
        self.payload_state = payload_state
        self.payload_interface = payload_interface

class ActuatorConfig:
    def __init__(self, channel, function, cal_min, cal_center, cal_max):
        self.channel = channel
        self.function = function
        self.cal_min = cal_min
        self.cal_center = cal_center
        self.cal_max = cal_max

class SerialConfig:
    def __init__(self, baud_rate, serial_interface):
        self.baud_rate = baud_rate
        self.serial_interface = serial_interface

class AppConfig:
    def __init__(self):
        self.name = ""
        self.payloads = []
        self.actuators = []
        self.serial = None


def parse_app_xml(xml_path):
    """Parse an app XML file into an AppConfig object.

    Supports two XML formats:
      - SwiftTab <param> format: <param>/<vehicle>/...
      - Legacy <App> format: <App>/...
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()

    app = AppConfig()

    if root.tag == 'param':
        # SwiftTab format: data lives under <vehicle>
        vehicle = root.find('vehicle')
        if vehicle is None:
            raise ValueError("<param> XML missing <vehicle> element")

        airframe = vehicle.find('airframe')
        app.name = _text(airframe, 'name', 'Unnamed') if airframe is not None else 'Unnamed'

        payloads_elem = vehicle.find('payloads')
        actuators_elem = vehicle.find('actuators')
        serial_elem = vehicle.find('payload_serial')

    elif root.tag == 'App':
        app.name = _text(root, 'Name', 'Unnamed')

        payloads_elem = root.find('Payloads')
        actuators_elem = root.find('Actuators')
        serial_elem = root.find('PayloadSerial')

    else:
        raise ValueError(f"Expected root element <param> or <App>, got <{root.tag}>")

    # Parse Payloads
    if payloads_elem is not None:
        for p in payloads_elem.findall('payload'):
            app.payloads.append(PayloadConfig(
                channel=_int(p, 'channel', 0),
                channel_name=_text(p, 'channelName', 'Unnamed'),
                delta_t=_float(p, 'deltaT', 0.0),
                pulse=_float(p, 'pulse', 0.0),
                power_up=_float(p, 'powerUp', 0.0),
                power_down=_float(p, 'powerDown', 0.0),
                payload_type=_text(p, 'payloadType', 'PAYLOAD_TYPE_UNUSED'),
                payload_signal=_text(p, 'payloadSignal', 'UNKNOWN_TYPE'),
                payload_state=_text(p, 'payloadState', 'PAYLOAD_STATE_UNKNOWN'),
                payload_interface=_normalize_enum(_text(p, 'payloadInterface', 'INTERFACE_UNKNOWN')),
            ))

    # Parse Actuators
    if actuators_elem is not None:
        for a in actuators_elem.findall('actuator'):
            cal = a.find('calibration')
            cal_min = 1500
            cal_center = 1500
            cal_max = 1500
            if cal is not None:
                cal_min = _int(cal, 'min', 1500)
                cal_center = _int(cal, 'center', 1500)
                cal_max = _int(cal, 'max', 1500)
            # Auto-swap reversed min/max (reversed = reversed servo direction)
            if cal_min > cal_max:
                cal_min, cal_max = cal_max, cal_min

            app.actuators.append(ActuatorConfig(
                channel=_int(a, 'channel', 0),
                function=_normalize_enum(_text(a, 'function', 'ACT_UNUSED')),
                cal_min=cal_min,
                cal_center=cal_center,
                cal_max=cal_max,
            ))

    # Parse PayloadSerial
    if serial_elem is not None:
        app.serial = SerialConfig(
            baud_rate=_int(serial_elem, 'baudRate', 0),
            serial_interface=_normalize_enum(_text(serial_elem, 'serialInterface', 'INTERFACE_UNKNOWN')),
        )

    return app


def _normalize_enum(name):
    """Map SwiftTab enum aliases to canonical SDK enum names."""
    return ENUM_ALIASES.get(name, name)

def _text(elem, tag, default=""):
    child = elem.find(tag)
    if child is not None and child.text is not None:
        return child.text.strip()
    return default

def _int(elem, tag, default=0):
    try:
        return int(float(_text(elem, tag, str(default))))
    except (ValueError, TypeError):
        return default

def _float(elem, tag, default=0.0):
    try:
        return float(_text(elem, tag, str(default)))
    except (ValueError, TypeError):
        return default


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate_app(app):
    """Validate all parameters in an AppConfig. Returns (ok, errors, warnings)."""
    errors = []
    warnings = []

    # -- Payloads --
    seen_channels = set()
    for pl in app.payloads:
        prefix = f"Payload ch{pl.channel} ({pl.channel_name})"

        if pl.channel < 0 or pl.channel >= MAX_PAYLOAD_CHANNELS:
            errors.append(f"{prefix}: channel must be 0-{MAX_PAYLOAD_CHANNELS - 1}")

        if pl.channel in seen_channels:
            errors.append(f"{prefix}: duplicate channel {pl.channel}")
        seen_channels.add(pl.channel)

        # Validate enum strings
        if not _valid_enum(PayloadType, pl.payload_type):
            errors.append(f"{prefix}: invalid payloadType '{pl.payload_type}'")

        if not _valid_enum(PayloadSignal, pl.payload_signal):
            errors.append(f"{prefix}: invalid payloadSignal '{pl.payload_signal}'")

        if not _valid_enum(PayloadState, pl.payload_state):
            errors.append(f"{prefix}: invalid payloadState '{pl.payload_state}'")

        if not _valid_enum(PayloadInterface, pl.payload_interface):
            warnings.append(f"{prefix}: unrecognized payloadInterface '{pl.payload_interface}' (not sent to autopilot)")

        # Value bounds
        if pl.delta_t < 0:
            errors.append(f"{prefix}: deltaT ({pl.delta_t}) must be >= 0")

        if pl.pulse < 0:
            errors.append(f"{prefix}: pulse ({pl.pulse}) must be >= 0")

        if pl.power_up < 0:
            errors.append(f"{prefix}: powerUp ({pl.power_up}) must be >= 0")

        if pl.power_down < 0:
            errors.append(f"{prefix}: powerDown ({pl.power_down}) must be >= 0")

        if len(pl.channel_name.encode('utf-8')) > 31:
            errors.append(f"{prefix}: channelName too long (max 31 chars)")

        # Camera-specific checks
        if pl.payload_type == 'PAYLOAD_TYPE_CAMERA':
            if pl.pulse <= 0:
                warnings.append(f"{prefix}: camera type with pulse=0 won't trigger")
            if pl.payload_signal == 'UNKNOWN_TYPE':
                warnings.append(f"{prefix}: camera type with UNKNOWN_TYPE signal")

    # -- Actuators --
    seen_act_channels = set()
    for act in app.actuators:
        prefix = f"Actuator ch{act.channel}"

        if act.channel < 0 or act.channel >= MAX_ACTUATOR_CHANNELS:
            errors.append(f"{prefix}: channel must be 0-{MAX_ACTUATOR_CHANNELS - 1}")

        if act.channel in seen_act_channels:
            errors.append(f"{prefix}: duplicate channel")
        seen_act_channels.add(act.channel)

        if not _valid_enum(ActuatorFunction, act.function):
            errors.append(f"{prefix}: invalid function '{act.function}'")

        # Calibration bounds
        if not (ACTUATOR_USEC_MIN <= act.cal_min <= ACTUATOR_USEC_MAX):
            errors.append(f"{prefix}: min ({act.cal_min}) out of range [{ACTUATOR_USEC_MIN}-{ACTUATOR_USEC_MAX}]")

        if not (ACTUATOR_USEC_MIN <= act.cal_center <= ACTUATOR_USEC_MAX):
            errors.append(f"{prefix}: center ({act.cal_center}) out of range [{ACTUATOR_USEC_MIN}-{ACTUATOR_USEC_MAX}]")

        if not (ACTUATOR_USEC_MIN <= act.cal_max <= ACTUATOR_USEC_MAX):
            errors.append(f"{prefix}: max ({act.cal_max}) out of range [{ACTUATOR_USEC_MIN}-{ACTUATOR_USEC_MAX}]")

        if act.cal_min > act.cal_max:
            errors.append(f"{prefix}: min ({act.cal_min}) > max ({act.cal_max})")

        if act.cal_center < act.cal_min or act.cal_center > act.cal_max:
            warnings.append(f"{prefix}: center ({act.cal_center}) outside [min, max] range")

        # Only allow payload-type functions from XML (safety)
        fn = act.function
        if _valid_enum(ActuatorFunction, fn):
            fn_val = ActuatorFunction[fn].value
            is_payload_fn = (ActuatorFunction.ACT_PAYLOAD_1.value <= fn_val <= ActuatorFunction.ACT_PAYLOAD_16.value)
            is_safe = fn in ('ACT_UNUSED',) or is_payload_fn
            if not is_safe:
                warnings.append(f"{prefix}: function '{fn}' is a control surface - "
                                "will only overwrite if current function is unused/payload")

    # -- Serial --
    if app.serial is not None:
        prefix = "PayloadSerial"
        if app.serial.baud_rate not in VALID_BAUD_RATES and app.serial.baud_rate != 0:
            warnings.append(f"{prefix}: baud rate {app.serial.baud_rate} is non-standard")

        if app.serial.baud_rate <= 0:
            errors.append(f"{prefix}: baud rate must be > 0")

        if not _valid_enum(PayloadInterface, app.serial.serial_interface):
            errors.append(f"{prefix}: invalid serialInterface '{app.serial.serial_interface}'")

    ok = len(errors) == 0
    return ok, errors, warnings


def _valid_enum(enum_cls, name):
    """Check if a string is a valid member of an Enum class."""
    return name in enum_cls.__members__


# ---------------------------------------------------------------------------
# BST Packet Helpers
# ---------------------------------------------------------------------------

def make_packet(pkt_type, action, data=bytearray(), addressing=True):
    """Build a BSTPacket with proper checksum."""
    pkt = BSTPacket()
    pkt.set_addressing(addressing)
    pkt.TYPE = pkt_type
    pkt.ACTION = action
    pkt.SIZE = len(data)
    pkt.TO = 0xFFFFFFFF
    pkt.FROM = 0
    pkt.DATA = bytearray(data)
    pkt.set_fletcher_16()
    return pkt


def serialize_payload_param(pl):
    """Serialize a PayloadConfig into the binary format for PAYLOAD_PARAMS."""
    buf = bytearray()
    buf.extend(struct.pack('<B', pl.channel))

    # Channel name: 32 bytes, null-terminated
    name_bytes = pl.channel_name.encode('utf-8')[:31]
    name_bytes = name_bytes + b'\x00' * (32 - len(name_bytes))
    buf.extend(name_bytes)

    buf.extend(struct.pack('<f', pl.delta_t))
    buf.extend(struct.pack('<f', pl.pulse))
    buf.extend(struct.pack('<f', pl.power_up))
    buf.extend(struct.pack('<f', pl.power_down))

    buf.extend(struct.pack('<B', PayloadType[pl.payload_type].value))
    buf.extend(struct.pack('<B', PayloadSignal[pl.payload_signal].value))
    buf.extend(struct.pack('<B', PayloadState[pl.payload_state].value))

    return buf


def serialize_actuator_calibration(act):
    """Serialize an ActuatorConfig into the binary format for ACTUATORS_CALIBRATION."""
    buf = bytearray()
    buf.extend(struct.pack('<B', act.channel))
    buf.extend(struct.pack('<B', ActuatorFunction[act.function].value))
    buf.extend(struct.pack('<H', act.cal_max))
    buf.extend(struct.pack('<H', act.cal_center))
    buf.extend(struct.pack('<H', act.cal_min))
    return buf


def serialize_payload_serial(serial):
    """Serialize a SerialConfig into the binary format for PAYLOAD_SERIAL."""
    buf = bytearray()
    buf.extend(struct.pack('<I', serial.baud_rate))
    buf.extend(struct.pack('<B', PayloadInterface[serial.serial_interface].value))
    return buf


# ---------------------------------------------------------------------------
# Communication
# ---------------------------------------------------------------------------

class AutopilotConnection:
    """Manages socket connection to GCS daemon for parameter operations."""

    def __init__(self, host='127.0.0.1', port=55555, verbose=False):
        self.host = host
        self.port = port
        self.verbose = verbose
        self.sock = None
        self.received_params = {}
        self.firmware_comms_rev = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(2.0)
        print(f"Connected to GCS daemon at {self.host}:{self.port}")

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def send_packet(self, pkt):
        """Send a BSTPacket over the socket."""
        raw = pkt.serialize()
        if self.verbose:
            pkt_name = _pkt_type_name(pkt.TYPE)
            action_name = _action_name(pkt.ACTION)
            print(f"  TX: {pkt_name} ({action_name}) size={pkt.SIZE}")
        self.sock.send(raw)

    def recv_packets(self, timeout=2.0, expected_type=None):
        """Receive and parse packets until timeout or expected type received."""
        results = []
        end_time = time.time() + timeout
        while time.time() < end_time:
            remaining = end_time - time.time()
            if remaining <= 0:
                break
            self.sock.settimeout(max(remaining, 0.1))
            try:
                data = self.sock.recv(4096)
                if len(data) == 0:
                    break
                # Parse incoming data
                parsed = self._parse_incoming(data)
                results.extend(parsed)
                if expected_type is not None:
                    for pkt_type, pkt_data in parsed:
                        if pkt_type == expected_type:
                            return results
            except socket.timeout:
                break
        return results

    def _parse_incoming(self, data):
        """Parse raw bytes into list of (pkt_type, parsed_object) tuples."""
        results = []
        pkt = BSTPacket()
        pkt.set_addressing(True)

        i = 0
        while i < len(data):
            if pkt.parse(data[i:], has_address=True):
                pkt_type = pkt.TYPE
                action = pkt.ACTION
                if self.verbose:
                    pkt_name = _pkt_type_name(pkt_type)
                    action_name = _action_name(action)
                    print(f"  RX: {pkt_name} ({action_name}) size={pkt.SIZE}")

                # Try to parse the packet data
                try:
                    parsed, _ = standard_handler(pkt)
                    if parsed is not None:
                        results.append((pkt_type, parsed))
                        self.received_params[pkt_type] = parsed
                        # Capture firmware comms_rev from SystemInitialize
                        if (hasattr(parsed, 'comms_rev') and
                                self.firmware_comms_rev is None):
                            self.firmware_comms_rev = parsed.comms_rev
                except Exception as e:
                    if self.verbose:
                        print(f"  Parse error for pkt {pkt_type}: {e}")

                i = i + pkt.SIZE + pkt.OVERHEAD
                pkt = BSTPacket()
                pkt.set_addressing(True)
            else:
                i = i + 1

        return results

    def request_parameter(self, pkt_type, channel=None):
        """Request a parameter from the autopilot. If channel is set, include
        it as the data byte (for channel-indexed params like payload/actuator)."""
        data = bytearray()
        if channel is not None:
            data = struct.pack('<B', channel)
        pkt = make_packet(pkt_type, PKT_ACTION_REQUEST, data)
        self.send_packet(pkt)

    def send_parameter(self, pkt_type, data):
        """Send a parameter command to the autopilot."""
        pkt = make_packet(pkt_type, PKT_ACTION_COMMAND, data)
        self.send_packet(pkt)


def _pkt_type_name(pkt_type_val):
    """Get PacketTypes name from value."""
    try:
        return PacketTypes(pkt_type_val).name
    except ValueError:
        return f"UNKNOWN({pkt_type_val})"

def _action_name(action):
    names = {0: "STATUS", 1: "REQUEST", 2: "COMMAND", 3: "ACK", 4: "NACK", 5: "LOAD"}
    return names.get(action, f"UNKNOWN({action})")


# ---------------------------------------------------------------------------
# Parameter Upload Logic
# ---------------------------------------------------------------------------

def detect_firmware_version(conn, selected_version, available_versions):
    """Request SystemInitialize to detect the firmware's comm protocol version.
    Returns the firmware comms_rev if detected, or None."""
    print("\nDetecting firmware comm protocol version...")
    conn.request_parameter(PacketTypes.SYSTEM_INITIALIZE.value)
    time.sleep(0.2)
    conn.recv_packets(timeout=2.0,
                      expected_type=PacketTypes.SYSTEM_INITIALIZE.value)

    fw_rev = conn.firmware_comms_rev
    if fw_rev is not None:
        print(f"  Firmware comms_rev: {fw_rev}")
        if fw_rev != selected_version:
            print(f"\n  *** VERSION MISMATCH ***")
            print(f"  Script is using comm version {selected_version}, "
                  f"but firmware reports comms_rev {fw_rev}.")
            if fw_rev in available_versions:
                print(f"  Use --comm-version {fw_rev} to match the firmware.")
            else:
                print(f"  WARNING: Firmware version {fw_rev} is not available "
                      f"in the SDK.")
                closest = min(available_versions,
                              key=lambda v: abs(v - fw_rev))
                print(f"  Closest available version: {closest}")
            print(f"  Telemetry packets will fail to parse with the wrong "
                  f"version.")
            print(f"  Parameter upload/verify may still work if the "
                  f"parameter packet formats haven't changed.\n")
    else:
        print("  WARNING: Could not detect firmware comm version "
              "(no SystemInitialize response).")
    return fw_rev


def request_current_params(conn, app):
    """Request all relevant current parameters from the autopilot."""
    print("\nRequesting current parameters from autopilot...")

    # Request payload params for each channel in the XML
    for pl in app.payloads:
        conn.request_parameter(PacketTypes.PAYLOAD_PARAMS.value, pl.channel)
        time.sleep(0.1)

    # Request actuator calibration for each channel in the XML
    for act in app.actuators:
        conn.request_parameter(PacketTypes.ACTUATORS_CALIBRATION.value, act.channel)
        time.sleep(0.1)

    # Request serial params
    if app.serial is not None and PayloadSerial is not None:
        conn.request_parameter(PacketTypes.PAYLOAD_SERIAL.value)
        time.sleep(0.1)

    # Give time for responses
    conn.recv_packets(timeout=2.0)


def compare_and_display(conn, app):
    """Compare XML params with current autopilot params and display changes."""
    changes = []

    for pl in app.payloads:
        prefix = f"Payload ch{pl.channel} ({pl.channel_name})"
        # We can't do field-by-field comparison without parsing the received
        # packets by channel, but we can note what will be sent
        changes.append(f"  {prefix}: type={pl.payload_type} signal={pl.payload_signal} "
                       f"state={pl.payload_state}")
        changes.append(f"    deltaT={pl.delta_t} pulse={pl.pulse} "
                       f"powerUp={pl.power_up} powerDown={pl.power_down}")

    for act in app.actuators:
        fn_name = act.function
        changes.append(f"  Actuator ch{act.channel}: function={fn_name} "
                       f"min={act.cal_min} center={act.cal_center} max={act.cal_max}")

    if app.serial is not None:
        changes.append(f"  Serial: baud={app.serial.baud_rate} "
                       f"interface={app.serial.serial_interface}")

    if changes:
        print("\nParameters to upload:")
        for c in changes:
            print(c)
    else:
        print("\nNo parameters to upload.")

    return len(changes) > 0


def check_actuator_overwrite(conn, act):
    """Check if an actuator channel can be safely overwritten.
    Returns (can_overwrite, reason)."""
    # Look for a cached actuator calibration response for this channel
    cached = conn.received_params.get(PacketTypes.ACTUATORS_CALIBRATION.value)
    if cached is None:
        # No cached data - we didn't get a response, proceed with caution
        return True, "no current data received (proceeding)"

    if hasattr(cached, 'channel') and cached.channel == act.channel:
        current_type = cached.type
        if isinstance(current_type, ActuatorFunction):
            current_val = current_type.value
        else:
            current_val = int(current_type)

        # Can overwrite if current function is UNUSED, INVALID, or a PAYLOAD function
        if current_val == ActuatorFunction.ACT_UNUSED.value:
            return True, "currently UNUSED"
        elif current_val == ActuatorFunction.ACT_INVALID.value:
            return True, "currently INVALID"
        elif (ActuatorFunction.ACT_PAYLOAD_1.value <= current_val
              <= ActuatorFunction.ACT_PAYLOAD_16.value):
            return True, f"currently {ActuatorFunction(current_val).name}"
        else:
            return False, f"currently {ActuatorFunction(current_val).name} (control surface - cannot overwrite)"

    return True, "channel mismatch in cached data (proceeding)"


def upload_parameters(conn, app):
    """Upload all parameters to the autopilot in the correct sequence:
    payloads -> actuators -> serial."""
    errors = []
    total = len(app.payloads) + len(app.actuators) + (1 if app.serial else 0)
    current = 0

    # Stage 1: Payloads
    if app.payloads:
        print("\nUploading payload parameters...")
        for pl in app.payloads:
            current += 1
            prefix = f"[{current}/{total}] Payload ch{pl.channel} ({pl.channel_name})"

            data = serialize_payload_param(pl)
            conn.send_parameter(PacketTypes.PAYLOAD_PARAMS.value, data)
            print(f"  {prefix} - sent")

            # Brief pause between sends
            time.sleep(0.15)

    # Stage 2: Actuators
    if app.actuators:
        print("\nUploading actuator parameters...")
        for act in app.actuators:
            current += 1
            prefix = f"[{current}/{total}] Actuator ch{act.channel} ({act.function})"

            # Check overwrite protection
            can_overwrite, reason = check_actuator_overwrite(conn, act)
            if not can_overwrite:
                msg = f"{prefix} - SKIPPED: {reason}"
                errors.append(msg)
                print(f"  {msg}")
                continue

            data = serialize_actuator_calibration(act)
            conn.send_parameter(PacketTypes.ACTUATORS_CALIBRATION.value, data)
            print(f"  {prefix} - sent ({reason})")
            time.sleep(0.15)

    # Stage 3: Serial
    if app.serial is not None:
        if PayloadSerial is None:
            errors.append("Serial: skipped (not supported in this comm version)")
            print(f"\n  Serial: SKIPPED (comm version does not support PayloadSerial)")
        else:
            current += 1
            prefix = f"[{current}/{total}] Serial (baud={app.serial.baud_rate})"
            print(f"\nUploading serial parameters...")
            data = serialize_payload_serial(app.serial)
            conn.send_parameter(PacketTypes.PAYLOAD_SERIAL.value, data)
            print(f"  {prefix} - sent")
            time.sleep(0.15)

    return errors


def verify_parameters(conn, app):
    """Read back parameters from autopilot to verify they were accepted."""
    print("\nVerifying uploaded parameters...")
    verify_errors = []

    # Re-request payload params
    for pl in app.payloads:
        conn.request_parameter(PacketTypes.PAYLOAD_PARAMS.value, pl.channel)
        time.sleep(0.15)
        results = conn.recv_packets(timeout=1.0, expected_type=PacketTypes.PAYLOAD_PARAMS.value)

        verified = False
        for pkt_type, parsed in results:
            if pkt_type == PacketTypes.PAYLOAD_PARAMS.value:
                if hasattr(parsed, 'channel') and parsed.channel == pl.channel:
                    # Check key fields
                    mismatches = []
                    if parsed.payloadType.value != PayloadType[pl.payload_type].value:
                        mismatches.append(f"type: expected {pl.payload_type}, got {parsed.payloadType.name}")
                    if parsed.payloadSignal.value != PayloadSignal[pl.payload_signal].value:
                        mismatches.append(f"signal: expected {pl.payload_signal}, got {parsed.payloadSignal.name}")
                    if parsed.payloadState.value != PayloadState[pl.payload_state].value:
                        mismatches.append(f"state: expected {pl.payload_state}, got {parsed.payloadState.name}")
                    if abs(parsed.pulse - pl.pulse) > 0.001:
                        mismatches.append(f"pulse: expected {pl.pulse}, got {parsed.pulse}")
                    if abs(parsed.powerUp - pl.power_up) > 0.001:
                        mismatches.append(f"powerUp: expected {pl.power_up}, got {parsed.powerUp}")
                    if abs(parsed.powerDown - pl.power_down) > 0.001:
                        mismatches.append(f"powerDown: expected {pl.power_down}, got {parsed.powerDown}")
                    if abs(parsed.deltaD - pl.delta_t) > 0.001:
                        mismatches.append(f"deltaT: expected {pl.delta_t}, got {parsed.deltaD}")

                    if mismatches:
                        for m in mismatches:
                            verify_errors.append(f"Payload ch{pl.channel}: {m}")
                    else:
                        print(f"  Payload ch{pl.channel} ({pl.channel_name}): OK")
                    verified = True
                    break

        if not verified:
            verify_errors.append(f"Payload ch{pl.channel}: no response received")

    # Re-request actuator params
    for act in app.actuators:
        conn.request_parameter(PacketTypes.ACTUATORS_CALIBRATION.value, act.channel)
        time.sleep(0.15)
        results = conn.recv_packets(timeout=1.0, expected_type=PacketTypes.ACTUATORS_CALIBRATION.value)

        verified = False
        for pkt_type, parsed in results:
            if pkt_type == PacketTypes.ACTUATORS_CALIBRATION.value:
                if hasattr(parsed, 'channel') and parsed.channel == act.channel:
                    mismatches = []
                    if parsed.type.value != ActuatorFunction[act.function].value:
                        mismatches.append(f"function: expected {act.function}, got {parsed.type.name}")
                    if parsed.min_usec != act.cal_min:
                        mismatches.append(f"min: expected {act.cal_min}, got {parsed.min_usec}")
                    if parsed.mid_usec != act.cal_center:
                        mismatches.append(f"center: expected {act.cal_center}, got {parsed.mid_usec}")
                    if parsed.max_usec != act.cal_max:
                        mismatches.append(f"max: expected {act.cal_max}, got {parsed.max_usec}")

                    if mismatches:
                        for m in mismatches:
                            verify_errors.append(f"Actuator ch{act.channel}: {m}")
                    else:
                        print(f"  Actuator ch{act.channel} ({act.function}): OK")
                    verified = True
                    break

        if not verified:
            verify_errors.append(f"Actuator ch{act.channel}: no response received")

    # Re-request serial
    if app.serial is not None and PayloadSerial is not None:
        conn.request_parameter(PacketTypes.PAYLOAD_SERIAL.value)
        time.sleep(0.15)
        results = conn.recv_packets(timeout=1.0, expected_type=PacketTypes.PAYLOAD_SERIAL.value)

        verified = False
        for pkt_type, parsed in results:
            if pkt_type == PacketTypes.PAYLOAD_SERIAL.value:
                mismatches = []
                if parsed.baudRate != app.serial.baud_rate:
                    mismatches.append(f"baudRate: expected {app.serial.baud_rate}, got {parsed.baudRate}")
                if parsed.payloadInterface.value != PayloadInterface[app.serial.serial_interface].value:
                    mismatches.append(f"interface: expected {app.serial.serial_interface}, got {parsed.payloadInterface.name}")

                if mismatches:
                    for m in mismatches:
                        verify_errors.append(f"Serial: {m}")
                else:
                    print(f"  Serial (baud={app.serial.baud_rate}): OK")
                verified = True
                break

        if not verified:
            verify_errors.append("Serial: no response received")

    return verify_errors


def app_has_serial_in_xml(xml_path):
    """Quick check if XML has a PayloadSerial section."""
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        if root.tag == 'param':
            vehicle = root.find('vehicle')
            return vehicle is not None and vehicle.find('payload_serial') is not None
        return root.find('PayloadSerial') is not None
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    available = get_available_versions()
    default_ver = get_default_version()

    parser = argparse.ArgumentParser(
        description='Load and upload app parameters from XML to a BST autopilot')
    parser.add_argument('xml_file', help='Path to app XML file')
    parser.add_argument('--host', default='127.0.0.1',
                        help='GCS daemon host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=55555,
                        help='GCS daemon port (default: 55555)')
    parser.add_argument('--comm-version', type=int, default=default_ver,
                        metavar='VER',
                        help=f'Comm protocol version (default: {default_ver}, '
                             f'available: {", ".join(str(v) for v in available)})')
    parser.add_argument('--validate-only', action='store_true',
                        help='Only validate the XML, do not connect or upload')
    parser.add_argument('--no-verify', action='store_true',
                        help='Skip read-back verification after upload')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed packet-level communication')
    args = parser.parse_args()

    # Load the requested comm protocol version
    if args.comm_version not in available:
        print(f"ERROR: Comm version {args.comm_version} not found. "
              f"Available: {', '.join(str(v) for v in available)}", file=sys.stderr)
        sys.exit(1)

    load_comm_version(args.comm_version)
    print(f"Using comm protocol version: {args.comm_version}")

    # Step 1: Parse XML
    print(f"Loading app XML: {args.xml_file}")
    try:
        app = parse_app_xml(args.xml_file)
    except ET.ParseError as e:
        print(f"ERROR: Malformed XML: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: Failed to parse XML: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"App: {app.name}")
    print(f"  Payloads:  {len(app.payloads)}")
    print(f"  Actuators: {len(app.actuators)}")
    print(f"  Serial:    {'yes' if app.serial else 'no'}")

    # Step 2: Validate
    print("\nValidating parameters...")
    ok, errors, warnings = validate_app(app)

    for w in warnings:
        print(f"  WARNING: {w}")
    for e in errors:
        print(f"  ERROR: {e}")

    if ok:
        print("  Validation passed.")
    else:
        print(f"\n  Validation FAILED with {len(errors)} error(s).")
        sys.exit(1)

    if args.validate_only:
        print("\n--validate-only: done.")
        sys.exit(0)

    # Step 3: Connect
    conn = AutopilotConnection(args.host, args.port, args.verbose)
    try:
        conn.connect()
    except Exception as e:
        print(f"ERROR: Could not connect to GCS daemon: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        # Step 3.5: Detect firmware version and warn on mismatch
        detect_firmware_version(conn, args.comm_version, available)

        # Step 4: Request current parameters
        request_current_params(conn, app)

        # Step 5: Display what will change
        has_changes = compare_and_display(conn, app)
        if not has_changes:
            print("Nothing to upload.")
            return

        # Step 6: Upload
        upload_errors = upload_parameters(conn, app)

        # Step 7: Verify (unless --no-verify)
        verify_errors = []
        if not args.no_verify:
            time.sleep(0.5)  # let autopilot process
            verify_errors = verify_parameters(conn, app)

        # Summary
        all_errors = upload_errors + verify_errors
        if all_errors:
            print(f"\nCompleted with {len(all_errors)} issue(s):")
            for e in all_errors:
                print(f"  - {e}")
            sys.exit(1)
        else:
            print("\nAll parameters uploaded and verified successfully.")

    finally:
        conn.disconnect()


if __name__ == '__main__':
    main()
