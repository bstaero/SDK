# #=+--+=#=+--             Black Swift Technologies SDK           --+=#=+--+=# #
#               Copyright (C) 2020 Black Swift Technologies LLC.               #
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
#                                  Ben Busby                                   #
#                         ben.busby@blackswifttech.com                         #
#                                                                              #
# *#=+--+=#=+--                 --+=#=+--+=#=+--                 --+=#=+--+=#* #

from .bstpacket import BSTPacket
from .comm_packets.handler import standard_handler
from .comm_packets.comm_packets import VehicleType, PacketTypes
from . import swig_parser
import copy
import importlib
import numpy as np
import scipy.io as spio
import os.path
from .xml_payloads import XMLUserPayload

from .user_payloads.s0_extra_packets import *

pkt = BSTPacket()

# Flight modes
CLIMBOUT = 4
FLYING = 5
LANDING = 6
LANDED = 7


gcs_name: str = "SwiftStation"
unknown_ac: str = "unknown_ac"
log_suffix: str = '_log_1'

s0_user_payload = S0UserPayload();

# Address for the single implicit aircraft when the log has no addressing
NO_ADDR: int = 0

# Comms rev at which the Telemetry* packets began reporting system_time in
# seconds; before it every packet reported milliseconds, and even after it the
# remaining types (State, GPS, MHP, Pressure, sensors) still do. Mixing the two
# makes the same instant read as 105.287 from one packet and 105058 from the
# next, so every comparison across types has to go through seconds first.
SYSTEM_TIME_SECONDS_REV: int = 3180
# Packets that decode a system_time of their own, so their unit is known. Every
# other type is stamped by the handler with its running clock and inherits
# whatever unit last fed it, which is why they cannot be compared against these.
CLOCK_PACKETS: frozenset = frozenset((
    'TELEMETRY_CONTROL', 'TELEMETRY_ORIENTATION', 'TELEMETRY_POSITION',
    'TELEMETRY_PRESSURE', 'TELEMETRY_SYSTEM',
))
# The S0 payload carries a float seconds timestamp at every rev
ALWAYS_SECONDS_PACKETS: frozenset = frozenset(('PAYLOAD_S0_SENSORS',))


class SourceState:
    """Decode state for one packet source (an aircraft, or the ground station).

    A GCS log can hold several aircraft at once, each powering up and rolling its
    system time independently. Tracking this per FROM address keeps them apart;
    a single shared state attributed every packet to the last SYSTEM_INITIALIZE
    seen and read each switch between airframes as a new log.
    """

    def __init__(self, has_addr=True):
        self.current_ac = unknown_ac
        if has_addr:
            self.current_ac = f'{unknown_ac}{log_suffix}'
        self.vehicle_type = VehicleType.VEHICLE_UNKNOWN
        self.sys_current_time = 0
        self.sys_previous_time = 0
        self.prev_pkt_time = 0
        self.prev_clock_seconds = 0
        self.sys_init_times = {}


class Parser:
    def __init__(self, has_addr=True, quick_mode=False, verbose=False, xml_payload_path=''):
        self.has_addr = has_addr
        self.verbose = verbose
        self.quick_mode = quick_mode

        self.parsed_logs = {}
        self.failed_pkts = {}
        self.gcs_sys_time = 0

        self.prev_type = 0

        self.comms_rev = 0

        # FROM address -> SourceState, created as addresses appear
        self.ac_states = {}
        self.gcs_state = SourceState(has_addr)
        # Most recent aircraft vehicle type, used to decode GCS-relayed packets
        self.ac_vehicle_type = VehicleType.VEHICLE_UNKNOWN

        self.results = {gcs_name: {}}

        if len(xml_payload_path) > 0:
            xml = XMLUserPayloads(xml_payload_path)
            self.payload_classes = xml.payload_classes
        else:
            self.payload_classes = []


    def reimport_comms(self, new_rev: int):
        print(f'-- Using comms rev: {new_rev}')
        self.comms_rev = new_rev

        handler_import = f'.comm_versions.ver_{new_rev}.handler'
        comm_packets_import = f'.comm_versions.ver_{new_rev}.comm_packets'

        handler = importlib.import_module(
                handler_import, package="bst_python_sdk")
        comm_packets = importlib.import_module(
                comm_packets_import, package="bst_python_sdk")

        globals()['standard_handler'] = handler.standard_handler
        globals()['VehicleType'] = comm_packets.VehicleType
        globals()['PacketTypes'] = comm_packets.PacketTypes

    def system_time_scale(self, pkt):
        """Multiplier turning this packet's system_time into seconds.

        None when the packet does not carry a clock of its own, so its stamp
        cannot be trusted to say whether time went backwards.
        """
        try:
            name = PacketTypes(pkt.TYPE).name
        except ValueError:
            return None
        if name in ALWAYS_SECONDS_PACKETS:
            return 1.0
        if name in CLOCK_PACKETS:
            return 1.0 if self.comms_rev >= SYSTEM_TIME_SECONDS_REV else 0.001
        return None

    def is_from_aircraft(self, pkt) -> bool:
        """True when a packet came from an aircraft rather than the ground station."""
        return (pkt.FROM & 0xFF000000) == 0x41000000 or not self.has_addr

    def source_state(self, pkt) -> SourceState:
        """Return the state for whichever source sent pkt, creating it if new."""
        if not self.is_from_aircraft(pkt):
            return self.gcs_state
        addr = pkt.FROM if self.has_addr else NO_ADDR
        if addr not in self.ac_states:
            state = SourceState(self.has_addr)
            if self.has_addr:
                # Tag the placeholder with the address so packets arriving before
                # this aircraft is named don't pile in with another unnamed one
                state.current_ac = f'{unknown_ac}_{addr:08X}{log_suffix}'
            self.ac_states[addr] = state
        return self.ac_states[addr]

    def parse_log(self, filename: str) -> dict:
        bst_packets = swig_parser.parse(filename, self.has_addr, self.quick_mode)

        for pkt in bst_packets:
            if self.is_from_aircraft(pkt):
                # AC packet - decoded against its own aircraft's clock
                state = self.source_state(pkt)
                if state.sys_current_time > state.sys_previous_time:
                    state.sys_previous_time = state.sys_current_time
                parsed_data, state.sys_current_time = standard_handler(
                    pkt,
                    state.sys_current_time,
                    state.vehicle_type)
                state.sys_current_time = max(
                    state.sys_current_time,
                    state.sys_previous_time)
            else:
                # GCS packet - TODO: ignore tablet request packets for now
                # Decoded against the aircraft's vehicle type: the ground station
                # relays that aircraft's packets, and some layouts differ by type
                if pkt.ACTION != 1:
                    parsed_data, self.gcs_sys_time = standard_handler(
                        pkt,
                        self.gcs_sys_time,
                        self.ac_vehicle_type)
                else:
                    parsed_data = None

            if parsed_data is not None:
                self.add_packet(pkt, parsed_data)

        return self.results

    def add_packet(self, pkt, pkt_data):
        # TODO: Need to refactor this and/or move parsing to swig code
        from_aircraft = self.is_from_aircraft(pkt)
        state = self.source_state(pkt)

        is_sys_init = pkt.TYPE == PacketTypes.SYSTEM_INITIALIZE.value
        is_telem_sys = pkt.TYPE == PacketTypes.TELEMETRY_SYSTEM.value
        is_telem_ctrl = pkt.TYPE == PacketTypes.TELEMETRY_CONTROL.value
        is_telem_pos = pkt.TYPE == PacketTypes.TELEMETRY_POSITION.value
        is_telem_orient = pkt.TYPE == PacketTypes.TELEMETRY_ORIENTATION.value
        is_telem_pres = pkt.TYPE == PacketTypes.TELEMETRY_PRESSURE.value

        is_pyld_data = (
            self.comms_rev > 3140 and
            pkt.TYPE >= PacketTypes.PAYLOAD_DATA_CHANNEL_0.value and
            pkt.TYPE <= PacketTypes.PAYLOAD_DATA_CHANNEL_7.value
        )

        has_sys_time = hasattr(pkt_data, 'system_time')

        prev_seconds = state.prev_clock_seconds
        to_seconds = self.system_time_scale(pkt)

        if has_sys_time and pkt_data.system_time != 0:
            state.prev_pkt_time = pkt_data.system_time
            if to_seconds is not None:
                state.prev_clock_seconds = pkt_data.system_time * to_seconds
            #print(f'adding system time {state.prev_pkt_time} from {pkt.TYPE}')

        if is_pyld_data and self.comms_rev < 3200:
            payload_num = pkt.TYPE - PacketTypes.PAYLOAD_DATA_CHANNEL_0.value
            try:
                payload_class = self.payload_classes[payload_num]
                payload_class.parse(bytes(pkt_data.buffer))
                if payload_class.system_time != 0:
                    state.prev_pkt_time = payload_class.system_time
                pkt_data = copy.deepcopy(payload_class)
            except BufferError as ErrorMessage:
                print(ErrorMessage)
            except IndexError:
                pass

        if not has_sys_time and is_telem_ctrl or is_telem_sys or is_telem_pos or is_telem_orient or is_telem_pres:
            pkt_data.system_time = state.prev_pkt_time

        # A power cycle restarts the clock, so it drops by more than 100 s.
        # Not applied while a source is still under a placeholder name: until an
        # aircraft identifies itself the broadcast address can be carrying
        # several of them at once, and their interleaved clocks read as a restart
        # on nearly every packet, shredding the log into one-fix fragments.
        is_new_sys_time = (has_sys_time and to_seconds is not None
                           and not state.current_ac.startswith(unknown_ac)
                           and pkt_data.system_time != 0
                           and prev_seconds - pkt_data.system_time * to_seconds > 100)

        if is_sys_init:
            sys_init_pkt: SystemInitialize = pkt_data
            if sys_init_pkt.comms_rev != self.comms_rev:
                # Everything decoded before the rev was known used whatever
                # classes were loaded last - the previous log's, when several are
                # converted in one process. Those objects carry the wrong wire
                # format, and leaving them in a group alongside correctly decoded
                # ones makes the field lists disagree and fails the conversion of
                # the whole log. They are garbage regardless, so drop them.
                stale = self.comms_rev == 0 and any(self.results.values())
                self.reimport_comms(sys_init_pkt.comms_rev)
                if stale:
                    self.results = {gcs_name: {}}
                    for st in self.ac_states.values():
                        st.current_ac = unknown_ac if not self.has_addr else st.current_ac

        if from_aircraft:
            if is_sys_init:
                sys_init_pkt: SystemInitialize = pkt_data
                state.vehicle_type = VehicleType(sys_init_pkt.vehicle_type.value)
                self.ac_vehicle_type = state.vehicle_type

                # Extract name and trim trailing 0s in name byte array
                name_arr = sys_init_pkt.name
                while name_arr[len(name_arr)-1] == 0:
                    del name_arr[len(name_arr)-1]

                ac_name = "".join(map(chr, sys_init_pkt.name))
                if state.current_ac.startswith(unknown_ac):
                    # First SYSTEM_INITIALIZE here - rename the placeholder in
                    # place, keeping what it collected and leaving others alone
                    new_ac = self.free_log_name(ac_name)
                    if state.current_ac in self.results:
                        self.results[new_ac] = self.results.pop(state.current_ac)
                    state.current_ac = new_ac
                elif not state.current_ac.startswith(ac_name):
                    # This address is now reporting a different airframe
                    state.current_ac = self.free_log_name(ac_name)

                prev_sys_init_time = state.sys_init_times.get(state.current_ac, 0)

                if sys_init_pkt.system_time < prev_sys_init_time:
                    # print(f"new sys init time - type: {pkt.TYPE} prev: {sys_init_pkt.system_time} this: {prev_sys_init_time}")
                    state.current_ac = self.increment_log_name(state.current_ac)

                state.sys_init_times[state.current_ac] = sys_init_pkt.system_time
            elif is_new_sys_time:
                # Same aircraft, new log data
                state.current_ac = self.increment_log_name(state.current_ac)
                state.sys_current_time = pkt_data.system_time
                state.sys_previous_time = pkt_data.system_time

            entry_name = state.current_ac
        else:
            entry_name = gcs_name

        if entry_name not in self.results:
            self.results[entry_name] = {}

        pkt_type = PacketTypes(pkt.TYPE)
        if pkt_type.name in self.results[entry_name]:
            self.results[entry_name][pkt_type.name].append(pkt_data)
        else:
            self.results[entry_name][pkt_type.name] = [pkt_data]

        self.prev_type = pkt.TYPE

    def free_log_name(self, ac_name: str) -> str:
        """Return the first {ac_name}_log_N no source has claimed yet.

        Two addresses can report the same name (a board swapped between
        airframes), and results is keyed by name alone. An unaddressed log has
        only one aircraft, so it keeps the bare name and never splits.
        """
        if not self.has_addr:
            return ac_name
        name = f'{ac_name}{log_suffix}'
        while self.results.get(name):
            name = self.increment_log_name(name)
        return name

    def increment_log_name(self, name: str) -> str:
        try:
            split_name = name.split('_')
            ac_name = '_'.join(split_name[0:len(split_name)-1])
            new_log_num = int(split_name[-1]) + 1
            return f'{ac_name}_{new_log_num}'
        except:
            return name


def find_system_info(filename, has_addressing=False):
    try:
        with open(filename, "rb") as binary_file:
            binary_file.seek(0, 2)  # Seek the end
            num_bytes = binary_file.tell()  # Get the file size

            i = 0

            while i < num_bytes:
                binary_file.seek(i)
                pkt_data = binary_file.read(BSTPacket.BST_MAX_PACKET_SIZE)

                if pkt.parse(pkt_data, has_addressing):
                    if pkt.TYPE is PacketTypes.SYSTEM_INITIALIZE:
                        return pkt

                    i = i + pkt.SIZE + pkt.OVERHEAD
                else:
                    i = i + 1

    except IOError:
        pass

    return None


def find_tof(logfile):
    launch, land = find_in_flight(logfile)
    t_flt = []
    for idx in range(launch.__len__()):
        t_flt.append((land[idx] - launch[idx]) / 60)
    return t_flt


# Returns when the aircraft is airborne (climbout to landed)
def find_in_flight(flight):
    data = load_mat_file(flight)
    in_flight_mode = np.where(data['command']['id'][()] == 10)
    t_mode = data['command']['system_time'][()][in_flight_mode]
    mode = data['command']['value'][()][in_flight_mode]

    t_launch = []
    t_land = []
    do_loop = True
    while do_loop:
        in_launch = np.where(mode == CLIMBOUT)
        if in_launch[0].__len__() > 0:
            in_launch = in_launch[0][0]
            t_launch.append(t_mode[in_launch])
            in_land = np.where(mode[in_launch:-1] == LANDED)
            if in_land[0].__len__() > 0:
                in_land = in_land[0][0]
                t_land.append(t_mode[in_launch + in_land])
                mode = mode[in_land + in_launch:-1]
                t_mode = t_mode[in_land + in_launch:-1]
            else:
                t_land.append(t_mode[-1])
                do_loop = False
        else:
            do_loop = False
    return t_launch, t_land


# Returns when the aircraft is in "flying mode"
def find_in_flying(flight):
    data = load_mat_file(flight)
    in_flight_mode = np.where(data['command']['id'][()] == 10)
    t_mode = data['command']['system_time'][()][in_flight_mode]
    mode = data['command']['value'][()][in_flight_mode]

    t_launch = []
    t_land = []
    do_loop = True
    while do_loop:
        in_launch = np.where(mode == CLIMBOUT)
        if in_launch[0].__len__() > 0:
            in_launch = in_launch[0][0]
            t_launch.append(t_mode[in_launch])
            in_landing = np.where(mode[in_launch:-1] == LANDING)
            in_landed = np.where(mode[in_launch:-1] == LANDED)
            if in_landing[0].__len__() > 0 and in_landed[0].__len__() > 0:
                # Grab the earlier one (see 2017-10-12 for why this is needed)
                in_landing = in_landing[0][0]
                in_landed = in_landed[0][0]
                if in_landed < in_landing:
                    t_land.append(t_mode[in_launch + in_landed])
                    mode = mode[in_landed + in_launch:-1]
                    t_mode = t_mode[in_landed + in_launch:-1]
                else:
                    t_land.append(t_mode[in_launch + in_landing])
                    mode = mode[in_landing + in_launch:-1]
                    t_mode = t_mode[in_landing + in_launch:-1]
            elif in_landing[0].__len__() > 0:
                in_landing = in_landing[0][0]
                t_land.append(t_mode[in_launch + in_landing])
                mode = mode[in_landing + in_launch:-1]
                t_mode = t_mode[in_landing + in_launch:-1]
            elif in_landed[0].__len__() > 0:
                in_landed = in_landed[0][0]
                t_land.append(t_mode[in_launch + in_landed])
                mode = mode[in_landed + in_launch:-1]
                t_mode = t_mode[in_landed + in_launch:-1]
            else:
                t_land.append(t_mode[-1])
                do_loop = False
        else:
            do_loop = False
    return t_launch, t_land


# Returns full log file while GPS is good.
def find_good_gps(flight):
    data = load_mat_file(flight)
    gps_acquired = np.where(data['gps']['hdop'][()] <= 3)[0][0]
    t_launch = []
    t_land = []
    t_launch.append(data['gps']['system_time'][()][gps_acquired])
    t_land.append(data['gps']['system_time'][()][-1])
    return t_launch, t_land


def load_mat_file(filename):
    if hasattr(filename, 'hasMatFile'):
        # It's a flight object, return the mat file data
        filename = os.path.join(filename.directory_path, 'logs', filename.mat_file)
        return spio.loadmat(filename, squeeze_me=True)
    elif isinstance(filename, str):
        # A string, probably a log file
        return spio.loadmat(filename, squeeze_me=True)
    elif isinstance(filename, dict):
        # Already a mat struct, just return it.
        return filename
    else:
        exit('loadMatFile Error: Unknown type')
