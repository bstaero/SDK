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
import importlib
import numpy as np
import scipy.io as spio
import os.path

pkt = BSTPacket()

# Flight modes
CLIMBOUT = 4
FLYING = 5
LANDING = 6
LANDED = 7


gcs_name: str = "SwiftStation"
unknown_ac: str = "unknown_ac"
log_suffix: str = '_log_1'


class Parser:
    def __init__(self, has_addr=True, quick_mode=False, verbose=False):
        self.has_addr = has_addr
        self.verbose = verbose
        self.quick_mode = quick_mode

        self.parsed_logs = {}
        self.failed_pkts = {}
        self.sys_init_times = {}
        self.ac_vehicle_type = VehicleType.VEHICLE_UNKNOWN
        self.ac_sys_current_time = 0
        self.ac_sys_previous_time = 0
        self.gcs_sys_time = 0

        self.comms_rev = 0

        self.current_ac = unknown_ac
        if has_addr:
            self.current_ac = f'{unknown_ac}{log_suffix}'

        self.results = {gcs_name: {}, self.current_ac:{}}

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

    def parse_log(self, filename: str) -> dict:
        bst_packets = swig_parser.parse(filename, self.has_addr, self.quick_mode)

        for pkt in bst_packets:
            if (pkt.FROM & 0xFF000000) == 0x41000000:
                # AC packet
                if self.ac_sys_current_time > self.ac_sys_previous_time:
                    self.ac_sys_previous_time = self.ac_sys_current_time
                parsed_data, self.ac_sys_current_time = standard_handler(
                    pkt,
                    self.ac_sys_current_time,
                    self.ac_vehicle_type)
            else:
                # GCS packet
                parsed_data, self.gcs_sys_time = standard_handler(
                    pkt,
                    self.gcs_sys_time,
                    self.ac_vehicle_type)

            if parsed_data is not None:
                self.add_packet(pkt, parsed_data)

        return self.results

    def add_packet(self, pkt, pkt_data):
        from_aircraft = (pkt.FROM & 0xFF000000) == 0x41000000
        is_sys_init = pkt.TYPE == PacketTypes.SYSTEM_INITIALIZE.value
        has_sys_time = hasattr(pkt_data, 'system_time')
        is_new_sys_time = has_sys_time and pkt_data.system_time < self.ac_sys_previous_time

        if is_sys_init:
            sys_init_pkt: SystemInitialize = pkt_data
            if sys_init_pkt.comms_rev != self.comms_rev:
                self.reimport_comms(sys_init_pkt.comms_rev)

        if from_aircraft or not self.has_addr:
            if is_new_sys_time:
                # Same aircraft, new log data
                self.current_ac = self.increment_log_name(self.current_ac)
                self.ac_sys_previous_time = pkt_data.system_time
            elif is_sys_init:
                sys_init_pkt: SystemInitialize = pkt_data
                self.ac_vehicle_type = VehicleType(sys_init_pkt.vehicle_type.value)

                # Extract name and trim trailing 0s in name byte array
                name_arr = sys_init_pkt.name
                while name_arr[len(name_arr)-1] == 0:
                    del name_arr[len(name_arr)-1]

                ac_name = "".join(map(chr, sys_init_pkt.name))
                if self.current_ac.startswith(unknown_ac):
                    # First aircraft log
                    new_ac = self.current_ac.replace(unknown_ac, ac_name)
                    self.results = {
                        gcs_name: self.results[gcs_name],
                        new_ac: self.results[self.current_ac]
                    }
                    self.current_ac = new_ac
                elif not self.current_ac.startswith(ac_name):
                    # New aircraft log
                    self.current_ac = f'{ac_name}_log_1'
                    try:
                        if len(self.results[self.current_ac]) > 0:
                            self.current_ac = self.increment_log_name(self.current_ac)
                    except:
                        pass

                prev_sys_init_time = 0
                has_prev_sys_init = self.current_ac in self.sys_init_times
                if has_prev_sys_init:
                    prev_sys_init_time = self.sys_init_times[self.current_ac]

                if sys_init_pkt.system_time < prev_sys_init_time:
                    self.current_ac = self.increment_log_name(self.current_ac)

                self.sys_init_times[self.current_ac] = sys_init_pkt.system_time

            entry_name = self.current_ac
        else:
            entry_name = gcs_name

        if entry_name not in self.results:
            self.results[entry_name] = {}

        pkt_type = PacketTypes(pkt.TYPE)
        if pkt_type.name in self.results[entry_name]:
            self.results[entry_name][pkt_type.name].append(pkt_data)
        else:
            self.results[entry_name][pkt_type.name] = [pkt_data]

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
