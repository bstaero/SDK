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
from .comm_packets.comm_packets import *
import numpy as np
import scipy.io as spio
import os.path

pkt = BSTPacket()

# Flight modes
CLIMBOUT = 4
FLYING = 5
LANDING = 6
LANDED = 7


def parse_log(filename, handler):
    try:
        with open(filename, "rb") as binary_file:
            binary_file.seek(0, 2)  # Seek the end
            num_bytes = binary_file.tell()  # Get the file size

            i = 0

            while i < num_bytes:
                binary_file.seek(i)
                pkt_data = binary_file.read(BSTPacket.BST_MAX_PACKET_SIZE)

                if pkt.parse(pkt_data):
                    handler(pkt)

                    i = i + pkt.SIZE + pkt.OVERHEAD
                else:
                    i = i + 1

    except IOError:
        print("Could not open file: " + filename)


def find_system_info(filename):
    try:
        with open(filename, "rb") as binary_file:
            binary_file.seek(0, 2)  # Seek the end
            num_bytes = binary_file.tell()  # Get the file size

            i = 0

            while i < num_bytes:
                binary_file.seek(i)
                pkt_data = binary_file.read(BSTPacket.BST_MAX_PACKET_SIZE)

                if pkt.parse(pkt_data):
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
