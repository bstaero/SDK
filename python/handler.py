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

from comm_packets.comm_packets import PacketTypes
from comm_packets.payload import *
from comm_packets.fixedwing import *
from comm_packets.canpackets import *

packet_mapping = {
    PacketTypes.TELEMETRY_CONTROL: TelemetryControl(),
    PacketTypes.TELEMETRY_ORIENTATION: TelemetryOrientation(),
    PacketTypes.TELEMETRY_POSITION: TelemetryPosition(),
    PacketTypes.TELEMETRY_PRESSURE: TelemetryPressure(),
    PacketTypes.TELEMETRY_SYSTEM: TelemetrySystem(),
    PacketTypes.PAYLOAD_CHANNEL_0: UserPayload()
}

can_actuators = CAN_Actuator()


def simulated_can_handler(pkt):
    # TODO: Need to check pkt.ID == CAN_Actuator
    if pkt.TYPE is PacketTypes.HWIL_CAN:
        try:
            can_actuators.parse(pkt.DATA)
            return can_actuators
        except BufferError as ErrorMessage:
            print(ErrorMessage)


def standard_handler(pkt):
    if (pkt.FROM & 0xFF000000) == 0x41000000:
        packet_data = None

        if pkt.TYPE not in packet_mapping:
            print('Packet type not yet set up for parsing')
            return None

        try:
            if pkt.TYPE >= PacketTypes.PAYLOAD_CHANNEL_0:
                packet_mapping[pkt.TYPE].buffer = [None] * 64

            packet_data = packet_mapping[pkt.TYPE].parse(pkt.DATA)
        except BufferError as ErrorMessage:
            print(ErrorMessage)

        return packet_data
