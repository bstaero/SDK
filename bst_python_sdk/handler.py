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

from .comm_packets.comm_packets import *
from .comm_packets.payload import *
from .comm_packets.fixedwing import *
from .comm_packets.canpackets import *

packet_mapping = {
    PacketTypes.TELEMETRY_CONTROL.value: TelemetryControl(),
    PacketTypes.TELEMETRY_ORIENTATION.value: TelemetryOrientation(),
    PacketTypes.TELEMETRY_POSITION.value: TelemetryPosition(),
    PacketTypes.TELEMETRY_PRESSURE.value: TelemetryPressure(),
    PacketTypes.TELEMETRY_SYSTEM.value: TelemetrySystem(),
    PacketTypes.TELEMETRY_PAYLOAD.value: TelemetryPayload(),
    PacketTypes.PAYLOAD_DATA_CHANNEL_0.value: UserPayload()
}

can_actuators = CAN_Actuator()


def simulated_can_handler(pkt):
    if pkt.TYPE is PacketTypes.HWIL_CAN:
        if pkt.PKT_ID == CAN_PacketTypes.CAN_PKT_ACTUATOR:
            try:
                can_actuators.parse(pkt.DATA)
                return can_actuators
            except BufferError as ErrorMessage:
                print(ErrorMessage)
        else:
            # TODO: Add parsing for other CAN packets
            pass


def standard_handler(pkt):
    if (pkt.FROM & 0xFF000000) == 0x41000000:
        packet_data = None

        if pkt.TYPE.value not in packet_mapping:
            print('Packet type ',pkt.TYPE.value,' not yet set up for parsing')
            return None

        try:
            if pkt.TYPE.value >= PacketTypes.PAYLOAD_DATA_CHANNEL_0.value:
                packet_mapping[pkt.TYPE.value].buffer = [None] * 64

            print(pkt.DATA)
            packet_mapping[pkt.TYPE.value].parse(pkt.DATA)
            packet_data = packet_mapping[pkt.TYPE.value]
        except BufferError as ErrorMessage:
            print(ErrorMessage)

        return packet_data
