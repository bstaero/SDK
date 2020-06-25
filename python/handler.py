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

telemetry_control = TelemetryControl()
telemetry_orientation = TelemetryOrientation()
telemetry_position = TelemetryPosition()
telemetry_pressure = TelemetryPressure()
telemetry_system = TelemetrySystem()

user_payload = UserPayload()

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

        # -----[ TELEMETRY ]-----%
        if pkt.TYPE is PacketTypes.TELEMETRY_CONTROL:
            try:
                telemetry_control.parse(pkt.DATA)
            except BufferError as ErrorMessage:
                print(ErrorMessage)

        if pkt.TYPE is PacketTypes.TELEMETRY_ORIENTATION:
            try:
                telemetry_orientation.parse(pkt.DATA)
            except BufferError as ErrorMessage:
                print(ErrorMessage)

        if pkt.TYPE is PacketTypes.TELEMETRY_POSITION:
            try:
                telemetry_position.parse(pkt.DATA)
            except BufferError as ErrorMessage:
                print(ErrorMessage)

        if pkt.TYPE is PacketTypes.TELEMETRY_PRESSURE:
            try:
                telemetry_pressure.parse(pkt.DATA)
            except BufferError as ErrorMessage:
                print(ErrorMessage)

        if pkt.TYPE is PacketTypes.TELEMETRY_SYSTEM:
            try:
                telemetry_system.parse(pkt.DATA)
            except BufferError as ErrorMessage:
                print(ErrorMessage)

        # -----[ PAYLOAD ]-----%
        if pkt.TYPE is PacketTypes.PAYLOAD_CHANNEL_0:
            try:
                user_payload.buffer = [None] * 64
                user_payload.parse(pkt.DATA)
                # print "Got %i bytes from the payload: [%s]" % (
                #        user_payload.size,
                #        ''.join(chr(e) for e in user_payload.buffer))
                print("Got bytes from the payload")

            except BufferError as ErrorMessage:
                print(ErrorMessage)
