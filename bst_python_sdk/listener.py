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

from .comm_packets.comm_packets import PacketTypes
from .comm_packets.canpackets import CAN_GNSS_LLA

def parse_stream(data, handler, addressing=True):
    pkt = BSTPacket()
    pkt.set_addressing(addressing)

    i = 0
    result = None
    num_bytes = len(data)

    while i < num_bytes:
        if pkt.parse(data[i:]):
            if pkt.CAN_PKT_SIZE:
                result = handler.simulated_can_handler(pkt)
            else:
                result = handler.standard_handler(pkt)

            i = i + pkt.SIZE + pkt.OVERHEAD
        else:
            i = i + 1

    return result
