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

import struct

from .comm_packets.comm_packets import PacketTypes
from .comm_packets.payload import PayloadID


class BSTPacket:
    BST_MAX_PACKET_SIZE = 128

    TYPE = PacketTypes.INVALID_PACKET
    ACTION = 0
    SIZE = 0
    TO = 0
    FROM = 0
    PKT_ID = 0
    CAN_PKT_SIZE = 0
    DATA = bytearray()
    CHKSUM = bytearray(2)
    CHECKSUM_SIZE = 2
    OVERHEAD = 8

    HAS_ADDRESS = False
    VERBOSE = False

    def parse(self, buf, has_address=False):
        if len(buf) < self.OVERHEAD: return False

        self.set_addressing(has_address)

        i = 0
        while i < len(buf):
            hwil = False
            header_bytes = buf[i:i + 2]
            if header_bytes == b"U$":
                i = i + 2

                if self.OVERHEAD > len(buf) - i: return False

                # PKT_TYPE
                pkt_type = buf[i]
                i = i + 1

                try:
                    self.TYPE = PacketTypes(pkt_type)
                    hwil = 'HWIL_' in self.TYPE.name
                except ValueError:
                    try:
                        self.TYPE = PayloadID(pkt_type)
                    except ValueError:
                        return False, False

                # PKT_ACTION
                self.ACTION = buf[i]
                i = i + 1

                # PKT_SIZE
                self.SIZE = struct.unpack_from('<H', buf[i:i + 2])[0]
                i = i + 2

                # Need to parse can pkt size and and pkt id if hwil
                if hwil:
                    # PKT_ID
                    self.PKT_ID = struct.unpack_from('<I', bytes(buf[i:i + 4]))[0]
                    i = i + 4

                    # CAN_PKT_SIZE
                    self.CAN_PKT_SIZE = struct.unpack_from('<B', bytes(buf[i:i + 2]))[0]
                    i = i + 2

                if not hwil and self.SIZE + self.CHECKSUM_SIZE > len(buf) - i: return False

                if self.HAS_ADDRESS:
                    # PKT_TO
                    self.TO = struct.unpack_from('<I', buf[i:i + 4])[0]
                    i = i + 4

                    # PKT_FROM
                    self.FROM = struct.unpack_from('<I', buf[i:i + 4])[0]
                    i = i + 4

                # PKT_DATA
                buf_size = i + self.SIZE if not hwil else i + self.CAN_PKT_SIZE
                self.DATA = buf[i:buf_size]
                i = buf_size

                # PKT_CHKSUM
                self.CHKSUM = buf[i:i + 2]
                i = i + 2

                return hwil or self.check_fletcher_16()
            else:
                # print("Dropping byte")
                i = i + 1
        return False

    def set_addressing(self, on_off):
        if on_off:
            self.OVERHEAD = 16
            self.HAS_ADDRESS = True
        else:
            self.OVERHEAD = 8
            self.HAS_ADDRESS = False

    def set_verbose(self, on_off):
        self.VERBOSE = on_off

    def serialize(self):
        buf = []

        buf.extend('U')
        buf.extend('$')
        buf.extend(struct.pack('<B', self.TYPE.value))
        buf.extend(struct.pack('<B', self.ACTION))
        buf.extend(struct.pack('<H', self.SIZE))

        if self.PKT_ID and self.CAN_PKT_SIZE:
            buf.extend(struct.pack('<I', self.PKT_ID))
            buf.extend(struct.pack('<B', self.CAN_PKT_SIZE))

        if self.HAS_ADDRESS:
            buf.extend(struct.pack('<I', self.TO))
            buf.extend(struct.pack('<I', self.FROM))

        # PKT_DATA
        buf.extend(self.DATA)

        buf.extend(self.CHKSUM)

        # Format non-int values in buf
        for i in range(0, len(buf)):
            try:
                buf[i] = ord(buf[i])
            except TypeError:
                pass

        return bytearray(buf)

    def check_fletcher_16(self):
        sum1 = 0
        sum2 = 0

        raw_data = self.serialize()
        total_size = self.OVERHEAD + self.SIZE

        if len(raw_data) != total_size:
            # raise ValueError(f'[{self.TYPE.name}] -- Bad checksum size: {len(raw_data)} vs {total_size}')
            if self.VERBOSE:
                print(f'[{self.TYPE.name}] -- Bad checksum size: {len(raw_data)} vs {total_size}')
            return False

        for i in range(0, total_size):
            sum1 = (sum1 + struct.unpack_from('<B', raw_data, i)[0]) % 255
            sum2 = (sum2 + sum1) % 255

        checksum_passed = ((sum2 << 8) | sum1) == 0
        if self.VERBOSE:
            print(f'[{self.TYPE.name}] -- Failed checksum')
        return checksum_passed

    def set_fletcher_16(self, data=None, size=None):
        sum1 = 0
        sum2 = 0

        raw_data = self.serialize() if data is None else data
        total_size = self.OVERHEAD + self.SIZE if size is None else size

        for i in range(0, total_size - self.CHECKSUM_SIZE):
            sum1 = (sum1 + struct.unpack_from('<B', raw_data, i)[0]) % 255
            sum2 = (sum2 + sum1) % 255

        if data is None:
            self.CHKSUM[0] = 255 - ((sum1 + sum2) % 255)
            self.CHKSUM[1] = 255 - ((sum1 + self.CHKSUM[0]) % 255)
            checksum = struct.unpack_from('<H', self.CHKSUM)[0]
        else:
            tmp_checksum = bytearray(2)
            tmp_checksum[0] = 255 - ((sum1 + sum2) % 255)
            tmp_checksum[1] = 255 - ((sum1 + tmp_checksum[0]) % 255)
            checksum = struct.unpack_from('<H', tmp_checksum)[0]

        return self.check_fletcher_16(), checksum

    def __str__(self):
        return "BSTPacket\nTYPE: " + str(self.TYPE) + \
               "\nACTION=" + str(self.ACTION) + \
               "\nSIZE=" + str(self.SIZE) + \
               "\nTO=" + str(self.TO) + \
               "\nFROM=" + str(self.FROM) + \
               "\nPKT_ID=" + str(self.PKT_ID) + \
               "\nCAN_PKT_SIZE=" + str(self.CAN_PKT_SIZE) + \
               "\nDATA=" + str(self.DATA) + \
               "\nCHKSUM=" + str(self.CHKSUM) + \
               "\nCHECKSUM_SIZE=" + str(self.CHECKSUM_SIZE) + \
               "\nOVERHEAD=" + str(self.OVERHEAD) + \
               "\nHAS_ADDRESS=" + str(self.HAS_ADDRESS)
