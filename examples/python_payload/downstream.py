#*=+--+=#=+--            SwiftPilot Autopilot Software            --+=#=+--+=#*#
#               Copyright (C) 2019 Black Swift Technologies LLC.               #
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

import struct

class Downstream:
	SIZE = 64

	def __init__ (self, system_time = 0.0, gas_ppm = [None] * 4, co2_ppm = 0, particle_counts = [None] * 6,
	latitude = 0.0, longitude = 0.0, altitude = 0.0):
		self.system_time = system_time

		if (len(gas_ppm) != 4):
			raise ValueError('array gas_ppm expecting length '+str(4)+' got '+str(len(gas_ppm)))

		self.gas_ppm = list(gas_ppm)

		self.co2_ppm = co2_ppm

		if (len(particle_counts) != 6):
			raise ValueError('array particle_counts expecting length '+str(4)+' got '+str(len(particle_counts)))

		self.particle_counts = list(particle_counts)

		self.latitude = latitude
		self.longitude = longitude
		self.altitude = altitude

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [Downstream]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.system_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset+struct.calcsize('<f')

		self.gas_ppm = [];

		for i in range(0,4):
                    self.gas_ppm.append(struct.unpack_from('<f',buf,offset)[0])
                    offset = offset+struct.calcsize('<f')

		self.co2_ppm = struct.unpack_from('<h',buf,offset)[0]
		offset = offset+struct.calcsize('<h')
                
		self.particle_counts = [];

		for i in range(0,6):
                    self.particle_counts.append(struct.unpack_from('<L',buf,offset)[0])
                    offset = offset+struct.calcsize('<L')

		self.latitude = struct.unpack_from('<f',buf,offset)[0]
		offset = offset+struct.calcsize('<f')

		self.longitude = struct.unpack_from('<f',buf,offset)[0]
		offset = offset+struct.calcsize('<f')

		self.altitude = struct.unpack_from('<f',buf,offset)[0]
		offset = offset+struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.system_time))

		for val in self.gas_ppm:
		    buf.extend(struct.pack('<f', val))

		buf.extend(struct.pack('<h', self.co2_ppm))

		for val in self.particle_counts:
		    buf.extend(struct.pack('<L', particle_counts))

		buf.extend(struct.pack('<f', self.latitude))
		buf.extend(struct.pack('<f', self.longitude))
		buf.extend(struct.pack('<f', self.altitude))
		return bytearray(buf)
