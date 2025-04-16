from enum import Enum
import struct
import sys

class S0UserPayload:
	SIZE = 64

	def __init__ (self, system_time = 0.0, static_pressure = 0.0,
	dynamic_pressure = [None] * 5, air_temperature = 0.0,
	humidity = 0.0, laser_distance = 0.0, ground_temperature = 0.0):
		self.system_time = system_time
		self.static_pressure = static_pressure

		if (len(dynamic_pressure) != 5):
			raise ValueError('array dynamic_pressure expecting length '+str(5)+' got '+str(len(dynamic_pressure)))

		self.dynamic_pressure = list(dynamic_pressure)

		self.air_temperature = air_temperature
		self.humidity = humidity
		self.laser_distance = laser_distance
		self.ground_temperature = ground_temperature

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [S0UserPayload]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.system_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.static_pressure = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.dynamic_pressure = [];

		for i in range(0,5):
			self.dynamic_pressure.append(struct.unpack_from('<f',buf,offset)[0])
			offset = offset+struct.calcsize('<f')

		self.air_temperature = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.humidity = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.laser_distance = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.ground_temperature = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.system_time))
		buf.extend(struct.pack('<f', self.static_pressure))

		for val in self.dynamic_pressure:
		    buf.extend(struct.pack('<f', val))

		buf.extend(struct.pack('<f', self.air_temperature))
		buf.extend(struct.pack('<f', self.humidity))
		buf.extend(struct.pack('<f', self.laser_distance))
		buf.extend(struct.pack('<f', self.ground_temperature))

		buf.extend(struct.pack('<f', 0.0))
		buf.extend(struct.pack('<f', 0.0))
		buf.extend(struct.pack('<f', 0.0))
		buf.extend(struct.pack('<f', 0.0))
		buf.extend(struct.pack('<f', 0.0))
		return bytearray(buf)
