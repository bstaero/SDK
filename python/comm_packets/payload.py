#*=+--+=#=+--         SwiftCore Flight Management Software        --+=#=+--+=#*#
#               Copyright (C) 2020 Black Swift Technologies LLC.               #
#                             All Rights Reserved.                             #
#                                                                              #
#    NOTICE:  All information contained herein is, and remains the property    #
#    of Black Swift Technologies.                                              #
#                                                                              #
#    The intellectual and technical concepts contained herein are              #
#    proprietary to Black Swift Technologies LLC and may be covered by U.S.    #
#    and foreign patents, patents in process, and are protected by trade       #
#    secret or copyright law.                                                  #
#                                                                              #
#    Dissemination of this information or reproduction of this material is     #
#    strictly forbidden unless prior written permission is obtained from       #
#    Black Swift Technologies LLC.                                             #
#                                                                              #
#*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*#

from enum import Enum
import struct

from .comm_packets import *

#                      THIS FILE IS AUTOGENERATED BY                           #
#                                 msg-gen.py                                   #
#                                DO NOT EDIT                                   #

#---------[ Controller ]---------#

class CommandID (Enum):
	# NOTE - you must check the numbers in the CommandID values

	# contained in the parent folder

	#


	# Command Interface

	CMD_PAYLOAD_CONTROL=35

#---------[ PAYLOAD ]---------#

class PayloadControl (Enum):
	PAYLOAD_CTRL_OFF=0
	PAYLOAD_CTRL_CONNECTED=1
	PAYLOAD_CTRL_READY=2
	PAYLOAD_CTRL_ACTIVE=3
	PAYLOAD_CTRL_SHUTDOWN=4
	PAYLOAD_CTRL_ERROR=5
	PAYLOAD_CTRL_INVALID=6

#---------[ Payload Sensors ]---------#

PARTICLESPLUS_MAX_CHANNELS = 6

class LDCRPlatformType (Enum):
	PLATFORM_TYPE_UAS=0
	PLATFORM_TYPE_PORTABLE=1
	PLATFORM_TYPE_FIXED=2

class K30:
	SIZE = 8

	def __init__ (self, system_time = 0.0, co2 = 0, temp = 0):
		self.system_time = system_time
		self.co2 = co2
		self.temp = temp

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [K30]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.system_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.co2 = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

		self.temp = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.system_time))
		buf.extend(struct.pack('<H', self.co2))
		buf.extend(struct.pack('<H', self.temp))
		return bytearray(buf)

class MiniGAS:
	SIZE = 60

	def __init__ (self, system_time = 0.0, gas01_mv = 0.0, gas01_ppm = 0.0,
	gas02_mv = 0.0, gas02_ppm = 0.0, gas03_mv = 0.0, gas03_ppm = 0.0,
	gas04_mv = 0.0, gas04_ppm = 0.0, co2_ppm = 0.0, h20_hpa = 0.0,
	co2_int_temp = 0.0, air_temp = 0.0, logger_temp = 0.0, pressure = 0.0):
		self.system_time = system_time
		self.gas01_mv = gas01_mv
		self.gas01_ppm = gas01_ppm
		self.gas02_mv = gas02_mv
		self.gas02_ppm = gas02_ppm
		self.gas03_mv = gas03_mv
		self.gas03_ppm = gas03_ppm
		self.gas04_mv = gas04_mv
		self.gas04_ppm = gas04_ppm
		self.co2_ppm = co2_ppm
		self.h20_hpa = h20_hpa
		self.co2_int_temp = co2_int_temp
		self.air_temp = air_temp
		self.logger_temp = logger_temp
		self.pressure = pressure

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [MiniGAS]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.system_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.gas01_mv = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.gas01_ppm = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.gas02_mv = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.gas02_ppm = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.gas03_mv = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.gas03_ppm = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.gas04_mv = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.gas04_ppm = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.co2_ppm = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.h20_hpa = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.co2_int_temp = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.air_temp = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.logger_temp = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.pressure = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.system_time))
		buf.extend(struct.pack('<f', self.gas01_mv))
		buf.extend(struct.pack('<f', self.gas01_ppm))
		buf.extend(struct.pack('<f', self.gas02_mv))
		buf.extend(struct.pack('<f', self.gas02_ppm))
		buf.extend(struct.pack('<f', self.gas03_mv))
		buf.extend(struct.pack('<f', self.gas03_ppm))
		buf.extend(struct.pack('<f', self.gas04_mv))
		buf.extend(struct.pack('<f', self.gas04_ppm))
		buf.extend(struct.pack('<f', self.co2_ppm))
		buf.extend(struct.pack('<f', self.h20_hpa))
		buf.extend(struct.pack('<f', self.co2_int_temp))
		buf.extend(struct.pack('<f', self.air_temp))
		buf.extend(struct.pack('<f', self.logger_temp))
		buf.extend(struct.pack('<f', self.pressure))
		return bytearray(buf)

class ParticlesPlusChannel:
	SIZE = 11

	def __init__ (self, channel_size = 0, differential_counts = 0,
	differential_counts_m = 0.0, differential_mass = 0.0):
		self.channel_size = channel_size
		self.differential_counts = differential_counts
		self.differential_counts_m = differential_counts_m
		self.differential_mass = differential_mass

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [ParticlesPlusChannel]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.channel_size = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.differential_counts = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

		self.differential_counts_m = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.differential_mass = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<B', self.channel_size))
		buf.extend(struct.pack('<H', self.differential_counts))
		buf.extend(struct.pack('<f', self.differential_counts_m))
		buf.extend(struct.pack('<f', self.differential_mass))
		return bytearray(buf)

class LDCR:
	SIZE = 101

	def __init__ (self, header = [None] * 2, serial_number = 0, hw_revision = 0,
	sw_revision = 0, platform_type = LDCRPlatformType(0), platform_serial = 0,
	system_time = 0, calibration_state = 0, sum_data = [None] * 2,
	sum_of_squares = [None] * 2, thermistor = [None] * 8, thermistor_ref = 0,
	week = 0, hour = 0, minute = 0, seconds = 0.0, latitude = 0.0,
	longitude = 0.0, altitude = 0.0, agl = 0.0, roll = 0.0, pitch = 0.0,
	crc = 0):
		if (len(header) != 2):
			raise ValueError('array header expecting length '+str(2)+' got '+str(len(header)))

		self.header = list(header)

		self.serial_number = serial_number
		self.hw_revision = hw_revision
		self.sw_revision = sw_revision

		self.platform_type = LDCRPlatformType(platform_type)

		self.platform_serial = platform_serial
		self.system_time = system_time
		self.calibration_state = calibration_state

		if (len(sum_data) != 2):
			raise ValueError('array sum_data expecting length '+str(2)+' got '+str(len(sum_data)))

		self.sum_data = list(sum_data)

		if (len(sum_of_squares) != 2):
			raise ValueError('array sum_of_squares expecting length '+str(2)+' got '+str(len(sum_of_squares)))

		self.sum_of_squares = list(sum_of_squares)

		if (len(thermistor) != 8):
			raise ValueError('array thermistor expecting length '+str(8)+' got '+str(len(thermistor)))

		self.thermistor = list(thermistor)

		self.thermistor_ref = thermistor_ref
		self.week = week
		self.hour = hour
		self.minute = minute
		self.seconds = seconds
		self.latitude = latitude
		self.longitude = longitude
		self.altitude = altitude
		self.agl = agl
		self.roll = roll
		self.pitch = pitch
		self.crc = crc

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [LDCR]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.header = [];

		for i in range(0,2):
			self.header.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

		self.serial_number = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

		self.hw_revision = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.sw_revision = struct.unpack_from('<I',buf,offset)[0]
		offset = offset + struct.calcsize('<I')

		self.platform_type = LDCRPlatformType(struct.unpack_from('<B',buf,offset)[0])
		offset = offset+struct.calcsize('<B')

		self.platform_serial = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

		self.system_time = struct.unpack_from('<I',buf,offset)[0]
		offset = offset + struct.calcsize('<I')

		self.calibration_state = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.sum_data = [];

		for i in range(0,2):
			self.sum_data.append(struct.unpack_from('<i',buf,offset)[0])
			offset = offset+struct.calcsize('<i')

		self.sum_of_squares = [];

		for i in range(0,2):
			self.sum_of_squares.append(struct.unpack_from('<Q',buf,offset)[0])
			offset = offset+struct.calcsize('<Q')

		self.thermistor = [];

		for i in range(0,8):
			self.thermistor.append(struct.unpack_from('<H',buf,offset)[0])
			offset = offset+struct.calcsize('<H')

		self.thermistor_ref = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

		self.week = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

		self.hour = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.minute = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.seconds = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.latitude = struct.unpack_from('<d',buf,offset)[0]
		offset = offset + struct.calcsize('<d')

		self.longitude = struct.unpack_from('<d',buf,offset)[0]
		offset = offset + struct.calcsize('<d')

		self.altitude = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.agl = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.roll = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.pitch = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.crc = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		for val in self.header:
		    buf.extend(struct.pack('<B', val))

		buf.extend(struct.pack('<H', self.serial_number))
		buf.extend(struct.pack('<B', self.hw_revision))
		buf.extend(struct.pack('<I', self.sw_revision))

		buf.put(LDCRPlatformType.encode(self.platform_type));

		buf.extend(struct.pack('<H', self.platform_serial))
		buf.extend(struct.pack('<I', self.system_time))
		buf.extend(struct.pack('<B', self.calibration_state))

		for val in self.sum_data:
		    buf.extend(struct.pack('<i', val))

		for val in self.sum_of_squares:
		    buf.extend(struct.pack('<Q', val))

		for val in self.thermistor:
		    buf.extend(struct.pack('<H', val))

		buf.extend(struct.pack('<H', self.thermistor_ref))
		buf.extend(struct.pack('<H', self.week))
		buf.extend(struct.pack('<B', self.hour))
		buf.extend(struct.pack('<B', self.minute))
		buf.extend(struct.pack('<f', self.seconds))
		buf.extend(struct.pack('<d', self.latitude))
		buf.extend(struct.pack('<d', self.longitude))
		buf.extend(struct.pack('<f', self.altitude))
		buf.extend(struct.pack('<f', self.agl))
		buf.extend(struct.pack('<f', self.roll))
		buf.extend(struct.pack('<f', self.pitch))
		buf.extend(struct.pack('<H', self.crc))
		return bytearray(buf)

class ParticlesPlus:
	SIZE = 104

	def __init__ (self, system_time = 0.0, date = [None] * 11,
	time = [None] * 9, duration = 0.0, sample_flow_rate = 0.0,
	sample_status_bits = 0, bp = 0.0, channel_data = [None] * 6):
		self.system_time = system_time

		if (len(date) != 11):
			raise ValueError('array date expecting length '+str(11)+' got '+str(len(date)))

		self.date = list(date)

		if (len(time) != 9):
			raise ValueError('array time expecting length '+str(9)+' got '+str(len(time)))

		self.time = list(time)

		self.duration = duration
		self.sample_flow_rate = sample_flow_rate
		self.sample_status_bits = sample_status_bits
		self.bp = bp

		if (len(channel_data) != 6):
			raise ValueError('array channel_data expecting length '+str(6)+' got '+str(len(channel_data)))

		self.channel_data = list(channel_data)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [ParticlesPlus]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.system_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.date = [];

		for i in range(0,11):
			self.date.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

		self.time = [];

		for i in range(0,9):
			self.time.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

		self.duration = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.sample_flow_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.sample_status_bits = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

		self.bp = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.channel_data = [];

		for i in range(0,6):
			self.channel_data.append(struct.unpack_from('<ParticlesPlusChannel',buf,offset)[0])
			offset = offset+struct.calcsize('<ParticlesPlusChannel')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.system_time))

		for val in self.date:
		    buf.extend(struct.pack('<B', val))

		for val in self.time:
		    buf.extend(struct.pack('<B', val))

		buf.extend(struct.pack('<f', self.duration))
		buf.extend(struct.pack('<f', self.sample_flow_rate))
		buf.extend(struct.pack('<H', self.sample_status_bits))
		buf.extend(struct.pack('<f', self.bp))

		for val in self.channel_data:
		    buf.extend(val.serialize())
		return bytearray(buf)

#---------[ Communication ]---------#

class TelemetryPayload:
	SIZE = 7

	def __init__ (self, node_status = PayloadControl(0), num_triggers = 0,
	percent_complete = 0.0):
		self.node_status = PayloadControl(node_status)

		self.num_triggers = num_triggers
		self.percent_complete = percent_complete

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [TelemetryPayload]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.node_status = PayloadControl(struct.unpack_from('<B',buf,offset)[0])
		offset = offset+struct.calcsize('<B')

		self.num_triggers = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

		self.percent_complete = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.put(PayloadControl.encode(self.node_status));

		buf.extend(struct.pack('<H', self.num_triggers))
		buf.extend(struct.pack('<f', self.percent_complete))
		return bytearray(buf)

#---------[ Payload ]---------#

class PayloadID (Enum):
	PAYLOAD_UNKNOWN=0
	PAYLOAD_QX1=1
	PAYLOAD_A6000=2
	PAYLOAD_FLIR_TAU2=3
	PAYLOAD_TETRACAM_ADC_LITE=4
	PAYLOAD_A5100=5
	PAYLOAD_MAPIR_KERNEL=6
	PAYLOAD_FLIR_VUE_PRO=7
	PAYLOAD_MICASENSE_REDEDGE=8
	PAYLOAD_PARTICLES_PLUS=9
	PAYLOAD_K30=10
	PAYLOAD_MINIGAS=11

class NDVI:
	SIZE = 21

	def __init__ (self, system_time = 0.0, id = 0, red = 0.0, near_ir = 0.0,
	ir_ambient = 0.0, ir_object = 0.0):
		self.system_time = system_time
		self.id = id
		self.red = red
		self.near_ir = near_ir
		self.ir_ambient = ir_ambient
		self.ir_object = ir_object

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [NDVI]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.system_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.id = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.red = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.near_ir = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.ir_ambient = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.ir_object = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.system_time))
		buf.extend(struct.pack('<B', self.id))
		buf.extend(struct.pack('<f', self.red))
		buf.extend(struct.pack('<f', self.near_ir))
		buf.extend(struct.pack('<f', self.ir_ambient))
		buf.extend(struct.pack('<f', self.ir_object))
		return bytearray(buf)

class PayloadParam:
	SIZE = 17

	def __init__ (self, channel = 0, deltaD = 0.0, deltaT = 0.0, pulse = 0.0,
	cruise_speed = 0.0):
		self.channel = channel
		self.deltaD = deltaD
		self.deltaT = deltaT
		self.pulse = pulse
		self.cruise_speed = cruise_speed

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [PayloadParam]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.channel = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.deltaD = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.deltaT = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.pulse = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.cruise_speed = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<B', self.channel))
		buf.extend(struct.pack('<f', self.deltaD))
		buf.extend(struct.pack('<f', self.deltaT))
		buf.extend(struct.pack('<f', self.pulse))
		buf.extend(struct.pack('<f', self.cruise_speed))
		return bytearray(buf)

class PayloadTrigger:
	SIZE = 39

	def __init__ (self, latitude = 0.0, longitude = 0.0, altitude = 0.0,
	q = [None] * 4, percent = 0, id = 0):
		self.latitude = latitude
		self.longitude = longitude
		self.altitude = altitude

		if (len(q) != 4):
			raise ValueError('array q expecting length '+str(4)+' got '+str(len(q)))

		self.q = list(q)

		self.percent = percent
		self.id = id

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [PayloadTrigger]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.latitude = struct.unpack_from('<d',buf,offset)[0]
		offset = offset + struct.calcsize('<d')

		self.longitude = struct.unpack_from('<d',buf,offset)[0]
		offset = offset + struct.calcsize('<d')

		self.altitude = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.q = [];

		for i in range(0,4):
			self.q.append(struct.unpack_from('<f',buf,offset)[0])
			offset = offset+struct.calcsize('<f')

		self.percent = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.id = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<d', self.latitude))
		buf.extend(struct.pack('<d', self.longitude))
		buf.extend(struct.pack('<f', self.altitude))

		for val in self.q:
		    buf.extend(struct.pack('<f', val))

		buf.extend(struct.pack('<B', self.percent))
		buf.extend(struct.pack('<H', self.id))
		return bytearray(buf)

class UserPayload:
	SIZE = 66

	def __init__ (self, system_id = 0, size = 0, buffer = [None] * 64):
		self.system_id = system_id
		self.size = size

		if (len(buffer) != 64):
			raise ValueError('array buffer expecting length '+str(64)+' got '+str(len(buffer)))

		self.buffer = list(buffer)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [UserPayload]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.system_id = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.size = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.buffer = [];

		for i in range(0,64):
			self.buffer.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<B', self.system_id))
		buf.extend(struct.pack('<B', self.size))

		for val in self.buffer:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

class CameraTag:
	SIZE = 79

	def __init__ (self, trigger_info = 0, week = 0, hour = 0, minute = 0,
	seconds = 0.0, filename = [None] * 32):
		self.trigger_info = PayloadTrigger(trigger_info)

		self.week = week
		self.hour = hour
		self.minute = minute
		self.seconds = seconds

		if (len(filename) != 32):
			raise ValueError('array filename expecting length '+str(32)+' got '+str(len(filename)))

		self.filename = list(filename)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [CameraTag]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.trigger_info = PayloadTrigger()
		self.trigger_info.parse(buf[offset:offset+PayloadTrigger.SIZE])
		offset = offset+PayloadTrigger.SIZE

		self.week = struct.unpack_from('<H',buf,offset)[0]
		offset = offset + struct.calcsize('<H')

		self.hour = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.minute = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.seconds = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.filename = [];

		for i in range(0,32):
			self.filename.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.extend(self.trigger_info.serialize())
		buf.extend(struct.pack('<H', self.week))
		buf.extend(struct.pack('<B', self.hour))
		buf.extend(struct.pack('<B', self.minute))
		buf.extend(struct.pack('<f', self.seconds))

		for val in self.filename:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

class PayloadStatus:
	SIZE = 4

	def __init__ (self, identifier = PayloadID.PAYLOAD_UNKNOWN, power_on = 0,
	initialized = 0, state = PayloadControl(0)):
		self.identifier = PayloadID(identifier)

		self.power_on = power_on
		self.initialized = initialized

		self.state = PayloadControl(state)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [PayloadStatus]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.identifier = PayloadID(struct.unpack_from('<B',buf,offset)[0])
		offset = offset+struct.calcsize('<B')

		self.power_on = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.initialized = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.state = PayloadControl(struct.unpack_from('<B',buf,offset)[0])
		offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def serialize(self):
		buf = []

		buf.put(PayloadID.encode(self.identifier));

		buf.extend(struct.pack('<B', self.power_on))
		buf.extend(struct.pack('<B', self.initialized))

		buf.put(PayloadControl.encode(self.state));
		return bytearray(buf)
