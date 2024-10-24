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

#---------[ Actuators ]---------#

MAX_NUM_ROTORS = 16

#---------[ Controller ]---------#

class CommandID (Enum):
	# NOTE - you must check the numbers in the CommandID values

	# contained in the parent folder

	#


	# Composite

	CMD_VEL_CTRL=26

	# Position

	CMD_X_POS=27
	CMD_Y_POS=28
	CMD_X_VEL=29
	CMD_Y_VEL=30

	# Thrust/Moment Commands

	CMD_THRUST=31
	CMD_MOMENT_X=32
	CMD_MOMENT_Y=33
	CMD_MOMENT_Z=34

	# Speed

	CMD_SOG=25

class ControlLoop (Enum):
	CTRL_ANG_TO_RATE=0  # Angle error to rate

	CTRL_ROLL_RATE=1  # PID on roll rate

	CTRL_PITCH_RATE=2  # PID on pitch rate

	CTRL_YAW_RATE=3  # PID on yaw rate

	CTRL_X_VEL_TO_ACC=4  # x velocity error to acceleration

	CTRL_Y_VEL_TO_ACC=5  # y velocity error to acceleration

	CTRL_POS=6  # PID on horizontal speed

	CTRL_ALT=7  # PID on vrate

	CTRL_THRUST=8  # PID on thrust

	CTRL_INVALID=9

class FlightControlParameters:
	SIZE = 48

	def __init__ (self, min_ground_speed = 0.0, nav_lookahead = 0.0,
	min_nav_lookahead_dist = 0.0, wpt_capture_dist = 0.0, unused = [None] * 32):
		self.min_ground_speed = min_ground_speed
		self.nav_lookahead = nav_lookahead
		self.min_nav_lookahead_dist = min_nav_lookahead_dist
		self.wpt_capture_dist = wpt_capture_dist

		if (len(unused) != 32):
			raise ValueError('array unused expecting length '+str(32)+' got '+str(len(unused)))

		self.unused = list(unused)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [FlightControlParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.min_ground_speed = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.nav_lookahead = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.min_nav_lookahead_dist = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.wpt_capture_dist = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.unused = [];

		for i in range(0,32):
			self.unused.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.min_ground_speed))
		buf.extend(struct.pack('<f', self.nav_lookahead))
		buf.extend(struct.pack('<f', self.min_nav_lookahead_dist))
		buf.extend(struct.pack('<f', self.wpt_capture_dist))

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

class VehicleLimits:
	SIZE = 69

	def __init__ (self, roll = 0, pitch = 0, roll_rate = 0.0,
	pitch_rate = 0.0, yaw_rate = 0.0, speed = 0.0, vrate = 0, lost_gps = 0,
	max_pdop = 0.0, unused = [None] * 24):
		self.roll = Limit(roll)

		self.pitch = Limit(pitch)

		self.roll_rate = roll_rate
		self.pitch_rate = pitch_rate
		self.yaw_rate = yaw_rate
		self.speed = speed

		self.vrate = Limit(vrate)

		self.lost_gps = lost_gps
		self.max_pdop = max_pdop

		if (len(unused) != 24):
			raise ValueError('array unused expecting length '+str(24)+' got '+str(len(unused)))

		self.unused = list(unused)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [VehicleLimits]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.roll = Limit()
		self.roll.parse(buf[offset:offset+Limit.SIZE])
		offset = offset+Limit.SIZE

		self.pitch = Limit()
		self.pitch.parse(buf[offset:offset+Limit.SIZE])
		offset = offset+Limit.SIZE

		self.roll_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.pitch_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.yaw_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.speed = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.vrate = Limit()
		self.vrate.parse(buf[offset:offset+Limit.SIZE])
		offset = offset+Limit.SIZE

		self.lost_gps = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.max_pdop = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.unused = [];

		for i in range(0,24):
			self.unused.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(self.roll.serialize())
		buf.extend(self.pitch.serialize())
		buf.extend(struct.pack('<f', self.roll_rate))
		buf.extend(struct.pack('<f', self.pitch_rate))
		buf.extend(struct.pack('<f', self.yaw_rate))
		buf.extend(struct.pack('<f', self.speed))
		buf.extend(self.vrate.serialize())
		buf.extend(struct.pack('<B', self.lost_gps))
		buf.extend(struct.pack('<f', self.max_pdop))

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

#---------[ Logging ]---------#

class LogFlightControl:
	SIZE = 64

	def __init__ (self, data = [None] * 64):
		if (len(data) != 64):
			raise ValueError('array data expecting length '+str(64)+' got '+str(len(data)))

		self.data = list(data)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [LogFlightControl]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.data = [];

		for i in range(0,64):
			self.data.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		for val in self.data:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

#---------[ Mission ]---------#

class MissionParameters:
	SIZE = 48

	def __init__ (self, altitude = 0, comm = 0, max_range = 0.0,
	safe_height = 0.0, flight_time = 0.0, battery_min = 0.0, initialized = 0,
	mag_dec = 0.0, unused = [None] * 16):
		self.altitude = Limit(altitude)

		self.comm = Timeout(comm)

		self.max_range = max_range
		self.safe_height = safe_height
		self.flight_time = flight_time
		self.battery_min = battery_min
		self.initialized = initialized
		self.mag_dec = mag_dec

		if (len(unused) != 16):
			raise ValueError('array unused expecting length '+str(16)+' got '+str(len(unused)))

		self.unused = list(unused)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [MissionParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.altitude = Limit()
		self.altitude.parse(buf[offset:offset+Limit.SIZE])
		offset = offset+Limit.SIZE

		self.comm = Timeout()
		self.comm.parse(buf[offset:offset+Timeout.SIZE])
		offset = offset+Timeout.SIZE

		self.max_range = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.safe_height = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.flight_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.battery_min = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.initialized = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.mag_dec = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.unused = [];

		for i in range(0,16):
			self.unused.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(self.altitude.serialize())
		buf.extend(self.comm.serialize())
		buf.extend(struct.pack('<f', self.max_range))
		buf.extend(struct.pack('<f', self.safe_height))
		buf.extend(struct.pack('<f', self.flight_time))
		buf.extend(struct.pack('<f', self.battery_min))
		buf.extend(struct.pack('<B', self.initialized))
		buf.extend(struct.pack('<f', self.mag_dec))

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

#---------[ Rotors ]---------#

class RotorDir (Enum):
	ROTOR_DIR_CW=0
	ROTOR_DIR_CCW=1
	ROTOR_DIR_INVALID=2

class RotorParameters:
	SIZE = 43

	def __init__ (self, id = 0, channel = 0, k_wv = 0.0, pwm_o = 0.0,
	pos_x = 0.0, pos_y = 0.0, pos_z = 0.0, t_x = 0.0, t_y = 0.0, t_z = 0.0,
	dir = RotorDir.ROTOR_DIR_INVALID, rpm_to_thrust = 0.0,
	thrust_to_moment = 0.0):
		self.id = id
		self.channel = channel
		self.k_wv = k_wv
		self.pwm_o = pwm_o
		self.pos_x = pos_x
		self.pos_y = pos_y
		self.pos_z = pos_z
		self.t_x = t_x
		self.t_y = t_y
		self.t_z = t_z

		self.dir = RotorDir(dir)

		self.rpm_to_thrust = rpm_to_thrust
		self.thrust_to_moment = thrust_to_moment

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [RotorParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.id = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.channel = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.k_wv = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.pwm_o = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.pos_x = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.pos_y = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.pos_z = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.t_x = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.t_y = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.t_z = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.dir = RotorDir(struct.unpack_from('<B',buf,offset)[0])
		offset = offset+struct.calcsize('<B')

		self.rpm_to_thrust = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.thrust_to_moment = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<B', self.id))
		buf.extend(struct.pack('<B', self.channel))
		buf.extend(struct.pack('<f', self.k_wv))
		buf.extend(struct.pack('<f', self.pwm_o))
		buf.extend(struct.pack('<f', self.pos_x))
		buf.extend(struct.pack('<f', self.pos_y))
		buf.extend(struct.pack('<f', self.pos_z))
		buf.extend(struct.pack('<f', self.t_x))
		buf.extend(struct.pack('<f', self.t_y))
		buf.extend(struct.pack('<f', self.t_z))

		buf.put(RotorDir.encode(self.dir));

		buf.extend(struct.pack('<f', self.rpm_to_thrust))
		buf.extend(struct.pack('<f', self.thrust_to_moment))
		return bytearray(buf)

#---------[ System ]---------#

class VehicleInitialization (Enum):
	ORIENTATION_INITIALIZED=1
	ACTUATORS_INITIALIZED=2
	PARAM_INITIALIZED=4
	VECHILE_LIMITS_INITIALIZED=8
	CONTROL_GAINS_INITIALIZED=16
	LAUNCH_PARAM_INITIALIZED=32
	LANDING_PARAM_INITIALIZED=64
	ROTORS_INITIALIZED=128
	ESTIMATOR_INITIALIZED=256
	FILTERS_INITIALIZED=512
	SENSORS_INITIALIZED=1024

#---------[ Vehicle ]---------#

class LandingStatus (Enum):
	LANDING_INVALID=0
	LANDING_STATUS_ENTER=1
	LANDING_STATUS_TRACKING=2
	LANDING_STATUS_HOLDING=3
	LANDING_STATUS_FINAL=4
	LANDING_STATUS_MANUAL=5
	LANDING_STATUS_COMMITTED=6

class LandingParameters:
	SIZE = 28

	def __init__ (self, safe_height = 0.0, descend_rate = 0.0,
	agl_offset = 0.0, unused = [None] * 16):
		self.safe_height = safe_height
		self.descend_rate = descend_rate
		self.agl_offset = agl_offset

		if (len(unused) != 16):
			raise ValueError('array unused expecting length '+str(16)+' got '+str(len(unused)))

		self.unused = list(unused)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [LandingParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.safe_height = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.descend_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.agl_offset = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.unused = [];

		for i in range(0,16):
			self.unused.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.safe_height))
		buf.extend(struct.pack('<f', self.descend_rate))
		buf.extend(struct.pack('<f', self.agl_offset))

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

class LaunchParameters:
	SIZE = 24

	def __init__ (self, climbout_height = 0.0, timeout = 0.0,
	unused = [None] * 16):
		self.climbout_height = climbout_height
		self.timeout = timeout

		if (len(unused) != 16):
			raise ValueError('array unused expecting length '+str(16)+' got '+str(len(unused)))

		self.unused = list(unused)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [LaunchParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.climbout_height = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.timeout = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.unused = [];

		for i in range(0,16):
			self.unused.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.climbout_height))
		buf.extend(struct.pack('<f', self.timeout))

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

class VehicleParameters:
	SIZE = 51

	def __init__ (self, name = [None] * 16, battery_cap = 0.0,
	battery_num_cells = 0, batt_chem = BatteryChemistry(0), num_rotors = 0,
	mass = 0.0, torque = 0.0, drag = 0.0, cruise_speed = 0.0,
	unused = [None] * 12):
		if (len(name) != 16):
			raise ValueError('array name expecting length '+str(16)+' got '+str(len(name)))

		self.name = list(name)

		self.battery_cap = battery_cap
		self.battery_num_cells = battery_num_cells

		self.batt_chem = BatteryChemistry(batt_chem)

		self.num_rotors = num_rotors
		self.mass = mass
		self.torque = torque
		self.drag = drag
		self.cruise_speed = cruise_speed

		if (len(unused) != 12):
			raise ValueError('array unused expecting length '+str(12)+' got '+str(len(unused)))

		self.unused = list(unused)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [VehicleParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.name = [];

		for i in range(0,16):
			self.name.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

		self.battery_cap = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.battery_num_cells = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.batt_chem = BatteryChemistry(struct.unpack_from('<B',buf,offset)[0])
		offset = offset+struct.calcsize('<B')

		self.num_rotors = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.mass = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.torque = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.drag = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.cruise_speed = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.unused = [];

		for i in range(0,12):
			self.unused.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		for val in self.name:
		    buf.extend(struct.pack('<B', val))

		buf.extend(struct.pack('<f', self.battery_cap))
		buf.extend(struct.pack('<B', self.battery_num_cells))

		buf.put(BatteryChemistry.encode(self.batt_chem));

		buf.extend(struct.pack('<B', self.num_rotors))
		buf.extend(struct.pack('<f', self.mass))
		buf.extend(struct.pack('<f', self.torque))
		buf.extend(struct.pack('<f', self.drag))
		buf.extend(struct.pack('<f', self.cruise_speed))

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)
