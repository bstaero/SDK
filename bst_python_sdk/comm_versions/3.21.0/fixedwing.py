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

	# Controller Modes

	CMD_TECS_MODE=24

	# Velocity

	CMD_IAS=25

	# System Commands

	CMD_CAPTURE_TRIMS=26

class ControlLoop (Enum):
	CTRL_TURNRATE_2_RUD=0
	CTRL_PITCH_2_ELEVATOR=1
	CTRL_ROLL_2_AIL=2

	CTRL_NAV_2_ROLL=3  # no I or D terms

	CTRL_ENG_2_THROTTLE=4  # D is a FF term

	CTRL_ENG_2_PITCH=5  # D is a FF term

	CTRL_INVALID=6

class TECSMode (Enum):
	TECS_MODE_OFF=0
	TECS_MODE_CLIMB=1
	TECS_MODE_ALT_HOLD=2
	TECS_MODE_VRATE=3
	TECS_MODE_LAND=4
	TECS_MODE_FLARE=5

class FilterParameters:
	SIZE = 40

	def __init__ (self, ias_alpha = 0.0, ias_dot_alpha = 0.0,
	gamma_alpha = 0.0, roll_cmd_rate = 0.0, pitch_cmd_rate = 0.0,
	ias_cmd_rate = 0.0, gamma_cmd_rate = 0.0, throttle_cmd_rate = 0.0,
	k_cmd_rate = 0.0, vx_dot_cmd_rate = 0.0):
		self.ias_alpha = ias_alpha
		self.ias_dot_alpha = ias_dot_alpha
		self.gamma_alpha = gamma_alpha
		self.roll_cmd_rate = roll_cmd_rate
		self.pitch_cmd_rate = pitch_cmd_rate
		self.ias_cmd_rate = ias_cmd_rate
		self.gamma_cmd_rate = gamma_cmd_rate
		self.throttle_cmd_rate = throttle_cmd_rate
		self.k_cmd_rate = k_cmd_rate
		self.vx_dot_cmd_rate = vx_dot_cmd_rate

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [FilterParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.ias_alpha = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.ias_dot_alpha = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.gamma_alpha = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.roll_cmd_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.pitch_cmd_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.ias_cmd_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.gamma_cmd_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.throttle_cmd_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.k_cmd_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.vx_dot_cmd_rate = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.ias_alpha))
		buf.extend(struct.pack('<f', self.ias_dot_alpha))
		buf.extend(struct.pack('<f', self.gamma_alpha))
		buf.extend(struct.pack('<f', self.roll_cmd_rate))
		buf.extend(struct.pack('<f', self.pitch_cmd_rate))
		buf.extend(struct.pack('<f', self.ias_cmd_rate))
		buf.extend(struct.pack('<f', self.gamma_cmd_rate))
		buf.extend(struct.pack('<f', self.throttle_cmd_rate))
		buf.extend(struct.pack('<f', self.k_cmd_rate))
		buf.extend(struct.pack('<f', self.vx_dot_cmd_rate))
		return bytearray(buf)

class FlightControlParameters:
	SIZE = 68

	def __init__ (self, tecs_Kv = 0.0, tecs_Kh = 0.0,
	tecs_max_vx_dot = 0.0, min_ground_speed = 0.0, nav_lookahead = 0.0,
	min_nav_lookahead_dist = 0.0, tuning_ias = 0.0, max_height_error_mode = 0.0,
	max_v_error_mode = 0.0, k_height_tracking = 0.0, k_flare = 0.0,
	k_land = 0.0, k_cruise = 0.0, k_climb = 0.0, k_speed_hold = 0.0,
	no_ias_a = 0.0, no_ias_b = 0.0):
		self.tecs_Kv = tecs_Kv
		self.tecs_Kh = tecs_Kh
		self.tecs_max_vx_dot = tecs_max_vx_dot
		self.min_ground_speed = min_ground_speed
		self.nav_lookahead = nav_lookahead
		self.min_nav_lookahead_dist = min_nav_lookahead_dist
		self.tuning_ias = tuning_ias
		self.max_height_error_mode = max_height_error_mode
		self.max_v_error_mode = max_v_error_mode
		self.k_height_tracking = k_height_tracking
		self.k_flare = k_flare
		self.k_land = k_land
		self.k_cruise = k_cruise
		self.k_climb = k_climb
		self.k_speed_hold = k_speed_hold
		self.no_ias_a = no_ias_a
		self.no_ias_b = no_ias_b

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [FlightControlParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.tecs_Kv = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.tecs_Kh = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.tecs_max_vx_dot = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.min_ground_speed = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.nav_lookahead = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.min_nav_lookahead_dist = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.tuning_ias = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.max_height_error_mode = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.max_v_error_mode = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.k_height_tracking = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.k_flare = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.k_land = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.k_cruise = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.k_climb = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.k_speed_hold = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.no_ias_a = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.no_ias_b = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.tecs_Kv))
		buf.extend(struct.pack('<f', self.tecs_Kh))
		buf.extend(struct.pack('<f', self.tecs_max_vx_dot))
		buf.extend(struct.pack('<f', self.min_ground_speed))
		buf.extend(struct.pack('<f', self.nav_lookahead))
		buf.extend(struct.pack('<f', self.min_nav_lookahead_dist))
		buf.extend(struct.pack('<f', self.tuning_ias))
		buf.extend(struct.pack('<f', self.max_height_error_mode))
		buf.extend(struct.pack('<f', self.max_v_error_mode))
		buf.extend(struct.pack('<f', self.k_height_tracking))
		buf.extend(struct.pack('<f', self.k_flare))
		buf.extend(struct.pack('<f', self.k_land))
		buf.extend(struct.pack('<f', self.k_cruise))
		buf.extend(struct.pack('<f', self.k_climb))
		buf.extend(struct.pack('<f', self.k_speed_hold))
		buf.extend(struct.pack('<f', self.no_ias_a))
		buf.extend(struct.pack('<f', self.no_ias_b))
		return bytearray(buf)

class SurfaceMixing:
	SIZE = 12

	def __init__ (self, mixing_roll_2_elevator = 0.0,
	mixing_aileron_2_rudder = 0.0, mixing_flap_2_elevator = 0.0):
		self.mixing_roll_2_elevator = mixing_roll_2_elevator
		self.mixing_aileron_2_rudder = mixing_aileron_2_rudder
		self.mixing_flap_2_elevator = mixing_flap_2_elevator

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [SurfaceMixing]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.mixing_roll_2_elevator = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.mixing_aileron_2_rudder = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.mixing_flap_2_elevator = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.mixing_roll_2_elevator))
		buf.extend(struct.pack('<f', self.mixing_aileron_2_rudder))
		buf.extend(struct.pack('<f', self.mixing_flap_2_elevator))
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
	SENSORS_INITIALIZED=128

#---------[ Vehicle ]---------#

class LandType (Enum):
	LAND_SPIRAL=0

class LandingStatus (Enum):
	LANDING_INVALID=0
	LANDING_STATUS_ENTER=1
	LANDING_STATUS_TRACKING=2
	LANDING_STATUS_HOLDING=3
	LANDING_STATUS_FINAL=4
	LANDING_STATUS_SHORT=5
	LANDING_STATUS_LONG=6
	LANDING_STATUS_LATERAL=7
	LANDING_STATUS_MANUAL=8
	LANDING_STATUS_COMMITTED=9

class LaunchType (Enum):
	LAUNCH_HAND=0
	LAUNCH_BUNGEE=1
	LAUNCH_WINCH=2
	LAUNCH_ROLLING=3
	LAUNCH_CAR=4
	LAUNCH_RAIL=5
	LAUNCH_DROP=6

class LandingParameters:
	SIZE = 68

	def __init__ (self, ias = 0.0, glide_slope = 0.0, safe_height = 0.0,
	abort_height = 0.0, flap_deflection = 0.0, flare_min_pitch = 0.0,
	cross_track_error = 0.0, cross_track_angle = 0.0, height_error_bound = 0.0,
	abort_trigger_time = 0.0, decision_time = 0.0, commit_time = 0.0,
	flare_time = 0.0, agl_offset = 0.0, unused = [None] * 12):
		self.ias = ias
		self.glide_slope = glide_slope
		self.safe_height = safe_height
		self.abort_height = abort_height
		self.flap_deflection = flap_deflection
		self.flare_min_pitch = flare_min_pitch
		self.cross_track_error = cross_track_error
		self.cross_track_angle = cross_track_angle
		self.height_error_bound = height_error_bound
		self.abort_trigger_time = abort_trigger_time
		self.decision_time = decision_time
		self.commit_time = commit_time
		self.flare_time = flare_time
		self.agl_offset = agl_offset

		if (len(unused) != 12):
			raise ValueError('array unused expecting length '+str(12)+' got '+str(len(unused)))

		self.unused = list(unused)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [LandingParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.ias = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.glide_slope = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.safe_height = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.abort_height = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.flap_deflection = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.flare_min_pitch = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.cross_track_error = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.cross_track_angle = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.height_error_bound = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.abort_trigger_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.decision_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.commit_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.flare_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.agl_offset = struct.unpack_from('<f',buf,offset)[0]
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

		buf.extend(struct.pack('<f', self.ias))
		buf.extend(struct.pack('<f', self.glide_slope))
		buf.extend(struct.pack('<f', self.safe_height))
		buf.extend(struct.pack('<f', self.abort_height))
		buf.extend(struct.pack('<f', self.flap_deflection))
		buf.extend(struct.pack('<f', self.flare_min_pitch))
		buf.extend(struct.pack('<f', self.cross_track_error))
		buf.extend(struct.pack('<f', self.cross_track_angle))
		buf.extend(struct.pack('<f', self.height_error_bound))
		buf.extend(struct.pack('<f', self.abort_trigger_time))
		buf.extend(struct.pack('<f', self.decision_time))
		buf.extend(struct.pack('<f', self.commit_time))
		buf.extend(struct.pack('<f', self.flare_time))
		buf.extend(struct.pack('<f', self.agl_offset))

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

class LaunchParameters:
	SIZE = 64

	def __init__ (self, ias = 0.0, flap_deflection = 0.0,
	throttle_delay = 0.0, throttle_setting = 0.0, min_pitch = 0.0,
	climbout_angle = 0.0, climbout_height = 0.0, timeout = 0.0,
	elevator_deflection = 0.0, unused = [None] * 28):
		self.ias = ias
		self.flap_deflection = flap_deflection
		self.throttle_delay = throttle_delay
		self.throttle_setting = throttle_setting
		self.min_pitch = min_pitch
		self.climbout_angle = climbout_angle
		self.climbout_height = climbout_height
		self.timeout = timeout
		self.elevator_deflection = elevator_deflection

		if (len(unused) != 28):
			raise ValueError('array unused expecting length '+str(28)+' got '+str(len(unused)))

		self.unused = list(unused)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [LaunchParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.ias = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.flap_deflection = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.throttle_delay = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.throttle_setting = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.min_pitch = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.climbout_angle = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.climbout_height = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.timeout = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.elevator_deflection = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.unused = [];

		for i in range(0,28):
			self.unused.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(struct.pack('<f', self.ias))
		buf.extend(struct.pack('<f', self.flap_deflection))
		buf.extend(struct.pack('<f', self.throttle_delay))
		buf.extend(struct.pack('<f', self.throttle_setting))
		buf.extend(struct.pack('<f', self.min_pitch))
		buf.extend(struct.pack('<f', self.climbout_angle))
		buf.extend(struct.pack('<f', self.climbout_height))
		buf.extend(struct.pack('<f', self.timeout))
		buf.extend(struct.pack('<f', self.elevator_deflection))

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

class VehicleLimits:
	SIZE = 67

	def __init__ (self, roll_angle = 0, pitch_angle = 0, ias = 0,
	lost_gps = 0, lost_gps_roll = 0.0, max_pdop = 0.0, max_scale_factor = 0.0,
	flightpath_angle = 0, flightpath_angle_flap = 0,
	flightpath_angle_fraction = 0.0, unused = [None] * 10):
		self.roll_angle = Limit(roll_angle)

		self.pitch_angle = Limit(pitch_angle)

		self.ias = Limit(ias)

		self.lost_gps = lost_gps
		self.lost_gps_roll = lost_gps_roll
		self.max_pdop = max_pdop
		self.max_scale_factor = max_scale_factor

		self.flightpath_angle = Limit(flightpath_angle)

		self.flightpath_angle_flap = Limit(flightpath_angle_flap)

		self.flightpath_angle_fraction = flightpath_angle_fraction

		if (len(unused) != 10):
			raise ValueError('array unused expecting length '+str(10)+' got '+str(len(unused)))

		self.unused = list(unused)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [VehicleLimits]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.roll_angle = Limit()
		self.roll_angle.parse(buf[offset:offset+Limit.SIZE])
		offset = offset+Limit.SIZE

		self.pitch_angle = Limit()
		self.pitch_angle.parse(buf[offset:offset+Limit.SIZE])
		offset = offset+Limit.SIZE

		self.ias = Limit()
		self.ias.parse(buf[offset:offset+Limit.SIZE])
		offset = offset+Limit.SIZE

		self.lost_gps = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.lost_gps_roll = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.max_pdop = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.max_scale_factor = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.flightpath_angle = Limit()
		self.flightpath_angle.parse(buf[offset:offset+Limit.SIZE])
		offset = offset+Limit.SIZE

		self.flightpath_angle_flap = Limit()
		self.flightpath_angle_flap.parse(buf[offset:offset+Limit.SIZE])
		offset = offset+Limit.SIZE

		self.flightpath_angle_fraction = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.unused = [];

		for i in range(0,10):
			self.unused.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

	def getSize(self):
		return self.SIZE

	def set_system_time(self, sys_time):
		self.system_time = sys_time

	def serialize(self):
		buf = []

		buf.extend(self.roll_angle.serialize())
		buf.extend(self.pitch_angle.serialize())
		buf.extend(self.ias.serialize())
		buf.extend(struct.pack('<B', self.lost_gps))
		buf.extend(struct.pack('<f', self.lost_gps_roll))
		buf.extend(struct.pack('<f', self.max_pdop))
		buf.extend(struct.pack('<f', self.max_scale_factor))
		buf.extend(self.flightpath_angle.serialize())
		buf.extend(self.flightpath_angle_flap.serialize())
		buf.extend(struct.pack('<f', self.flightpath_angle_fraction))

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

class VehicleParameters:
	SIZE = 66

	def __init__ (self, name = [None] * 16, flight_time = 0.0,
	standard_bank = 0.0, cruise_speed = 0.0, battery_cap = 0.0,
	battery_num_cells = 0, batt_chem = BatteryChemistry(0),
	unused = [None] * 32):
		if (len(name) != 16):
			raise ValueError('array name expecting length '+str(16)+' got '+str(len(name)))

		self.name = list(name)

		self.flight_time = flight_time
		self.standard_bank = standard_bank
		self.cruise_speed = cruise_speed
		self.battery_cap = battery_cap
		self.battery_num_cells = battery_num_cells

		self.batt_chem = BatteryChemistry(batt_chem)

		if (len(unused) != 32):
			raise ValueError('array unused expecting length '+str(32)+' got '+str(len(unused)))

		self.unused = list(unused)

	def parse(self,buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [VehicleParameters]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0

		self.name = [];

		for i in range(0,16):
			self.name.append(struct.unpack_from('<B',buf,offset)[0])
			offset = offset+struct.calcsize('<B')

		self.flight_time = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.standard_bank = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.cruise_speed = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.battery_cap = struct.unpack_from('<f',buf,offset)[0]
		offset = offset + struct.calcsize('<f')

		self.battery_num_cells = struct.unpack_from('<B',buf,offset)[0]
		offset = offset + struct.calcsize('<B')

		self.batt_chem = BatteryChemistry(struct.unpack_from('<B',buf,offset)[0])
		offset = offset+struct.calcsize('<B')

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

		for val in self.name:
		    buf.extend(struct.pack('<B', val))

		buf.extend(struct.pack('<f', self.flight_time))
		buf.extend(struct.pack('<f', self.standard_bank))
		buf.extend(struct.pack('<f', self.cruise_speed))
		buf.extend(struct.pack('<f', self.battery_cap))
		buf.extend(struct.pack('<B', self.battery_num_cells))

		buf.put(BatteryChemistry.encode(self.batt_chem));

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)

#---------[ Mission ]---------#

class MissionParameters:
	SIZE = 50

	def __init__ (self, altitude = 0, comm = 0,
	launch_type = LaunchType(0), land_type = LandType(0), max_range = 0.0,
	safe_height = 0.0, flight_time = 0.0, battery_min = 0.0, initialized = 0,
	mag_dec = 0.0, unused = [None] * 16):
		self.altitude = Limit(altitude)

		self.comm = Timeout(comm)

		self.launch_type = LaunchType(launch_type)

		self.land_type = LandType(land_type)

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

		self.launch_type = LaunchType(struct.unpack_from('<B',buf,offset)[0])
		offset = offset+struct.calcsize('<B')

		self.land_type = LandType(struct.unpack_from('<B',buf,offset)[0])
		offset = offset+struct.calcsize('<B')

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

		buf.put(LaunchType.encode(self.launch_type));

		buf.put(LandType.encode(self.land_type));

		buf.extend(struct.pack('<f', self.max_range))
		buf.extend(struct.pack('<f', self.safe_height))
		buf.extend(struct.pack('<f', self.flight_time))
		buf.extend(struct.pack('<f', self.battery_min))
		buf.extend(struct.pack('<B', self.initialized))
		buf.extend(struct.pack('<f', self.mag_dec))

		for val in self.unused:
		    buf.extend(struct.pack('<B', val))
		return bytearray(buf)
