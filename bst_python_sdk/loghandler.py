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

gps = type('', (), {})()
gps.velocity = type('', (), {})()
acc = type('', (), {})()
gyr = type('', (), {})()
dyn_p = type('', (), {})()
stat_p = type('', (), {})()
mag = type('', (), {})()
sensor_cal = type('', (), {})()

state = type('', (), {})()

command = type('', (), {})()
waypoint = type('', (), {})()
payload_trigger = type('', (), {})()

act = type('', (), {})()

sys_status = type('', (), {})()

last_system_time = 0


def initialize_variables():
    gps.system_time = []
    gps.hour = []
    gps.minute = []
    gps.seconds = []
    gps.latitude = []
    gps.longitude = []
    gps.altitude = []
    gps.speed = []
    gps.course = []
    gps.satellites = []
    gps.hdop = []
    gps.last_fix = []
    gps.velocity.system_time = []
    gps.velocity.x = []
    gps.velocity.y = []
    gps.velocity.z = []

    acc.system_time = []
    acc.x = []
    acc.y = []
    acc.z = []

    gyr.system_time = []
    gyr.x = []
    gyr.y = []
    gyr.z = []

    mag.system_time = []
    mag.x = []
    mag.y = []
    mag.z = []

    dyn_p.system_time = []
    dyn_p.pressure = []
    dyn_p.temperature = []

    stat_p.system_time = []
    stat_p.pressure = []
    stat_p.temperature = []

    state.system_time = []
    state.q = [[], [], [], []]
    state.altitude = []
    state.ias = []
    state.ias_dot = []
    state.h = []
    state.h_dot = []
    state.turnrate = []
    state.accel_y = []

    command.system_time = []
    command.id = []
    command.value = []

    sys_status.system_time = []
    sys_status.batt_voltage = []
    sys_status.batt_current = []
    sys_status.batt_percent = []
    sys_status.flight_time = []
    sys_status.rssi = []
    sys_status.lost_comm = []
    sys_status.lost_gps = []
    sys_status.error_code = []

    sensor_cal.system_time = []

    waypoint.system_time = []
    waypoint.num = []
    waypoint.next = []
    waypoint.latitude = []
    waypoint.longitude = []
    waypoint.altitude = []
    waypoint.radius = []
    waypoint.action = []

    payload_trigger.system_time = []
    payload_trigger.latitude = []
    payload_trigger.longitude = []
    payload_trigger.altitude = []

    act.system_time = []
    act.usec = [[], [], [], [], [], [], [], [], []]

    last_system_time = 0


def standard_handler(pkt):
    global gps, acc, gyr, mag, dyn_p, stat_p
    global state
    global command, sensor_cal, waypoint, payload_trigger
    global sys_status
    global act
    global last_system_time

    # -----[ ACCELERATIONS ]-----%
    if pkt.TYPE is PacketTypes.SENSORS_ACCELEROMETER:
        temp_struct = ThreeAxisSensor()
        try:
            temp_struct.parse(pkt.DATA)
            last_system_time = temp_struct.system_time
            acc.system_time.append(temp_struct.system_time)
            acc.x.append(temp_struct.x)
            acc.y.append(temp_struct.y)
            acc.z.append(temp_struct.z)
        except BufferError as ErrorMessage:
            print(ErrorMessage)

    # -----[ ROTATIONS ]-----%
    if pkt.TYPE is PacketTypes.SENSORS_GYROSCOPE:
        temp_struct = ThreeAxisSensor()
        try:
            temp_struct.parse(pkt.DATA)
            gyr.system_time.append(temp_struct.system_time)
            gyr.x.append(temp_struct.x)
            gyr.y.append(temp_struct.y)
            gyr.z.append(temp_struct.z)
        except BufferError as ErrorMessage:
            print(ErrorMessage)

    # -----[ PRESSURE ]-----%
    if pkt.TYPE is PacketTypes.SENSORS_DYNAMIC_PRESSURE:
        temp_struct = Pressure()
        try:
            temp_struct.parse(pkt.DATA)
            dyn_p.system_time.append(temp_struct.system_time)
            dyn_p.pressure.append(temp_struct.pressure)
            dyn_p.temperature.append(temp_struct.temperature)
        except BufferError as ErrorMessage:
            print(ErrorMessage)

    if pkt.TYPE is PacketTypes.SENSORS_STATIC_PRESSURE:
        temp_struct = Pressure()
        try:
            temp_struct.parse(pkt.DATA)
            stat_p.system_time.append(temp_struct.system_time)
            stat_p.pressure.append(temp_struct.pressure)
            stat_p.temperature.append(temp_struct.temperature)
        except BufferError as ErrorMessage:
            print(ErrorMessage)

    # -----[ GPS ]-----%
    if pkt.TYPE is PacketTypes.SENSORS_GPS:
        temp_struct = GPS()
        try:
            temp_struct.parse(pkt.DATA)
            gps.system_time.append(last_system_time)
            gps.hour.append(temp_struct.hour)
            gps.minute.append(temp_struct.minute)
            gps.seconds.append(temp_struct.seconds)
            gps.latitude.append(temp_struct.latitude)
            gps.longitude.append(temp_struct.longitude)
            gps.altitude.append(temp_struct.altitude)
            gps.speed.append(temp_struct.speed)
            gps.course.append(temp_struct.course)
            gps.satellites.append(temp_struct.satellites)
            gps.pdop.append(temp_struct.pdop)
            gps.last_fix.append(temp_struct.last_fix)
            gps.velocity.system_time.append(temp_struct.velocity.system_time)
            gps.velocity.x.append(temp_struct.velocity.x)
            gps.velocity.y.append(temp_struct.velocity.y)
            gps.velocity.z.append(temp_struct.velocity.z)

        except BufferError as ErrorMessage:
            print(ErrorMessage)

    # -----[ STATE ]-----%
    if pkt.TYPE is PacketTypes.STATE_STATE:
        temp_struct = State()
        try:
            temp_struct.parse(pkt.DATA)
            state.system_time.append(last_system_time)
            for i in range(0, 4):
                state.q[i].append(temp_struct.q[i])
            state.altitude.append(temp_struct.altitude)
            state.ias.append(temp_struct.ias)
            state.ias_dot.append(temp_struct.ias_dot)
            state.h.append(temp_struct.h)
            state.h_dot.append(temp_struct.h_dot)
            state.turnrate.append(temp_struct.turnrate)
            state.accel_y.append(temp_struct.accel_y)
        except BufferError as ErrorMessage:
            print(ErrorMessage)

    # -----[ WAYPOINTS ]-----%
    if pkt.TYPE is PacketTypes.FLIGHT_PLAN_WAYPOINT:
        temp_struct = Waypoint()
        try:
            temp_struct.parse(pkt.DATA)
            waypoint.system_time.append(last_system_time)
            waypoint.num.append(temp_struct.num)
            waypoint.next.append(temp_struct.next)
            waypoint.latitude.append(temp_struct.latitude)
            waypoint.longitude.append(temp_struct.longitude)
            waypoint.altitude.append(temp_struct.altitude)
            waypoint.radius.append(temp_struct.radius)
            waypoint.action.append(temp_struct.action)
        except BufferError as ErrorMessage:
            print(ErrorMessage)

    # -----[ PAYLOAD TRIGGER ]-----%
    if pkt.TYPE is PacketTypes.PAYLOAD_TRIGGER:
        temp_struct = PayloadTrigger()
        try:
            temp_struct.parse(pkt.DATA)
            payload_trigger.system_time.append(last_system_time)
            payload_trigger.latitude.append(temp_struct.latitude)
            payload_trigger.longitude.append(temp_struct.longitude)
            payload_trigger.altitude.append(temp_struct.altitude)
        except BufferError as ErrorMessage:
            print(ErrorMessage)

    # -----[ COMMANDS ]-----%
    if pkt.TYPE is PacketTypes.CONTROL_COMMAND:
        temp_struct = Command()
        try:
            temp_struct.parse(pkt.DATA)
            command.system_time.append(last_system_time)
            command.id.append(temp_struct.id)
            command.value.append(temp_struct.value)
        except BufferError as ErrorMessage:
            print(ErrorMessage)

    # -----[ ACTUATORS ]-----%
    if pkt.TYPE is PacketTypes.ACTUATORS_VALUES:
        temp_struct = Actuators()
        try:
            temp_struct.parse(pkt.DATA)
            act.system_time.append(last_system_time)
            for i in range(0, 9):
                act.usec[i].append(temp_struct.usec[i])
        except BufferError as ErrorMessage:
            print(ErrorMessage)

    # -----[ HEALTH AND STATUS ]-----%
    if pkt.TYPE is PacketTypes.SYSTEM_HEALTH_AND_STATUS:
        temp_struct = SystemStatus()
        try:
            temp_struct.parse(pkt.DATA)
            sys_status.system_time.append(last_system_time)
            sys_status.batt_voltage.append(temp_struct.batt_voltage)
            sys_status.batt_current.append(temp_struct.batt_current)
            sys_status.batt_percent.append(temp_struct.batt_percent)
            sys_status.flight_time.append(temp_struct.flight_time)
            sys_status.rssi.append(temp_struct.rssi)
            sys_status.lost_comm.append(temp_struct.lost_comm)
            sys_status.lost_gps.append(temp_struct.lost_gps)
            sys_status.error_code.append(temp_struct.error_code)
        except BufferError as ErrorMessage:
            print(ErrorMessage)
