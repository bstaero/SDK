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
from .comm_packets.gcs import *
from .comm_packets.payload import *
from .comm_packets import fixedwing
from .comm_packets import multicopter
from .comm_packets.canpackets import *

packet_mapping = {
    # GCS Packets
    PacketTypes.TELEMETRY_GCS.value: GCSStatus,
    PacketTypes.TELEMETRY_GCS_LOCATION.value: TelemetryGCS,
    PacketTypes.TELEMETRY_GCS_SVIN.value: GCSSurveyIn,

    # Aircraft Packets
    PacketTypes.ACTUATORS_CALIBRATION.value: ActuatorCalibration,
    PacketTypes.ACTUATORS_VALUES.value: Actuators,
    PacketTypes.CONTROL_COMMAND.value: Command,
    PacketTypes.CONTROL_PID.value: PID,
    PacketTypes.DUBIN_PATH.value: DubinsPath,
    PacketTypes.FLIGHT_PLAN_MAP.value: FlightPlanMap,
    PacketTypes.FLIGHT_PLAN_WAYPOINT.value: Waypoint,
    PacketTypes.HANDSET_CALIBRATION.value: HandsetCalibration,
    PacketTypes.INPUT_HANDSET_VALUES.value: HandsetValues,
    PacketTypes.LAST_MAPPING_WAYPOINT.value: int,
    PacketTypes.PAYLOAD_DATA_CHANNEL_0.value: UserPayload,
    PacketTypes.PAYLOAD_DATA_CHANNEL_1.value: UserPayload,
    PacketTypes.PAYLOAD_DATA_CHANNEL_2.value: UserPayload,
    PacketTypes.PAYLOAD_DATA_CHANNEL_3.value: UserPayload,
    PacketTypes.PAYLOAD_DATA_CHANNEL_4.value: UserPayload,
    PacketTypes.PAYLOAD_DATA_CHANNEL_5.value: UserPayload,
    PacketTypes.PAYLOAD_DATA_CHANNEL_6.value: UserPayload,
    PacketTypes.PAYLOAD_DATA_CHANNEL_7.value: UserPayload,
    PacketTypes.PAYLOAD_PARAMS.value: PayloadParam,
    PacketTypes.PAYLOAD_S0_SENSORS.value: S0Sensors,
    PacketTypes.PAYLOAD_SERIAL.value: PayloadSerial,
    PacketTypes.SENSORS_ACCELEROMETER.value: ThreeAxisSensor,
    PacketTypes.SENSORS_AGL.value: SingleValueSensor,
    PacketTypes.SENSORS_AIR_TEMPERATURE.value: SingleValueSensor,
    PacketTypes.SENSORS_BOARD_ORIENTATION.value: AxisMapping,
    PacketTypes.SENSORS_CALIBRATE.value: CalibrateSensor,
    PacketTypes.SENSORS_DYNAMIC_PRESSURE.value: Pressure,
    PacketTypes.SENSORS_DYNP_CALIBRATION.value: SingleAxisSensorCalibration,
    PacketTypes.SENSORS_GNSS_ORIENTATION.value: AxisMapping,
    PacketTypes.SENSORS_GNSS_RTCM.value: RTCM,
    PacketTypes.SENSORS_GPS.value: GPS,
    PacketTypes.SENSORS_GYROSCOPE.value: ThreeAxisSensor,
    PacketTypes.SENSORS_GYRO_CALIBRATION.value: ThreeAxisSensorCalibration,
    PacketTypes.SENSORS_HUMIDITY.value: SingleValueSensor,
    PacketTypes.SENSORS_MAGNETOMETER.value: ThreeAxisSensor,
    PacketTypes.SENSORS_MAG_CALIBRATION.value: ThreeAxisSensorCalibration,
    PacketTypes.SENSORS_MAG_CURRENT_CAL.value: ThreeAxisFirstOrderCorrection,
    PacketTypes.SENSORS_MHP_SENSORS.value: MHPSensors,
    PacketTypes.SENSORS_STATIC_PRESSURE.value: Pressure,
    PacketTypes.STATE_ESTIMATOR_PARAM.value: EstimatorParameters,
    PacketTypes.STATE_STATE.value: State,
    PacketTypes.SYSTEM_HEALTH_AND_STATUS.value: SystemStatus,
    PacketTypes.SYSTEM_INITIALIZE.value: SystemInitialize,
    PacketTypes.SYSTEM_POWER_ON.value: PowerOn,
    PacketTypes.TELEMETRY_CONTROL.value: TelemetryControl,
    PacketTypes.TELEMETRY_DEPLOYMENT_TUBE.value: DeploymentTube,
    PacketTypes.TELEMETRY_ORIENTATION.value: TelemetryOrientation,
    PacketTypes.TELEMETRY_PAYLOAD.value: TelemetryPayload,
    PacketTypes.TELEMETRY_POSITION.value: TelemetryPosition,
    PacketTypes.TELEMETRY_PRESSURE.value: TelemetryPressure,
    PacketTypes.TELEMETRY_SYSTEM.value: TelemetrySystem,
}

fw_mapping = {
    PacketTypes.ACTUATORS_MIXING_PARAMS.value: fixedwing.SurfaceMixing,
    PacketTypes.CONTROL_FILTER_PARAMS.value: fixedwing.FilterParameters,
    PacketTypes.CONTROL_FLIGHT_PARAMS.value: fixedwing.FlightControlParameters,
    PacketTypes.MISSION_PARAMETERS.value: fixedwing.MissionParameters,
    PacketTypes.VEHICLE_LAND_PARAMS.value: fixedwing.LandingParameters,
    PacketTypes.VEHICLE_LAUNCH_PARAMS.value: fixedwing.LaunchParameters,
    PacketTypes.VEHICLE_LIMITS.value: fixedwing.VehicleLimits,
    PacketTypes.VEHICLE_PARAMS.value: fixedwing.VehicleParameters,
}

mr_mapping = {
    PacketTypes.MISSION_PARAMETERS.value: multicopter.MissionParameters,
    PacketTypes.ACTUATORS_ROTOR_PARAMS.value: multicopter.RotorParameters,
    PacketTypes.VEHICLE_LAND_PARAMS.value: multicopter.LandingParameters,
    PacketTypes.VEHICLE_LAUNCH_PARAMS.value: multicopter.LaunchParameters,
    PacketTypes.VEHICLE_LIMITS.value: multicopter.VehicleLimits,
    PacketTypes.VEHICLE_PARAMS.value: multicopter.VehicleParameters,
}

primitive_pkts = [
    PacketTypes.LAST_MAPPING_WAYPOINT.value,
]

ignore_pkts = [
    PacketTypes.TELEMETRY_HEARTBEAT.value
]

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


def standard_handler(pkt, sys_time=0, vehicle_type=VehicleType.VEHICLE_UNKNOWN):
    packet_data = None

    pkt_map = packet_mapping
    if pkt.TYPE.value not in pkt_map:
        if vehicle_type != VehicleType.VEHICLE_UNKNOWN:
            if vehicle_type == VehicleType.FIXED_WING and pkt.TYPE.value in fw_mapping:
                pkt_map = fw_mapping
            elif vehicle_type == VehicleType.MULTI_COPTER and pkt.TYPE.value in mr_mapping:
                pkt_map = mr_mapping
            else:
                if pkt.TYPE.value not in ignore_pkts:
                    print(f'Parsing not set up for "{vehicle_type.name}.{pkt.TYPE.name}" packet...')
                return None, sys_time
        else:
            if pkt.TYPE != PacketTypes.TELEMETRY_HEARTBEAT and 'HWIL' not in pkt.TYPE.name:
                print(f'Parsing not set up for "{pkt.TYPE.name}" packet...')
            return None, sys_time

    try:
        if pkt.TYPE.value >= PacketTypes.PAYLOAD_DATA_CHANNEL_0.value:
            pkt_map[pkt.TYPE.value].buffer = [None] * 64

        if pkt_map[pkt.TYPE.value] == int:
            packet_data = int.from_bytes(pkt.DATA)
        else:
            pkt_cls = pkt_map[pkt.TYPE.value]()
            pkt_cls.parse(pkt.DATA)
            packet_data = pkt_cls
    except BufferError as ErrorMessage:
        print(ErrorMessage)
        return None, sys_time

    if hasattr(packet_data, "system_time"):
        return packet_data, packet_data.system_time
    elif pkt.TYPE.value not in primitive_pkts:
        packet_data.set_system_time(sys_time)
    return packet_data, sys_time
