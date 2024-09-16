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
from .comm_packets.fixedwing import *
from .comm_packets.canpackets import *

packet_mapping = {
    # GCS Packets
    PacketTypes.TELEMETRY_GCS.value: GCSStatus,
    PacketTypes.TELEMETRY_GCS_LOCATION.value: TelemetryGCS,
    PacketTypes.TELEMETRY_GCS_SVIN.value: GCSSurveyIn,

    # Aircraft Packets
    PacketTypes.ACTUATORS_CALIBRATION.value: ActuatorCalibration,
    PacketTypes.ACTUATORS_MIXING_PARAMS.value: SurfaceMixing,
    PacketTypes.ACTUATORS_VALUES.value: Actuators,
    PacketTypes.CONTROL_COMMAND.value: Command,
    PacketTypes.CONTROL_FILTER_PARAMS.value: FilterParameters,
    PacketTypes.CONTROL_FLIGHT_PARAMS.value: FlightControlParameters,
    PacketTypes.CONTROL_PID.value: PID,
    PacketTypes.DUBIN_PATH.value: DubinsPath,
    PacketTypes.FLIGHT_PLAN_MAP.value: FlightPlanMap,
    PacketTypes.FLIGHT_PLAN_WAYPOINT.value: Waypoint,
    PacketTypes.HANDSET_CALIBRATION.value: HandsetCalibration,
    PacketTypes.INPUT_HANDSET_VALUES.value: HandsetValues,
    PacketTypes.LAST_MAPPING_WAYPOINT.value: int,
    PacketTypes.MISSION_PARAMETERS.value: MissionParameters,
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
    PacketTypes.VEHICLE_LAND_PARAMS.value: LandingParameters,
    PacketTypes.VEHICLE_LAUNCH_PARAMS.value: LaunchParameters,
    PacketTypes.VEHICLE_LIMITS.value: VehicleLimits,
    PacketTypes.VEHICLE_PARAMS.value: VehicleParameters,
}

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


def standard_handler(pkt):
    packet_data = None

    if pkt.TYPE.value not in packet_mapping:
        if pkt.TYPE != PacketTypes.TELEMETRY_HEARTBEAT and 'HWIL' not in pkt.TYPE.name:
            print(f'Parsing not set up for "{pkt.TYPE.name}" packet...')
        return None

    try:
        if pkt.TYPE.value >= PacketTypes.PAYLOAD_DATA_CHANNEL_0.value:
            packet_mapping[pkt.TYPE.value].buffer = [None] * 64

        if packet_mapping[pkt.TYPE.value] == int:
            packet_data = int.from_bytes(pkt.DATA)
        else:
            pkt_cls = packet_mapping[pkt.TYPE.value]()
            pkt_cls.parse(pkt.DATA)
            packet_data = pkt_cls
    except BufferError as ErrorMessage:
        print(ErrorMessage)

    return packet_data
