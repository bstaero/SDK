#include "bst_module_basic.h"

#include "gcs.h"

#include "debug.h"

using namespace bst::comms;
using namespace bst::comms::gcs;

BSTModuleBasic::BSTModuleBasic() : BSTCommunicationsModule() {
	pmesg(VERBOSE_ALLOC, "BSTModuleBasic::BSTModuleBasic()\n");

	max_num_data_types = MAX_DATA_TYPES;
	data_types = new DataType_t[MAX_DATA_TYPES]; 

	/* SENSORS */
	registerDataType(SENSORS_GPS, sizeof(GPS_t), false, false);
	registerDataType(SENSORS_GYROSCOPE, sizeof(ThreeAxisSensor_t), false, false);
	registerDataType(SENSORS_ACCELEROMETER, sizeof(ThreeAxisSensor_t), false, false);
	registerDataType(SENSORS_MAGNETOMETER, sizeof(ThreeAxisSensor_t), false, false);
	registerDataType(SENSORS_IMU, sizeof(IMU_t), false, false);
	registerDataType(SENSORS_DYNAMIC_PRESSURE, sizeof(Pressure_t), false, false);
	registerDataType(SENSORS_STATIC_PRESSURE, sizeof(Pressure_t), false, false);
	registerDataType(SENSORS_AIR_TEMPERATURE, sizeof(SingleValueSensor_t), false, false);
	registerDataType(SENSORS_AGL, sizeof(SingleValueSensor_t), false, false);
	registerDataType(SENSORS_CALIBRATE, sizeof(CalibrateSensor_t), true, true);
	registerDataType(SENSORS_BOARD_ORIENTATION, sizeof(AxisMapping_t), true, true);
	registerDataType(SENSORS_GNSS_ORIENTATION, sizeof(AxisMapping_t), true, true);

	/* STATE */
	registerDataType(STATE_STATE, sizeof(State_t), false, false); // FIXME
	registerDataType(STATE_ESTIMATOR_PARAM, sizeof(EstimatorParameters_t), true, true);

	/* CONTROL */
	//registerDataType(CONTROL_VALUES, sizeof(Control_t), true, true);
	registerDataType(CONTROL_COMMAND, sizeof(Command_t), true, true);
	registerDataType(CONTROL_PID, sizeof(PID_t), true, true);
	registerDataType(CONTROL_FLIGHT_PARAMS, sizeof(FlightControlParameters_t), true, true);
#if defined(VEHICLE_FIXEDWING)
	registerDataType(CONTROL_FILTER_PARAMS, sizeof(FilterParameters_t), true, true);
#endif
	// registerDataType(CONTROL_CMD_FILTER, sizeof(CommandFilter_t), true, true);

	/* ACTUATORS */
	registerDataType(ACTUATORS_VALUES, sizeof(Actuators_t), true, true);
	registerDataType(ACTUATORS_CALIBRATION, sizeof(ActuatorCalibration_t), true, true);
#if defined(VEHICLE_FIXEDWING)
	registerDataType(ACTUATORS_MIXING_PARAMS, sizeof(SurfaceMixing_t), true, true);
#elif defined(VEHICLE_MULTIROTOR)
	registerDataType(ACTUATORS_ROTOR_PARAMS, sizeof(RotorParameters_t), true, true);
#endif
	//registerDataType(ACTUATORS_ROTOR_PARAMS, sizeof(), true, true);

	/* HANDSET */
	registerDataType(HANDSET_CALIBRATION, sizeof(HandsetCalibration_t), true, true);

	/* INPUT */
	registerDataType(INPUT_HANDSET_VALUES, sizeof(HandsetValues_t), true, false);
	//registerDataType(INPUT_HANDSET_SETUP, sizeof(), true, true);
	registerDataType(INPUT_JOYSTICK_VALUES, sizeof(TabletJoystick_t), true, false);
	registerDataType(INPUT_JOYSTICK_SETUP, sizeof(JoystickFunction_t), true, true);

	/* SYSTEM */
	registerDataType(SYSTEM_POWER_ON, sizeof(PowerOn_t), false, false);
	registerDataType(SYSTEM_INITIALIZE, sizeof(SystemInitialize_t), true, true);
	registerDataType(SYSTEM_HEALTH_AND_STATUS, sizeof(SystemStatus_t), false, true);
	registerDataType(SYSTEM_REBOOT, 0, true, false);

	/* TELEMETRY */
	registerDataType(TELEMETRY_HEARTBEAT, 0, false, false);
	registerDataType(TELEMETRY_POSITION, sizeof(TelemetryPosition_t), false, false);
	registerDataType(TELEMETRY_ORIENTATION, sizeof(TelemetryOrientation_t), false, false);
	registerDataType(TELEMETRY_PRESSURE, sizeof(TelemetryPressure_t), false, false);
	registerDataType(TELEMETRY_CONTROL, sizeof(TelemetryControl_t), false, false);
	registerDataType(TELEMETRY_SYSTEM, sizeof(TelemetrySystem_t), false, false);
	registerDataType(TELEMETRY_GCS_LOCATION, sizeof(TelemetryGCS_t), false, false);
	registerDataType(TELEMETRY_PAYLOAD, sizeof(TelemetryPayload_t), false, false);
	//registerDataType(TELEMETRY_GCS, sizeof(), false, false);

	/* HWIL */
	//registerDataType(HWIL_SENSORS, sizeof(), false, false);
	//registerDataType(HWIL_ACTUATORS, sizeof(), false, false);

	/* VEHICLE CONFIGURATION */
	registerDataType(VEHICLE_PARAMS, sizeof(VehicleParameters_t), true, true);
	registerDataType(VEHICLE_LIMITS, sizeof(VehicleLimits_t), true, true);
	registerDataType(LAST_MAPPING_WAYPOINT, sizeof(uint8_t), true, true);
	registerDataType(VEHICLE_LAUNCH_PARAMS, sizeof(LaunchParameters_t), true, true);
	registerDataType(VEHICLE_LAND_PARAMS, sizeof(LandingParameters_t), true, true);

	/* MISSION */
	registerDataType(MISSION_PARAMETERS, sizeof(MissionParameters_t), true, true);

	/* PAYLOAD */
	//registerDataType(PAYLOAD_NDVI, sizeof(NDVI_t), false, false); // FIXME

	registerDataType(PAYLOAD_PARAMS, sizeof(PayloadParam_t), true, true); 
	registerDataType(PAYLOAD_CONTROL, sizeof(PayloadControl_t), true, true); 
	registerDataType(PAYLOAD_TRIGGER, sizeof(PayloadTrigger_t), false, false); 
	registerDataType(PAYLOAD_CAMERA_TAG, sizeof(CameraTag_t), false, false); 
	registerDataType(PAYLOAD_STATUS, sizeof(PayloadStatus_t), false, true); 

	registerDataType(PAYLOAD_CHANNEL_0, sizeof(UserPayload_t), false, false); 
	registerDataType(PAYLOAD_CHANNEL_1, sizeof(UserPayload_t), false, false); 
	registerDataType(PAYLOAD_CHANNEL_2, sizeof(UserPayload_t), false, false); 
	registerDataType(PAYLOAD_CHANNEL_3, sizeof(UserPayload_t), false, false); 
	registerDataType(PAYLOAD_CHANNEL_4, sizeof(UserPayload_t), false, false); 
	registerDataType(PAYLOAD_CHANNEL_5, sizeof(UserPayload_t), false, false); 
	registerDataType(PAYLOAD_CHANNEL_6, sizeof(UserPayload_t), false, false); 
	registerDataType(PAYLOAD_CHANNEL_7, sizeof(UserPayload_t), false, false); 
}
