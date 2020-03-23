function packet = make_calibration_packet(sensor_type, action)

SENSORS_CALIBRATE = 10;
 
% CAL_UNKNOWN       = 0;
% REQUESTED         = 1;
% SENT              = 2;
% CALIBRATED        = 3;
 
% ACCELEROMETER     = 0;
% GYROSCOPE         = 1;
% MAGNETOMETER      = 2;
% DYNAMIC_PRESSURE  = 3;
% STATIC_PRESSURE   = 4;
% TEMPERATURE       = 5;
% HUMIDITY          = 6;
% AGL               = 7;
% GPS               = 8;
% SENSOR_PAYLOAD_1  = 9;
% SENSOR_PAYLOAD_2  = 10;
% SENSOR_PAYLOAD_3  = 11;
% SENSOR_PAYLOAD_4  = 12;
% SENSOR_PAYLOAD_5  = 13;
% UNKNOWN_SENSOR    = 14;

information(1) = uint8(sensor_type);
information(2) = uint8(action);

packet = make_packet(SENSORS_CALIBRATE, information, 2);
