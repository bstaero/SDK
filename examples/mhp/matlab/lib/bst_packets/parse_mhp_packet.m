function mhp = parse_mhp_packet(data)

mhp.error_code = data(1);
mhp.static_pressure_time = double(typecast(uint8(data(2:5)),'single'));
mhp.static_pressure = double(typecast(uint8(data(6:9)),'single'));
mhp.dynamic_pressure_time(1) = double(typecast(uint8(data(10:13)),'single'));
mhp.dynamic_pressure_time(2) = double(typecast(uint8(data(14:17)),'single'));
mhp.dynamic_pressure_time(3) = double(typecast(uint8(data(18:21)),'single'));
mhp.dynamic_pressure_time(4) = double(typecast(uint8(data(22:25)),'single'));
mhp.dynamic_pressure_time(5) = double(typecast(uint8(data(26:29)),'single'));
mhp.dynamic_pressure(1) = double(typecast(uint8(data(30:33)),'single'));
mhp.dynamic_pressure(2) = double(typecast(uint8(data(34:37)),'single'));
mhp.dynamic_pressure(3) = double(typecast(uint8(data(38:41)),'single'));
mhp.dynamic_pressure(4) = double(typecast(uint8(data(42:45)),'single'));
mhp.dynamic_pressure(5) = double(typecast(uint8(data(46:49)),'single'));
mhp.air_temperature_time = double(typecast(uint8(data(50:53)),'single'));
mhp.air_temperature = double(typecast(uint8(data(54:57)),'single'));
mhp.humidity_time = double(typecast(uint8(data(58:61)),'single'));
mhp.humidity = double(typecast(uint8(data(62:65)),'single'));
mhp.imu_time = double(typecast(uint8(data(66:69)),'single'));
mhp.gyroscope(1) = double(typecast(uint8(data(70:73)),'single'));
mhp.gyroscope(2) = double(typecast(uint8(data(74:77)),'single'));
mhp.gyroscope(3) = double(typecast(uint8(data(78:81)),'single'));
mhp.accelerometer(1) = double(typecast(uint8(data(82:85)),'single'));
mhp.accelerometer(2) = double(typecast(uint8(data(86:89)),'single'));
mhp.accelerometer(3) = double(typecast(uint8(data(90:93)),'single'));
mhp.magnetometer_time = double(typecast(uint8(data(94:97)),'single'));
mhp.magnetometer(1) = double(typecast(uint8(data(98:101)),'single'));
mhp.magnetometer(2) = double(typecast(uint8(data(102:105)),'single'));
mhp.magnetometer(3) = double(typecast(uint8(data(106:109)),'single'));
mhp.alpha = double(typecast(uint8(data(110:113)),'single'));
mhp.beta = double(typecast(uint8(data(114:117)),'single'));
mhp.q = double(typecast(uint8(data(118:121)),'single'));
mhp.ias = double(typecast(uint8(data(122:125)),'single'));
mhp.tas = double(typecast(uint8(data(126:129)),'single'));

% Close enough for plotting
mhp.time = mhp.dynamic_pressure_time(1);
