function setup()
%% setup.m
% Black Swift Technologies Multi-Hole Probe Matlab Interface
%
% This script will setup a connection the the BST MHP and provide
% telemetery as well as a control interface.
% 
% Author - Jack Elston
%          elstonj@bst.aero

%% Clean up
clear all

%% Add Libraries and controllers folder to MATLAB path
addpath lib/
addpath lib/communications
addpath lib/bst_packets

%% Local Variables

SENSORS_MHP=13;
SENSORS_MHP_SENSORS=15;
SENSORS_MHP_GNSS=30;
SENSORS_MHP_TIMING=31;
SENSORS_CALIBRATE=10;

GYROSCOPE         = 1;
DYNAMIC_PRESSURE  = 3;
UNKNOWN_SENSOR    = 14;

CAL_UNKNOWN       = 0;
REQUESTED         = 1;
SENT              = 2;
CALIBRATED        = 3;

%% Global Variables
global comm_port running waiting_for_ack dyn_btn gyro_btn

running = 1;
waiting_for_ack.sensor = UNKNOWN_SENSOR;
waiting_for_ack.counter = 0;

%% Interface Code
[figure] = setup_plot();
index = 1;

comm_port = [];
data_remaining = [];

output.mhp_time = 0;
output.alpha = 0;
output.beta = 0;
output.q = 0;
output.ias = 0;
output.tas = 0;
output.wind = 0;

output.mhp_sensors_time = 0;
output.error_code = 0;
output.static_pressure = 0;
output.dynamic_pressure(1,1) = 0;
output.dynamic_pressure(1,2) = 0;
output.dynamic_pressure(1,3) = 0;
output.dynamic_pressure(1,4) = 0;
output.dynamic_pressure(1,5) = 0;
output.air_temperature = 0;
output.humidity = 0;
output.gyroscope(1,1) = 0;
output.gyroscope(1,2) = 0;
output.gyroscope(1,3) = 0;
output.accelerometer(1,1) = 0;
output.accelerometer(1,2) = 0;
output.accelerometer(1,3) = 0;

output.mhp_gnss_time = 0;
output.magnetometer(1,1) = 0;
output.magnetometer(1,2) = 0;
output.magnetometer(1,3) = 0;
output.week = 0;
output.hour = 0;
output.minute = 0;
output.seconds = 0;
output.latitude = 0;
output.longitude = 0;
output.altitude = 0;
output.velocity = 0;
output.pdop = 0;

output.static_pressure_time = 0;
output.dynamic_pressure_time = 0;
output.air_temperature_time = 0;
output.humidity_time = 0;
output.imu_time = 0;
output.magnetometer_time = 0;
output.gps_time = 0;

while running == 1

	drawnow

  if isempty(comm_port)
		continue
	end

	if waiting_for_ack.sensor ~= UNKNOWN_SENSOR
		waiting_for_ack.counter = waiting_for_ack.counter+1;
		if  waiting_for_ack.counter > 20
			packet = make_calibration_packet(waiting_for_ack.sensor,REQUESTED);
			fwrite(comm_port,packet);
			waiting_for_ack.counter = 0;
		end
	end

	if comm_port.BytesAvailable > 0

		data = [data_remaining; fread(comm_port,comm_port.BytesAvailable)];
		[Packet_data, Pack_count, Type, data_remaining] = bst_packet_handler(data);

		for p = 1:Pack_count
			
			if( Type(p) == SENSORS_MHP )

				mhp_data = parse_mhp_packet(Packet_data{p});

				output.mhp_time(index) = mhp_data.time;
				output.alpha(index) = mhp_data.alpha;
				output.beta(index) = mhp_data.beta;
				output.q(index) = mhp_data.q;
				output.ias(index) = mhp_data.ias;
				output.tas(index) = mhp_data.tas;
				output.wind(index,1) = mhp_data.wind(1);
				output.wind(index,2) = mhp_data.wind(2);
				output.wind(index,3) = mhp_data.wind(3);

				if index == 1000
					output.mhp_time(1:999) = output.mhp_time(2:1000);
					output.alpha(1:999) = output.alpha(2:1000);
					output.beta(1:999) = output.beta(2:1000);
					output.q(1:999) = output.q(2:1000);
					output.ias(1:999) = output.ias(2:1000);
					output.tas(1:999) = output.tas(2:1000);
					output.wind(1:999,1) = output.wind(2:1000,1);
					output.wind(1:999,2) = output.wind(2:1000,2);
					output.wind(1:999,3) = output.wind(2:1000,3);
				else
					index = (index+1);
				end

				plot_mhp_data(output,figure);

			end

			if( Type(p) == SENSORS_MHP_SENSORS )

				mhp_data = parse_mhp_sensors_packet(Packet_data{p});

				output.mhp_sensors_time(index) = mhp_data.time;
				output.error_code(index) = mhp_data.error_code;
				output.static_pressure(index) = mhp_data.static_pressure;
				output.dynamic_pressure(index,1) = mhp_data.dynamic_pressure(1);
				output.dynamic_pressure(index,2) = mhp_data.dynamic_pressure(2);
				output.dynamic_pressure(index,3) = mhp_data.dynamic_pressure(3);
				output.dynamic_pressure(index,4) = mhp_data.dynamic_pressure(4);
				output.dynamic_pressure(index,5) = mhp_data.dynamic_pressure(5);
				output.air_temperature(index) = mhp_data.air_temperature;
				output.humidity(index) = mhp_data.humidity;
				output.gyroscope(index,1) = mhp_data.gyroscope(1);
				output.gyroscope(index,2) = mhp_data.gyroscope(2);
				output.gyroscope(index,3) = mhp_data.gyroscope(3);
				output.accelerometer(index,1) = mhp_data.accelerometer(1);
				output.accelerometer(index,2) = mhp_data.accelerometer(2);
				output.accelerometer(index,3) = mhp_data.accelerometer(3);

				if index == 1000
					output.mhp_sensors_time(1:999) = output.mhp_sensors_time(2:1000);
					output.error_code(1:999) = output.error_code(2:1000);
					output.static_pressure(1:999) = output.static_pressure(2:1000);
					output.dynamic_pressure(1:999,1) = output.dynamic_pressure(2:1000,1);
					output.dynamic_pressure(1:999,2) = output.dynamic_pressure(2:1000,2);
					output.dynamic_pressure(1:999,3) = output.dynamic_pressure(2:1000,3);
					output.dynamic_pressure(1:999,4) = output.dynamic_pressure(2:1000,4);
					output.dynamic_pressure(1:999,5) = output.dynamic_pressure(2:1000,5);
					output.air_temperature(1:999) = output.air_temperature(2:1000);
					output.humidity(1:999) = output.humidity(2:1000);
					output.gyroscope(1:999,1) = output.gyroscope(2:1000,1);
					output.gyroscope(1:999,2) = output.gyroscope(2:1000,2);
					output.gyroscope(1:999,3) = output.gyroscope(2:1000,3);
					output.accelerometer(1:999,1) = output.accelerometer(2:1000,1);
					output.accelerometer(1:999,2) = output.accelerometer(2:1000,2);
					output.accelerometer(1:999,3) = output.accelerometer(2:1000,3);
				end

			end

			if( Type(p) == SENSORS_MHP_GNSS )

				mhp_data = parse_mhp_gnss_packet(Packet_data{p});

				output.mhp_gnss_time(index) = mhp_data.time;
				output.magnetometer(index,1) = mhp_data.magnetometer(1);
				output.magnetometer(index,2) = mhp_data.magnetometer(2);
				output.magnetometer(index,3) = mhp_data.magnetometer(3);
				output.week(index) = mhp_data.week;
				output.hour(index) = mhp_data.hour;
				output.minute(index) = mhp_data.minute;
				output.seconds(index) = mhp_data.seconds;
				output.latitude(index) = mhp_data.latitude;
				output.longitude(index) = mhp_data.longitude;
				output.altitude(index) = mhp_data.altitude;
				output.velocity(index,1) = mhp_data.velocity(1);
				output.velocity(index,2) = mhp_data.velocity(2);
				output.velocity(index,3) = mhp_data.velocity(3);
				output.pdop(index) = mhp_data.pdop;

				if index == 1000
					output.mhp_gnss_time(1:999) = output.mhp_gnss_time(2:1000);
					output.magnetometer(1:999,1) = output.magnetometer(2:1000,1);
					output.magnetometer(1:999,2) = output.magnetometer(2:1000,2);
					output.magnetometer(1:999,3) = output.magnetometer(2:1000,3);
					output.week(1:999) = output.week(2:1000);
					output.hour(1:999) = output.hour(2:1000);
					output.minute(1:999) = output.minute(2:1000);
					output.seconds(1:999) = output.seconds(2:1000);
					output.latitude(1:999) = output.latitude(2:1000);
					output.longitude(1:999) = output.longitude(2:1000);
					output.altitude(1:999) = output.altitude(2:1000);
					output.velocity(1:999,1) = output.velocity(2:1000,1);
					output.velocity(1:999,2) = output.velocity(2:1000,2);
					output.velocity(1:999,3) = output.velocity(2:1000,3);
					output.pdop(1:999) = output.pdop(2:1000);
				end

			end

			if( Type(p) == SENSORS_CALIBRATE )
				calibrate_data = parse_calibrate_packet(Packet_data{p});

				if calibrate_data.state == CALIBRATED

					if calibrate_data.sensor == GYROSCOPE
						set(gyro_btn,'BackgroundColor',[0.95,0.95,0.95]);
						waiting_for_ack.sensor = UNKNOWN_SENSOR;
					end

					if calibrate_data.sensor == DYNAMIC_PRESSURE
						set(dyn_btn,'BackgroundColor',[0.95,0.95,0.95]);
						waiting_for_ack.sensor = UNKNOWN_SENSOR;
					end

				end

			end

		end

	else
		%pause(0.01);
	end

end

%% Close Comm Port
terminate_connection;

close all;
