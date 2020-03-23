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

				output.error_code(index) = mhp_data.error_code;
				output.time(index) = mhp_data.time;
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
				output.magnetometer(index,1) = mhp_data.magnetometer(1);
				output.magnetometer(index,2) = mhp_data.magnetometer(2);
				output.magnetometer(index,3) = mhp_data.magnetometer(3);
				output.alpha(index) = mhp_data.alpha;
				output.beta(index) = mhp_data.beta;
				output.q(index) = mhp_data.q;
				output.ias(index) = mhp_data.ias;
				output.tas(index) = mhp_data.tas;

				if index == 1000
					output.error_code(1:999) = output.error_code(2:1000);
					output.time(1:999) = output.time(2:1000);
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
					output.magnetometer(1:999,1) = output.magnetometer(2:1000,1);
					output.magnetometer(1:999,2) = output.magnetometer(2:1000,2);
					output.magnetometer(1:999,3) = output.magnetometer(2:1000,3);
					output.alpha(1:999) = output.alpha(2:1000);
					output.beta(1:999) = output.beta(2:1000);
					output.q(1:999) = output.q(2:1000);
					output.ias(1:999) = output.ias(2:1000);
					output.tas(1:999) = output.tas(2:1000);
				else
					index = (index+1);
				end

				plot_mhp_data(output,figure);

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
		pause(0.1);
	end

end

%% Close Comm Port
terminate_connection;

close all;
