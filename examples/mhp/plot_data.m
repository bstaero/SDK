%*=+--+=#=+--            SwiftPilot Autopilot Software            --+=#=+--+=#*%
%               Copyright (C) 2020 Black Swift Technologies LLC.               %
%                             All Rights Reserved.                             %
%                                                                              %
%    This program is free software: you can redistribute it and/or modify      %
%    it under the terms of the GNU General Public License version 2 as         %
%    published by the Free Software Foundation.                                %
%                                                                              %
%    This program is distributed in the hope that it will be useful,           %
%    but WITHOUT ANY WARRANTY; without even the implied warranty of            %
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             %
%    GNU General Public License for more details.                              %
%                                                                              %
%    You should have received a copy of the GNU General Public License         %
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.     %
%                                                                              %
%                                 Jack Elston                                  %
%                          elstonj@blackswifttech.com                          %
%                                                                              %
%*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*%
function plot_data(data)

	start_index = 1000;
	fig_index = 1;

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	ax(fig_index) = subplot(1,1,1);
	plot(data.STATIC_PRESSURE_TIME(start_index:end),...
			data.STATIC_PRESSURE(start_index:end));

	title('Static Pressure [hPa]');
	xlabel('time [s]');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	ax(fig_index) = subplot(1,1,1);
	plot(data.MAGNETOMETER_TIME(start_index:end),...
			[data.MAGNETOMETER_X(start_index:end),...
			 data.MAGNETOMETER_Y(start_index:end),...
			 data.MAGNETOMETER_Z(start_index:end),]);

	title('Magnetometer [uT]');
	legend('x','y','z');
	xlabel('time [s]');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	ax(fig_index) = subplot(1,1,1);
	plot(data.IMU_TIME(start_index:end),...
			[data.ACCELEROMETER_X(start_index:end),...
			 data.ACCELEROMETER_Y(start_index:end),...
			 data.ACCELEROMETER_Z(start_index:end),]);

	title('Accelerometers [G]');
	legend('x','y','z');
	xlabel('time [s]');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	ax(fig_index) = subplot(1,1,1);
	plot(data.IMU_TIME(start_index:end),...
			[data.GYROSCOPE_X(start_index:end).*180/pi,...
			 data.GYROSCOPE_Y(start_index:end).*180/pi,...
			 data.GYROSCOPE_Z(start_index:end).*180/pi,]);

	title('Gyroscope [deg/s]');
	legend('x','y','z');
	xlabel('time [s]');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	ax(fig_index) = subplot(1,1,1);
	plot(data.DYNAMIC_PRESSURE_TIME_0(start_index:end),...
			data.DYNAMIC_PRESSURE_0(start_index:end).*180/pi);
	hold on;
	plot(data.DYNAMIC_PRESSURE_TIME_1(start_index:end),...
			data.DYNAMIC_PRESSURE_1(start_index:end).*180/pi);
	plot(data.DYNAMIC_PRESSURE_TIME_2(start_index:end),...
			data.DYNAMIC_PRESSURE_2(start_index:end).*180/pi);
	plot(data.DYNAMIC_PRESSURE_TIME_3(start_index:end),...
			data.DYNAMIC_PRESSURE_3(start_index:end).*180/pi);
	plot(data.DYNAMIC_PRESSURE_TIME_4(start_index:end),...
			data.DYNAMIC_PRESSURE_4(start_index:end).*180/pi);
	hold off;

	title('Dynamic Pressure [Pa]');
	legend('0','1','2','3','4');
	xlabel('time [s]');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	ax(fig_index) = subplot(1,1,1);
	plot(data.AIR_TEMPERATURE_TIME(start_index:end),...
			data.AIR_TEMPERATURE(start_index:end));

	title('Air Temperature [deg C]');
	xlabel('time [s]');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	ax(fig_index) = subplot(1,1,1);
	plot(data.HUMIDITY_TIME(start_index:end),...
			data.HUMIDITY(start_index:end));

	title('Humidity [%]');
	xlabel('time [s]');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	ax(fig_index) = subplot(1,1,1);
	plot(data.DATA_PRODUCT_TIME(start_index:end),...
			[data.ALPHA(start_index:end).*180/pi,...
			 data.BETA(start_index:end).*180/pi,]);

	title('Wind Angles [deg]');
	legend('\alpha','\beta');
	xlabel('time [s]');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	ax(fig_index) = subplot(1,1,1);
	plot(data.DATA_PRODUCT_TIME(start_index:end),...
			[data.IAS(start_index:end),...
			 data.TAS(start_index:end),]);

	title('Speeds [m/s]');
	legend('IAS','TAS');
	xlabel('time [s]');


	linkaxes(ax,'x');
