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

	start_index = 10;
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
			data.DYNAMIC_PRESSURE_0(start_index:end));
	hold on;
	plot(data.DYNAMIC_PRESSURE_TIME_1(start_index:end),...
			data.DYNAMIC_PRESSURE_1(start_index:end));
	plot(data.DYNAMIC_PRESSURE_TIME_2(start_index:end),...
			data.DYNAMIC_PRESSURE_2(start_index:end));
	plot(data.DYNAMIC_PRESSURE_TIME_3(start_index:end),...
			data.DYNAMIC_PRESSURE_3(start_index:end));
	plot(data.DYNAMIC_PRESSURE_TIME_4(start_index:end),...
			data.DYNAMIC_PRESSURE_4(start_index:end));
	if isfield(data,'DYNAMIC_PRESSURE_TIME_5')
		plot(data.DYNAMIC_PRESSURE_TIME_5(start_index:end),...
				data.DYNAMIC_PRESSURE_5(start_index:end));
		plot(data.DYNAMIC_PRESSURE_TIME_6(start_index:end),...
				data.DYNAMIC_PRESSURE_6(start_index:end));
		plot(data.DYNAMIC_PRESSURE_TIME_7(start_index:end),...
				data.DYNAMIC_PRESSURE_7(start_index:end));
		plot(data.DYNAMIC_PRESSURE_TIME_8(start_index:end),...
				data.DYNAMIC_PRESSURE_8(start_index:end));
	end
	hold off;

	title('Dynamic Pressure [Pa]');
	if isfield(data,'DYNAMIC_PRESSURE_TIME_5')
		legend('0','1','2','3','4','5','6','7','8');
	else
		legend('0','1','2','3','4');
	end
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
	%plot(data.DATA_PRODUCT_TIME(start_index:end),...
			%[smooth(data.ALPHA(start_index:end).*180/pi,0.005),...
			 %smooth(data.BETA(start_index:end).*180/pi,0.005),]);

	title('Wind Angles [deg]');
	legend('\alpha','\beta');
	xlabel('time [s]');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	ax(fig_index) = subplot(1,1,1);
	%plot(data.DATA_PRODUCT_TIME(start_index:end),...
			%[data.IAS(start_index:end),...
			 %data.TAS(start_index:end),]);
	plot(data.DATA_PRODUCT_TIME(start_index:end),...
			[data.IAS(start_index:end),...
			 data.TAS(start_index:end),]);

	title('Speeds [m/s]');
	legend('IAS','TAS');
	xlabel('time [s]');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;
	[N,edges] = histcounts(data.DYNAMIC_PRESSURE_0, 100);
	edges = edges(2:end) - (edges(2)-edges(1))/2; plot(edges, N);
	hold on;
	[N,edges] = histcounts(data.DYNAMIC_PRESSURE_1, 100);
	edges = edges(2:end) - (edges(2)-edges(1))/2; plot(edges, N);
	[N,edges] = histcounts(data.DYNAMIC_PRESSURE_2, 100);
	edges = edges(2:end) - (edges(2)-edges(1))/2; plot(edges, N);
	[N,edges] = histcounts(data.DYNAMIC_PRESSURE_3, 100);
	edges = edges(2:end) - (edges(2)-edges(1))/2; plot(edges, N);
	[N,edges] = histcounts(data.DYNAMIC_PRESSURE_4, 100);
	edges = edges(2:end) - (edges(2)-edges(1))/2; plot(edges, N);
	if isfield(data,'DYNAMIC_PRESSURE_TIME_5')
		[N,edges] = histcounts(data.DYNAMIC_PRESSURE_5, 100);
		edges = edges(2:end) - (edges(2)-edges(1))/2; plot(edges, N);
		[N,edges] = histcounts(data.DYNAMIC_PRESSURE_6, 100);
		edges = edges(2:end) - (edges(2)-edges(1))/2; plot(edges, N);
		[N,edges] = histcounts(data.DYNAMIC_PRESSURE_7, 100);
		edges = edges(2:end) - (edges(2)-edges(1))/2; plot(edges, N);
		[N,edges] = histcounts(data.DYNAMIC_PRESSURE_8, 100);
		edges = edges(2:end) - (edges(2)-edges(1))/2; plot(edges, N);
	end
	hold off;

	xlim([-1,1])  

	if isfield(data,'DYNAMIC_PRESSURE_TIME_5')
		legend('Center','1','2','3','4','5','6','7','8');
	else
		legend('Center','1','2','3','4');
	end
	xlabel('Pressure [Pa]');
	ylabel('Count');

	linkaxes(ax,'x');

	%----- [ ] -----%
	figure(fig_index); fig_index = fig_index+1;

	ind=find(data.PDOP > 0.0 & data.PDOP < 3.0);
	geoplot(data.LATTIUDE(ind), data.LONGITUDE(ind),'.');
