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

	figure(1);
	plot(data.STATIC_PRESSURE_TIME(start_index:end),...
			data.STATIC_PRESSURE(start_index:end));

	title('Static Pressure [hPa]');
	xlabel('time [s]');

	figure(2);
	plot(data.MAGNETOMETER_TIME(start_index:end),...
			[data.MAGNETOMETER_X(start_index:end),...
			 data.MAGNETOMETER_Y(start_index:end),...
			 data.MAGNETOMETER_Z(start_index:end),]);

	title('Magnetometer [uT]');
	legend('x','y','z');
	xlabel('time [s]');
