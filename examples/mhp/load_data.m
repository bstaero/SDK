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
function data = load_data (filename)

	fid = fopen(filename);
	if fid < 0
	  fprintf(1,'Could not open file %s\n for reading.',filename)
		return
	end

	label_str = fgetl(fid);
	fclose(fid);

	label_str = label_str(2:end);
	labels = strsplit(label_str,',');

	content = load(filename);
	
	for i=1:length(labels)
	  fprintf(1,'creating %s\n',labels{i});
		eval(['data.' labels{i} ' = content(:,' num2str(i) ');']);
	end
