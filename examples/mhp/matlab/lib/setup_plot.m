function [fig] = setup_plot();

	global t_3view acc_f acc_counter t_dp t_pth t_imu t_est s_3view h_3view s_dp1 s_dp2 s_dp3 s_dp4 s_dp5 s_pth1 s_pth2 s_pth3 s_imu1 s_imu2 s_imu3 s_est1 s_est2 s_est3 dyn_btn gyro_btn

	% CF Tube
	[z1 y1 x1] = cylinder(1);
	x1 = x1*50;

	% Orange Tip
	t = [ones(1,50), cos(0:0.1:pi/2)];
	[z2, y2, x2] = cylinder(t);
	x2 = 3.*x2+50;

	% Base
	[x3, y3, z3] = cylinder(1);
	x3 = x3*10;
	y3 = y3*10;
	z3 = z3*5 + 6;

	% Dome
	t = 0:.1:1;
	[x4, y4, z4] = cylinder(sqrt(t));
	x4 = x4*10;
	y4 = y4*10;
	z4 = z4*10-4;

	% Plot
	figure; hold on; grid on; axis equal;
	view(3);

	h(1) = surf(x1,y1,z1);
	h(1).FaceColor = 'k';
	h(2) = surf(x2,y2,z2);
	h(2).FaceColor = [.91, .41, .17];
	h(2).EdgeColor = [.91, .41, .17];
	h(3) = mesh(x3,y3,z3);
	h(3).FaceColor = .8*[1 1 1];
	h(3).EdgeColor = .7*[1 1 1];
	h(4) = surf(x4,y4,z4);
	h(4).FaceColor = .8*[1 1 1];
	h(4).EdgeColor = .7*[1 1 1];
	ax = gca;
	set(ax,'XTickLabel',[]);
	set(ax,'YTickLabel',[]);
	set(ax,'ZTickLabel',[]);

	t = hgtransform('Parent',ax);
	set(h,'Parent',t)

	%% Setup the plotting axes
	close all
	fig = figure;
	fig.Position(2) = fig.Position(2) - 400;
	fig.Position(3) = 1200;
	fig.Position(4) = 800;
	% Create sections
	axes('Position',[0, 0, 0.350, 1],'xtick',[],'ytick',[],'box','on','handlevisibility','off')
	axes('Position',[.350, 0, 0.34, 1],'xtick',[],'ytick',[],'box','on','handlevisibility','off')
	axes('Position',[.350, 0, 0.34, .535],'xtick',[],'ytick',[],'box','on','handlevisibility','off')
	axes('Position',[.690, 0, 0.310, .535],'xtick',[],'ytick',[],'box','on','handlevisibility','off')
	axes('Position',[.690, .535, 0.310, .465],'xtick',[],'ytick',[],'box','on','handlevisibility','off')

	% Orientation
	s_3view = subplot('Position',[.7 .55 .3 .4]);
	h_3view = setup_3view;

	% Plot limits
	dp_limits = [-200 200];
	rh_limits = [-5 105];
	temp_limits = [15 30];
	press_limits = [82, 85];
	acc_limits = [-1 1]*9.8;
	gyro_limits = [-180 180];
	mag_limits = [-50 50];
	ab_limits = [-20 20];
	ias_limits = [-5 25];

	% Dynamic sensors
	vstart = .10; vstep = .18; i_step = 0;
	s_dp5 = subplot('Position',[.052,vstart+vstep*i_step,.29,.15]); i_step=i_step+1; xlabel('Time [s]'); grid on; hold on; ylim(dp_limits);
	s_dp4 = subplot('Position',[.052,vstart+vstep*i_step,.29,.15]); i_step=i_step+1; set(gca,'XTickLabel',[]); grid on; hold on; ylim(dp_limits);
	s_dp3 = subplot('Position',[.052,vstart+vstep*i_step,.29,.15]); i_step=i_step+1; set(gca,'XTickLabel',[]); ylabel('Pa'); grid on; hold on; ylim(dp_limits);
	s_dp2 = subplot('Position',[.052,vstart+vstep*i_step,.29,.15]); i_step=i_step+1; set(gca,'XTickLabel',[]); grid on; hold on; ylim(dp_limits);
	s_dp1 = subplot('Position',[.052,vstart+vstep*i_step,.29,.15]); i_step=i_step+1; set(gca,'XTickLabel',[]); title('Dynamic Pressure'); grid on; hold on; ylim(dp_limits);

	vstart = .56; vstep = .14; i_step = 0;
	% PTH
	s_pth3 = subplot('Position',[.4, vstart + vstep*i_step, .28, .12]); i_step = i_step+1; set(gca,'XTickLabel',[]); ylabel('RH [%]'); grid on; hold on; ylim(rh_limits);
	s_pth2 = subplot('Position',[.4, vstart + vstep*i_step, .28, .12]); i_step = i_step+1; set(gca,'XTickLabel',[]); ylabel('Temp [^oC]'); grid on; hold on; ylim(temp_limits);
	s_pth1 = subplot('Position',[.4, vstart + vstep*i_step, .28, .12]); i_step = i_step+1; set(gca,'XTickLabel',[]); ylabel('Pressure [kPa]'); title('PTH'); grid on; hold on; ylim(press_limits);

	vstart = .1; vstep = .14; i_step = 0;
	% PTH
	s_imu3 = subplot('Position',[.4, vstart + vstep*i_step, .28, .12]); i_step = i_step+1; xlabel('Time [s]');       ylabel('Mag []'); grid on; hold on; ylim(mag_limits);
	s_imu2 = subplot('Position',[.4, vstart + vstep*i_step, .28, .12]); i_step = i_step+1; set(gca,'XTickLabel',[]); ylabel('Gyro [^o/s]'); grid on; hold on; ylim(gyro_limits);
	s_imu1 = subplot('Position',[.4, vstart + vstep*i_step, .28, .12]); i_step = i_step+1; set(gca,'XTickLabel',[]); ylabel('Acc [m/s^2]'); title('IMU'); grid on; hold on; ylim(acc_limits);

	vstart = .1; vstep = .14; i_step = 0;
	% PTH
	s_est1 = subplot('Position',[.74, vstart + vstep*i_step, .25, .12]); i_step = i_step+1; xlabel('Time [s]');       ylabel('\beta [^o]'); grid on; hold on; ylim(ab_limits);
	s_est2 = subplot('Position',[.74, vstart + vstep*i_step, .25, .12]); i_step = i_step+1; set(gca,'XTickLabel',[]); ylabel('\alpha [^o]'); grid on; hold on; ylim(ab_limits);
	s_est3 = subplot('Position',[.74, vstart + vstep*i_step, .25, .12]); i_step = i_step+1; set(gca,'XTickLabel',[]); ylabel('IAS [m/s]'); title('Measurements'); grid on; hold on; ylim(ias_limits);

	%% Control Buttons
	btn_pad = 2; btn_height=30;btn_width = 60;
	pause_btn = uicontrol('Style', 'togglebutton', 'String', 'Pause','Position', [fig.Position(3)-btn_width-btn_pad, btn_pad, btn_width, btn_height],'Min',0,'Max',1,'Value',1);
	btn_width = 100;
	dyn_btn = uicontrol('Style', 'togglebutton', 'String', 'Zero Dynamic','Position', [fig.Position(3)*.264, btn_pad, btn_width, btn_height],'Min',0,'Max',1,'Value',1, 'Callback', @zero_sensor, 'BackgroundColor',[0.95 0.95 0.95]);
	gyro_btn = uicontrol('Style', 'togglebutton', 'String', 'Zero Gyros','Position', [fig.Position(3)*.355, btn_pad btn_width btn_height],'Min',0,'Max',1,'Value',1, 'Callback', @zero_sensor, 'BackgroundColor', [0.95 0.95 0.95]);
	btn_width = 110;
	mags_btn = uicontrol('Style', 'togglebutton', 'String', 'Calibrate Mags','Position', [fig.Position(3)*.596, btn_pad btn_width btn_height],'Min',0,'Max',1,'Value',1);
	btn_width = 130;
	file_btn = uicontrol('Style', 'togglebutton', 'String', 'Write to File','Position', [fig.Position(3)*.8, btn_pad btn_width btn_height],'Min',0,'Max',1,'Value',1);

	list = {'Select Connection...'};
	%ports = instrlist; %FIXME -- seems to only work in 2018
	%if ~isempty(ports)
	%	list = {list{1}, ports.Port}
	%end

	list = {list{1},...
		'/dev/ttyUSB0',...
		'/dev/ttyUSB1',...
		'/dev/ttyUSB2',...
		'/dev/ttyUSB3'};

	comm_port = uicontrol('Style', 'popup',...
			'String', list,...
			'Position', [10 -22 160 50],...
			'Callback', @change_connection);
	close_btn = uicontrol('Style', 'togglebutton',...
			'String', 'Close',...
			'Position', [180, btn_pad 60 btn_height],...
			'Min',0,...
			'Max',1,...
			'Value',1,...
			'Callback', 'close_window');

	axes('Position',[0.865 0.925 .2*0.77 .088*0.77]);
	imshow('lib/logo.png');

	%% Plotting rates
	tic;
	% 3 View
	t_3view = toc;
	acc_f = [0 0 0];
	acc_counter = 0;

	% Dynamic Pressure
	t_dp = toc;

	% PTH
	t_pth = toc;

	% IMU
	t_imu = toc;

	% Estimate of alpha, beta, and IAS
	t_est = toc;

  drawnow
end


function t = setup_3view;
  % CF Tube
  [z1 y1 x1] = cylinder(1);
  x1 = x1*50;
  
  % Orange Tip
  t = [ones(1,50), cos(0:0.1:pi/2)];
  [z2, y2, x2] = cylinder(t);
  x2 = 3.*x2+50;
  
  % Base
  [x3, y3, z3] = cylinder(1);
  x3 = x3*10;
  y3 = y3*10;
  z3 = -z3*5 - 6;
  
  % Dome
  t = 0:.1:1;
  [x4, y4, z4] = cylinder(sqrt(t));
  x4 = x4*10;
  y4 = y4*10;
  z4 = z4*-10+4;
  
  % Plot
  hold on; grid on; axis equal;
  view(3);
  
  h(1) = surf(x1,y1,z1);
  h(1).FaceColor = 'k';
  h(2) = surf(x2,y2,z2);
  h(2).FaceColor = [.91, .41, .17];
  h(2).EdgeColor = [.91, .41, .17];
  h(3) = mesh(x3,y3,z3);
  h(3).FaceColor = .8*[1 1 1];
  h(3).EdgeColor = .7*[1 1 1];
  h(4) = surf(x4,y4,z4);
  h(4).FaceColor = .8*[1 1 1];
  h(4).EdgeColor = .7*[1 1 1];
  ax = gca;
  set(ax,'XTickLabel',[]);
  set(ax,'YTickLabel',[]);
  set(ax,'ZTickLabel',[]);
  
  t = hgtransform('Parent',ax);
  set(h,'Parent',t)
end
