function plot_mhp_data(data, figure);

	global t_3view acc_f acc_counter t_dp t_pth t_imu t_est s_3view h_3view s_dp1 s_dp2 s_dp3 s_dp4 s_dp5 s_pth1 s_pth2 s_pth3 s_imu1 s_imu2 s_imu3 s_est1 s_est2 s_est3

	%% Plotting rates
	% 3 View
	dt_3view = 0.2;

	% Dynamic Pressure
	dt_dp = .1;

	% PTH
	dt_pth = .1;

	% IMU
	dt_imu = .1;

	% Estimate of alpha, beta, and IAS
	dt_est = .1;

	% 3D
	if (toc - t_3view > dt_3view)
		t_3view = toc;

		if acc_counter == 0;
			roll = 0; pitch = 0;
		else
			acc_f = acc_f/acc_counter;
			roll = atan2(-acc_f(2), -acc_f(3));
			pitch = atan2(acc_f(1), -acc_f(3));
		end

		rotate_3view(s_3view, h_3view,roll,-pitch,0);
		str = sprintf('Roll = %.1f^o, Pitch = %.1f^o                               ',roll*180/pi,pitch*180/pi);
		title(str);
		acc_f = [0 0 0];
		acc_counter = 0;
	else
		acc_f = acc_f + data.accelerometer(end,:);
		acc_counter = acc_counter+1;
	end

	% Dynamic Pressure
	if (toc - t_dp > dt_dp)
		t_dp = toc;
		plot_data(data.time, data.dynamic_pressure(:,1), s_dp1,'k');
		plot_data(data.time, data.dynamic_pressure(:,2), s_dp2,'k');
		plot_data(data.time, data.dynamic_pressure(:,3), s_dp3,'k');
		plot_data(data.time, data.dynamic_pressure(:,4), s_dp4,'k');
		plot_data(data.time, data.dynamic_pressure(:,5), s_dp5,'k');
	end

		% PTH
	if (toc - t_pth > dt_pth)
		t_pth = toc;
		plot_data(data.time, data.static_pressure./1000, s_pth1,'k');
		ylim([data.static_pressure(end)./1000-0.1,...
				  data.static_pressure(end)./1000+0.1]);
		plot_data(data.time, data.air_temperature, s_pth2,'k');
		plot_data(data.time, data.humidity, s_pth3,'k');
	end

		% IMU
	if (toc - t_imu > dt_imu)
		t_imu = toc;
		col{1} = '-k';
		col{2} = '-b';
		col{3} = '-r';
		plot_data3(data.time, data.accelerometer.*9.8, s_imu1,col);
		plot_data3(data.time, data.gyroscope.*180/pi, s_imu2,col);
		plot_data3(data.time, data.magnetometer, s_imu3,col);
	end

		% Estimate of IAS, alpha, and beta
	if (toc - t_est > dt_est)
		t_est = toc;
		d1 = data.dynamic_pressure(:,1);
		d2 = data.dynamic_pressure(:,2);
		d3 = data.dynamic_pressure(:,3);
		d4 = data.dynamic_pressure(:,4);
		d5 = data.dynamic_pressure(:,5);

		alpha = data.alpha.*180/pi;
		beta = data.beta.*180/pi;
		q = data.q;
		
		col{1} = '-k';
		col{2} = '-b';
		plot_data2(data.time, [data.ias',data.tas'], s_est3, col);

		plot_data(data.time, alpha, s_est2,'k');
		plot_data(data.time, beta, s_est1,'k');
	end
end

function plot_data2(t,val,s,col)
  subplot(s);
  set(gca,'NextPlot','replacechildren');
  start = 1;
  if length(t) > 500
    start = length(t)-500;
  end
	plot(t(start:end), val(start:end,1),col{1}, t(start:end), val(start:end,2),col{2});
end

function plot_data3(t,val,s,col)
  subplot(s);
  set(gca,'NextPlot','replacechildren');
  start = 1;
  if length(t) > 500
    start = length(t)-500;
  end
	plot(t(start:end), val(start:end,1),col{1}, t(start:end), val(start:end,2),col{2}, t(start:end), val(start:end,3),col{3});
end

function plot_data(t,val,s,col)
  subplot(s);
  set(gca,'NextPlot','replacechildren');
  start = 1;
  if length(t) > 500
    start = length(t)-500;
  end
	if(length(t(start:end)) == length(val(start:end)))
		plot(t(start:end), val(start:end),'-','color',col);
	end
end

function rotate_3view(s,t,roll,pitch,yaw);
  subplot(s);
  Ry = makehgtform('yrotate',pitch);
  Rx = makehgtform('xrotate',roll);
  R = Ry*Rx;
  % set the transform Matrix property
  set(t,'Matrix',R)
  %drawnow
end
