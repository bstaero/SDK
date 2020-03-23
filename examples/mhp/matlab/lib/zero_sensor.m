function zero_sensor(src,~)
global comm_port waiting_for_ack

REQUESTED         = 1;
GYROSCOPE         = 1;
DYNAMIC_PRESSURE  = 3;
UNKNOWN_SENSOR    = 14;

if waiting_for_ack.sensor ~= UNKNOWN_SENSOR
	return
end

set(src,'BackgroundColor',[1,0,0]);
sensor = UNKNOWN_SENSOR;

if strcmp(src.String,'Zero Dynamic')
	sensor = DYNAMIC_PRESSURE;
end

if strcmp(src.String,'Zero Gyros')
	sensor = GYROSCOPE;
end

waiting_for_ack.sensor = sensor;
waiting_for_ack.counter = 0;

packet = make_calibration_packet(sensor,REQUESTED);
fwrite(comm_port,packet);
