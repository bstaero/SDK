function calibrate_data = parse_calibrate_packet(data)

calibrate_data.sensor = data(1);
calibrate_data.state = data(2);
