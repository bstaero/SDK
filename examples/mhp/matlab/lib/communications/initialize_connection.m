function comm_port = initialize_connection(dev)
	global comm_port

	ports = instrfind('Status','open');

	for i=1:length(ports)
		fclose( ports(i) );
	end

	if strcmp(dev,'Select Connection...')
		return
	end

	comm_port = serial(dev,'InputBufferSize', 1024*10,'OutputBufferSize', 1024*10);
	comm_port.BaudRate = 921600;
	comm_port.Terminator = '';
	comm_port.Timeout = 1;

	flushoutput(comm_port)
	flushinput(comm_port)

	try
		fopen(comm_port);
	catch
	  comm_port = [];
	end
