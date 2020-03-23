function terminate_connection
	global comm_port

	if ~isempty(comm_port)
		fclose( comm_port );
		delete( comm_port );
		clear comm_port
	end

	instrreset
