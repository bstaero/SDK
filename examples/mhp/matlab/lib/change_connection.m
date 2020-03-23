function change_connection(src,~)

	global comm_port

	initialize_connection(src.String{src.Value});
