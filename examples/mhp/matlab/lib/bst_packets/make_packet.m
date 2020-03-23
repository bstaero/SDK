function [packet] = make_packet(type, information, data_size)
% This function makes a CAN Packet for a given set of data and datatype

% Obtain given headers and IDs for given datatype
d_size = typecast(uint16(data_size), 'uint8');

% Initialize packet
packet = uint8(zeros(1,(data_size + 8)));

% Define Header parameters
packet(1:6) = uint8(['U', '$', type, 0, d_size]);

% Insert data within packet
packet(7:end-2) = uint8(information);

% Compute final packet checksum
packet(end-1:end) = typecast(checksum_create(packet(1:end-2)), 'uint8');
packet = packet';

end
