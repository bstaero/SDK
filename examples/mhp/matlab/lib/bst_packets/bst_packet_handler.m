function [PacketData, PacketCount, PacketType, buff] = bst_packet_handler(data)

% Get Data
data_size = length(data);

% Init values
PacketCount = 0;
PacketData = {};
PacketType = [];

index = 1;
exit = 0;

% Start Parsing
while exit ~= 1

		if index+1 > data_size
			break;
		end
    
    % Start byte
    if data(index) == 'U' && data(index+1) == '$'
				%fprintf('HDR\n');
        
        % do we have enough bytes
        if( data_size - index < 5 )
            exit = 1;
						break;
        end
        
				% Read the packet type indentifier
				type = data(index + 2);

				% Read data size
				d_size = data(index + 4) + 256*data(index + 5);
        
        % packet size
        psize = d_size + 8;
        
        % do we have enough data
        if (index + psize - 1) <= data_size
            
            % do we have a valid packet
            err = checksum_validate(data, index, psize);
            
            if( err == 0 )
                %fprintf('PACKET\n');
                
                data_start = index + 6;
                data_end   = index + 5 + d_size;
                
                PacketCount = PacketCount + 1;
                PacketData{PacketCount} = data(data_start:data_end);
                PacketType(PacketCount) = type;
                
                % move down data
                index = index + psize;
            else
                % drop byte
                %fprintf('BAD CHECKSUM\n');
                index = index + 1;
            end
            
        else
            %fprintf('NOT ENOUGH BYTES\n');
						exit = 1;
            break
        end
    else
        %fprintf('BAD START BYTE\n');
        index = index + 1;
    end

end

if(index == data_size)
    buff = [];
else
    buff = data(index:end);
end
