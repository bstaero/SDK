function mhp = parse_mhp_packet(data)

ptr = 1;

mhp.time    = double(typecast(uint8(data(ptr:ptr+3)),'single')); ptr = ptr+4;
mhp.alpha   = double(typecast(uint8(data(ptr:ptr+3)),'single')); ptr = ptr+4;
mhp.beta    = double(typecast(uint8(data(ptr:ptr+3)),'single')); ptr = ptr+4;
mhp.q       = double(typecast(uint8(data(ptr:ptr+3)),'single')); ptr = ptr+4;
mhp.ias     = double(typecast(uint8(data(ptr:ptr+3)),'single')); ptr = ptr+4;
mhp.tas     = double(typecast(uint8(data(ptr:ptr+3)),'single')); ptr = ptr+4;
mhp.wind(1) = double(typecast(uint8(data(ptr:ptr+3)),'single')); ptr = ptr+4;
mhp.wind(2) = double(typecast(uint8(data(ptr:ptr+3)),'single')); ptr = ptr+4;
mhp.wind(3) = double(typecast(uint8(data(ptr:ptr+3)),'single'));
