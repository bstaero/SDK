function [chksum] = checksum_create(packet)
% Computes a fletcher16 checksum for an array of uint8 data
length = max(size(packet));
sum1 = uint16(0);
sum2 = uint16(0);

for i = 1:length
    sum1 = mod((sum1 + typecast([packet(i), 0], 'uint16')), 255);
    sum2 = mod((sum2 + sum1), 255);
end

s0 = 255 - mod((sum1 + sum2), 255);
s1 = 255 - mod((sum1 + s0), 255);

s1 = bitand(bitshift(s1, 8), 65280);
chksum = bitor(s1, s0);
end

