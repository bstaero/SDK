function [flag] = checksum_validate(data, index, psize)
l = psize;
sum1 = uint16(0);
sum2 = uint16(0);

for i = 1:l
    sum1 = mod((sum1 + data(i + index - 1)), 255);
    sum2 = mod((sum2 + sum1), 255);
end

if sum1 == 0 && sum2 == 0
    flag = 0;
else
    flag = 1;
end
