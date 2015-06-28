function [A,B] = dataTransAB(AA,BB)
len = size(AA,3);
A = zeros(4,4*len);
B = zeros(4,4*len);
for i = 1:1:len
    A(:,(4*i - 3):(4*i)) = AA(:,:,i);
    B(:,(4*i - 3):(4*i)) = BB(:,:,i);
end

end