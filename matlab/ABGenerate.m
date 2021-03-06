function [A, B, X, Y] = ABGenerate(length, opt, M, Sig)

%% Times of simulation steps
len = length;
% e = precision;
% digits(64);
%% generate fixed data X Y
if opt == 0
    Y = [-0.99908 -0.03266 0.02786 164.226/1000;
        0.02737  0.01553 0.99950 301.638/1000;
        -0.03308  0.99935 -0.01462 -962.841/1000;
        0.00000   0.00000 0.00000  1.00000;];
    X = inv([-0.97651 -0.09468 -0.19356 9.190/1000;
        0.06362 -0.98493 0.16082 5.397/1000;
        -0.20587 0.14473 0.96782 62.628/1000;
        0.00000 0.00000 0.00000 1.00000;]);
else
    
    % generate random X and Y
    x = randn(6,1); x = x./norm(x); X = expm(se3_vec(x));  %Generate a Random X
    y = randn(6,1); y = y./norm(y); Y = expm(se3_vec(y));  %Generate a Random Y
    
end

% using puma560 to generate tranformation B
qz = [pi/6,pi/3,pi/4,pi/4,-pi/4,0];
mdl_puma560;
Binitial = p560.fkine(qz);

A = zeros(4, 4, len);
B = zeros(4, 4, len);

for m = 1:1:len
    B(:, :, m) = expm(se3_vec(mvg(M, Sig, 1)))*Binitial;
    A(:, :, m) = Y * B(:,:,m) / X;
end

end