function [A, B] =  generateAB(length, optPDF, M, Sig, X, Y)
%% Data generation for AXB = YCZ problem
% Input:
%       length: number of generated data pairs
%       optFix: option for fixing different data streams
%       optPDF: option for generating data using different distributions
%       M:      mean of perturbance in lie algebra
%       Sig:    covariance of perturbance in lie algebra
%       X, Y  : ground truths
% Output:
%       A, B  : 4 x 4 x length or 4 x 4 
%                noise-free data streams with correspondence
%

%% Times of simulation steps
len = length; 
% e = precision;
% digits(64);

%using puma560 to generate tranformation A_inittial, B_initial or C_initial
qz2 = [pi/3, pi/4, pi/3, -pi/4,  pi/4, 0];

dataGenMode = 3;
if dataGenMode == 1
    
    % The following instantiation of puma object is time consuming
    mdl_puma560; % include in the puma560 parameters from "rvctools" toolbox
    B_initial = p560.fkine(qz2);
    
elseif dataGenMode == 2
    
    B_initial = ...
       [ 0.0268   -0.7039   -0.7098    0.0714;
        -0.9536    0.1951   -0.2294   -0.1764;
         0.3000    0.6830   -0.6660    0.2132;
              0         0         0    1.0000]; %  p560.fkine(qz2);

elseif dataGenMode == 3
    
    b = randn(6,1); b = b./norm(b); B_initial = expm(se3_vec(b));

end
    
% This is only physically achievable on multi-robot hand-eye
% calibration
A = zeros(4, 4, len);
B = zeros(4, 4, len);

for m = 1:1:len
    
    if optPDF == 1
        B(:,:,m) = expm(se3_vec(mvg(M, Sig, 1)))*B_initial;
    elseif optPDF == 2
        B(:,:,m) = B_initial*expm(se3_vec(mvg(M, Sig, 1)));
    elseif optPDF == 3
        gmean = [0; 0; 0; 0; 0; 0];
        B(:,:,m) = sensorNoise(B_initial, gmean, Sig(1), 1);
    end

    A(:,:,m) = Y * B(:,:,m) / X;
    
end 

end