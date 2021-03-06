clc
clear 
close all
%%
opt_XY = 3; % generate X and Y

[XActual, YActual] = InitializeXY(opt_XY);

gmean = [0; 0; 0; 0; 0 ;0];

k = 1;

Num = 50;

coeff1 = 0.02;

cov = eye(6, 6);

optPDF = 1;

%%
[A1, B1] = ...
    generateMultiSetsOfAB(k, Num, optPDF, gmean, coeff1*cov, XActual, YActual);

[A_mean, Sig_A] = getMeanCovOfSet(A1);
[B_mean, Sig_B] = getMeanCovOfSet(B1);

S  = []; q = [];
% Explanations:
% S*[x; y] = q where S = [S1 S2]
% x = [x11, x22, x33, x44, x55, x66];
% y = [y11, y22, y33, y44, y55, y66];
% Assume that SigX, SigY are both diagonal matrices
% SigX : diag(x)
% SigY : diag(y)

% Select the columns in S and rows in q corresponding to x,y and z
% index = 1:7:36;
index = [1:6, 8:12, 15:18, 22:24,  29:30, 36]; % a total of 21 elements

S1 = kron(eye(6), eye(6));

for i = 1:size(A_mean, 3)
    
    S2 =  -kron(ad(inv(B_mean(:,:,i))), ad(inv(B_mean(:,:,i)))');
    S = [S; S1(:,index) S2(:,index)]
    
    q1 = vec(Sig_B(:,:,i));
    q2 = -kron(ad(inv(XActual)), ad(inv(XActual))')*vec(Sig_A(:,:,i));
    q = [q; q1 + q2];
    
end

%%
sigma_xy = (S'*S)\S'*q;
% 
% Sig_X = diag(sigma_xy( 1:6))
% Sig_Y = diag(sigma_xy(7:12))
Sig_X = diag(sigma_xy([1,7,12,16,19,21]))
% Sig_Y = diag(sigma_xy(7:12))