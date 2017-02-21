function [ mean_X, mean_Y, Sig_X, Sig_Y ] = probAXYB( A, B )

[A_mean, Sig_A] = getMeanCovOfSet(A);
[B_mean, Sig_B] = getMeanCovOfSet(B);

[n1, n2, n3] = size(A_mean);

A = reshape(A_mean, n1, n2*n3);
B = reshape(B_mean, n1, n2*n3);

[ mean_X, mean_Y ] = shah( A, B );

S  = []; q = [];
% Explanations:
% S*[x; y] = q where S = [S1 S2]
% x = [x11, x22, x33, x44, x55, x66];
% y = [y11, y22, y33, y44, y55, y66];
% Assume that SigX, SigY are both diagonal matrices
% SigX : diag(x)
% SigY : diag(y)

% Select the columns in S and rows in q corresponding to x,y and z
index = 1:7:36;
S1 = kron(eye(6), eye(6));
 
for i = 1:size(A_mean, 3)
    
    S2 =  -kron(ad(inv(B_mean(:,:,i))), ad(inv(B_mean(:,:,i)))');
    S = [S; S1(:,index) S2(:,index)];
    
    q1 = vec(Sig_B(:,:,i));
    q2 = -kron(ad(inv(mean_X)), ad(inv(mean_X))')*vec(Sig_A(:,:,i));
    q = [q; q1 + q2];
 
end
    
sigma_xy = (S'*S)\S'*q;

Sig_X = diag(sigma_xy( 1:6));
Sig_Y = diag(sigma_xy(7:12));


end