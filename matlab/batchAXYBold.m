function [ X,Y, X_candidate, Y_candidate, MeanA, MeanB, SigA, SigB ] = batchAXYBold(A, B)

% This solves for the case where A and B have correspondence
X_candidate = zeros(4,4,8);
Y_candidate = zeros(4,4,8);
thetaA = zeros(size(A,3),1);
thetaB = zeros(size(A,3),8);
dA = zeros(size(A,3),1);
dB = zeros(size(A,3),8);
deltaTheta = zeros(size(A,3),8);
deltaD = zeros(size(A,3),8);
normMin = zeros(8,1);

% Reshape A and B for matching the input sies of mex functions
[a1, a2, a3] = size(A);
A_mex = reshape(A, a1, a2*a3);
B_mex = reshape(B, a1, a2*a3);

[ MeanA, SigA ] = distibutionPropsMex_mex(A_mex);
[ MeanB, SigB ] = distibutionPropsMex_mex(B_mex);

[ VA, ~ ] = eig( SigA(1:3,1:3) );
[ VB, ~ ] = eig( SigB(1:3,1:3) );

Q1 = eye(3);
Q2 = [-1 0 0; 0 -1 0; 0 0 1];
Q3 = [-1 0 0; 0 1 0; 0 0 -1];
Q4 = [1 0 0; 0 -1 0; 0 0 -1];

% There are 8 possiblities of Rx

Rx_solved(:,:,1) = VA*Q1*VB';
Rx_solved(:,:,2) = VA*Q2*VB';
Rx_solved(:,:,3) = VA*Q3*VB';
Rx_solved(:,:,4) = VA*Q4*VB';
Rx_solved(:,:,5) = VA*-Q1*VB';
Rx_solved(:,:,6) = VA*-Q2*VB';
Rx_solved(:,:,7) = VA*-Q3*VB';
Rx_solved(:,:,8) = VA*-Q4*VB';

Bnew = zeros(4,4,8);
for i = 1:1:8
    tx_temp = so3_vec(((Rx_solved(:,:,i)'*SigA(1:3,1:3)*Rx_solved(:,:,i))^-1*(SigB(1:3,4:6)-Rx_solved(:,:,i)'*SigA(1:3,4:6)*Rx_solved(:,:,i)))');
    tx = -Rx_solved(:,:,i)*tx_temp;
    X_candidate(:,:,i) = [Rx_solved(:,:,i) tx; [0 0 0] 1];
    Y_candidate(:,:,i) = MeanA * X_candidate(:,:,i) /(MeanB);
end
% use Fouier transformation to find the correspondence between A and X-1YB

for i = 1:1:size(A,3)
    [thetaA(i), ~, dA(i), ~] = param_extract(A(:,:,i));
end

for j = 1:1:8
    for i = 1:1:size(A,3)
        Bnew(:,:,i) = X_candidate(:,:,j)\Y_candidate(:,:,j)*B(:,:,i);
        [thetaB(i,j), ~, dB(i,j), ~] = param_extract(Bnew(:,:,i));
        deltaTheta (i,j) = thetaA(i) - thetaB(i,j);
        deltaD (i,j) = dA(i) - dB(i,j);
    end
    
    normMin(j) = norm(deltaTheta (:,j)) + norm(deltaD(:,j));
end
[~, indexMin] = min(normMin);
X = X_candidate(:,:,indexMin);
Y = Y_candidate(:,:,indexMin);

% [~, Na, ~, ~] = param_extract(MeanA);
% [~, Nb, ~, ~] = param_extract(MeanB);
% na = so3_vec(Na);
% nb = so3_vec(Nb);

% min = inf;
% for i = 1:8
%     if (abs(det(Rx_solved(:,:,i))-1)<0.001) && (norm(na-Rx_solved(1:3,1:3,i)*nb) < min)
%         min = norm(na-Rx_solved(1:3,1:3,i)*nb);
%         Rx = Rx_solved(:,:,i);
%    end
% end

end