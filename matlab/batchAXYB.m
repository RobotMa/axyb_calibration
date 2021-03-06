function [X, Y] = batchAXYB(A, B, opt, nstd_A, nstd_B)

% Resize A to fit mex functions
n = size(A, 3);

%% ------ Generate A^-1 and B^-1 -------
A_inv = zeros(4, 4, n);
B_inv = zeros(4, 4, n);
for i = 1:n
    A_inv(:,:,i) = inv(A(:,:,i));
    B_inv(:,:,i) = inv(B(:,:,i));
end

%% ------ using probility methods ------
% Option of enabling covariance correction based on the distribution of
% noise
if opt
    case1 = 1;
    case2 = 2;
else
    case1 = 0;
    case2 = 0;
end

[ X1, Y1, MeanA, MeanB, SigA, SigB ] = batchSolveXY(A, B, case1, nstd_A, nstd_B);

% SigAinv = SE3_Ad(MeanA)*SigA*SE3_Ad(MeanA);
% SigBinv = SE3_Ad(MeanB)*SigB*SE3_Ad(MeanB);

[ Y2_inv, X2_inv, MeanA_inv, MeanB_inv, ~, ~] = batchSolveXY(B_inv, A_inv, case2, nstd_B, nstd_A);

m = size(X1,3);
X2 = zeros(4,4,m);
Y2 = zeros(4,4,m);
for i = 1:m
    X2(:,:,i) = inv(X2_inv(:,:,i));
    Y2(:,:,i) = inv(Y2_inv(:,:,i));
end

%%
cost1 = zeros(8,8);
for i = 1:size(X1,3)
    for j = 1:size(Y2,3)
        diff1 = MeanA*X1(:,:,i) - Y2(:,:,j)*MeanB;
        diff2 = MeanB_inv/Y2(:,:,j) - X1(:,:,i)\MeanA_inv;
        cost1(i,j) = norm(diff1(:)) + norm(diff2(:));
    end
end

[~, I1] = min(cost1(:));

cost2 = zeros(8,8);
for i = 1:size(X2,3)
    for j = 1:size(Y1,3)
        diff1 = MeanA*X2(:,:,i) - Y1(:,:,j)*MeanB;
        diff2 = MeanB_inv/Y1(:,:,j) - X2(:,:,i)\MeanA_inv;
        cost2(i,j) = norm(diff1(:)) + norm(diff2(:));
    end
end

[I_row, I_col] = ind2sub(size(cost1), I1);
X = X1(:, :, I_row);
Y = Y2(:, :, I_col);

end