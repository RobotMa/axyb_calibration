clear all
close all
clc
% [A,B] =  AABB(100);
% load invariantrandom.mat
% global xxx yyy bbb
counter = 0;

Mean=[0;0;0;0;0;0];

Cov = 0.3*eye(6,6); 

point = 10:10:100;
for k = point
%     waitbar((k-point(1))/(point(3)-point(1)));
    
%  if k==92
%      continue;
%  end
%% --Trajectory Generation-------------------------
 [A, B, XActual, YActual] =  ABGenerate(k, 0, Mean, Cov);
% [A,B,XActual,YActual] =  ABGen(k,0);

%% add noise
gmean = [0;0;0;0;0;0];	%Gaussian Noise Mean
nstd = 0.05;  %Gaussian Noise standard deviation Range

% B = sensorNoise(B, gmean, nstd, 1);
%% Generate A^-1 and B^-1
A_inv = zeros(4,4,k);
B_inv = zeros(4,4,k);
for i = 1:k
    A_inv(:,:,i) = inv(A(:,:,i));
    B_inv(:,:,i) = inv(B(:,:,i));
end

%% --using probility methods------------------------
% [XX(:,:,k),YY(:,:,k),err(k,:)]= haiyuan(A,B,X,Y,k,0.1);
[ X1, Y1, MeanA, MeanB, SigA, SigB ] = batchAXXB(A, B);
[ Y2_inv, X2_inv, ~, ~, ~, ~] = batchAXXB(B_inv, A_inv);
[ X3, Y3, ~, ~, ~, ~ ] = batchAXYB(A, B);

% [XX(:,:,k)] = AXYB_LieGroup2( A, B );
% Y(4,1:3) = [0 0 0];

m = size(X1,3);
X2 = zeros(4,4,m);
Y2 = zeros(4,4,m);
for i = 1:m
    X2(:,:,i) = inv(X2_inv(:,:,i));
    Y2(:,:,i) = inv(Y2_inv(:,:,i));
end

cost = zeros(8,8);
for i = 1:size(X1,3)
    for j = 1:size(Y2,3)
        cost(i,j) = norm( MeanA*X1(:,:,i) - Y2(:,:,j)*MeanB); 
    end
end

[M,I] = min(cost(:));
[I_row, I_col] = ind2sub(size(cost),I);

X = X1(:,:,I_row);
Y = Y2(:,:,I_col);
%% err analysis
counter = counter + 1;
num(counter) = k;
Err(counter,1) = roterror(X, XActual);
Err(counter,2) = roterror(Y, YActual);
Err(counter,3) = roterror(X3, XActual);
Err(counter,4) = roterror(Y3, YActual);

Err(counter,5) = tranerror(X, XActual);
Err(counter,6) = tranerror(Y, YActual);
Err(counter,7) = tranerror(X3, XActual);
Err(counter,8) = tranerror(Y3, YActual);

%% plot 
% for j = 1:1:k
%     figure(3);
%     trplot(A(:,:,j),'color','r');
%     axis auto
%     hold on
%     figure(4);
%     trplot(B(:,:,j),'color','b');
%     axis auto
%     hold on
% end
% figure(3);
% legend('Generated A');
% figure(4);
% legend('Generated B');

%% --plot X and Y-------------------------------------------
figure(5);
trplot(X(:,:),'color','b');
axis auto
hold on
figure(6);
trplot(Y(:,:),'color','b');
axis auto
hold on
% for n = 1:1:k
% 	figure(3);
%     trplot(A(:,:,n),'color','b');
%     axis auto
%     hold on
% end
% for n = 1:1:k
% 	figure(4);
%     trplot(B(:,:,n),'color','b');
%     axis auto
%     hold on
% end
end

%%

figure(5)
trplot(XActual(:,:),'color','r');
axis auto
% legend('solved and actual X');
figure(6)
trplot(YActual(:,:),'color','r');
axis auto
% legend('solved and actual Y');

%%
figure
plot(Err, point)
