clear all
close all
clc

%% Edited by Qianli
counter = 0;

% Require rvctool Matlab toolbox but not the "common" folder inside
addpath('~/Dropbox/2014Summer/Robotics Research/rvctools/robot')
addpath('~/Dropbox/2014Summer/Robotics Research/rvctools/common')
addpath('~/Dropbox/2015Spring/AXXB Journal/Batch_AXXB/Matlab_AXXB/codegen')
addpath('~/Dropbox/2015Spring/AXXB Journal/Batch_AXXB/Matlab_AXXB')
addpath('~/Dropbox/2014Summer/Robotics Research/kinematics/kinematics/screws')
addpath('~/Dropbox/2014Summer/Robotics Research/kinematics/kinematics/util')

Mean=[0; 0; 0; 0; 0 ;0];

Cov = 0.3*eye(6,6);

point = 10:20:100;

num = 30;

for k = point
    
    counter = counter + 1;
    for s = 1:num
        %% ------ Trajectory Generation --------
        [A, B, XActual, YActual] =  ABGenerate(k, 0, Mean, Cov);
        
        % Resize A to fit mex functions
        [a1,a2,a3] = size(A);
        
        %% ------- add noise -------
        gmean = [0; 0; 0; 0; 0; 0];	% Gaussian Noise Mean
        nstd = 0.05;                % Gaussian Noise standard deviation Range
        
        % B = sensorNoise(B, gmean, nstd, 1);
        
        %% ------ Generate A^-1 and B^-1 -------
        A_inv = zeros(4,4,k);
        B_inv = zeros(4,4,k);
        for i = 1:k
            A_inv(:,:,i) = inv(A(:,:,i));
            B_inv(:,:,i) = inv(B(:,:,i));
        end
        
        %% ------ using probility methods ------
        [ X1, Y1, MeanA, MeanB, SigA, SigB ] = batchAXXB(A, B);
        [ Y2_inv, X2_inv, ~, ~, ~, ~] = batchAXXB(B_inv, A_inv);
        [ X3, Y3, ~, ~, ~, ~ ] = batchAXYB(A, B);
        
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
                diff = (MeanA*X1(:,:,i)) - Y1(:,:,i)*MeanB
                diff = (MeanA*X1(:,:,i))\Y2(:,:,j)*MeanB;
                cost1(i,j) = norm( diff );
            end
        end
        
        [M1, I1] = min(cost1(:));
        
        cost2 = zeros(8,8);
        for i = 1:size(X2,3)
            for j = 1:size(Y1,3)
                diff = (MeanA*X2(:,:,i)) - Y2(:,:,i)*MeanB
                diff = (MeanA*X2(:,:,i))\Y1(:,:,j)*MeanB;
                cost2(i,j) = norm( diff );
            end
        end
        
        [M2, I2] = min(cost2(:));
        
        if M1 <  M2
            [I_row, I_col] = ind2sub(size(cost1), I1);
            
            X = X1(:, :, I_row);
            Y = Y2(:, :, I_col);
        else
            [I_row, I_col] = ind2sub(size(cost2), I2);
            
            X = X2(:, :, I_row);
            Y = Y1(:, :, I_col);
        end
        
        %% ----- err analysis ------
        Err(counter,1,s) = roterror(X, XActual);
        Err(counter,2,s) = roterror(Y, YActual);
        Err(counter,3,s) = roterror(X3, XActual);
        Err(counter,4,s) = roterror(Y3, YActual);
        
        Err(counter,5,s) = tranerror(X, XActual);
        Err(counter,6,s) = tranerror(Y, YActual);
        Err(counter,7,s) = tranerror(X3, XActual);
        Err(counter,8,s) = tranerror(Y3, YActual);
        
        %% ------ plot X and Y ------
        figure(5);
        trplot(X(:,:),'color','b');
        axis auto
        hold on
        
        figure(6);
        trplot(Y(:,:),'color','b');
        axis auto
        hold on
    end
    
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
Err_Avg = sum(Err,3)/num;

%%
figure
plot(point, Err_Avg(:,1:4))
legend('rot_{Xnew}','rot_{Ynew}','rot_X','rot_Y')

figure
plot(point, Err_Avg(:,5:8))
legend('tran_{Xnew}','tran_{Ynew}','tran_X','tran_Y')

%%
figure
plot(point, Err_Avg(:,[2,4]))
legend('rot_{Ynew}','rot_Y')

figure
plot(point, Err_Avg(:,[6,8]))
legend('tran_{Ynew}','tran_Y')

