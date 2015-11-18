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

point = 10:10:100;

num = 50;

boxplot = false;
lineplot = true;

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
        [ Y2_inv, X2_inv, MeanA_inv, MeanB_inv, ~, ~] = batchAXXB(B_inv, A_inv);
        [ X3, Y3, X_can, Y_can, ~, ~, ~, ~ ] = batchAXYB(A, B);
        
        for i = 1:size(X1,3)
            Xdiff = X1(:,:,i) - X_can(:,:,i);
            if norm(Xdiff(:)) > 0.001
                display('X1 and X3 are not consistent')
            end
        end
        
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
        
        [M1, I1] = min(cost1(:));

        
        cost2 = zeros(8,8);
        for i = 1:size(X2,3)
            for j = 1:size(Y1,3)
                diff1 = MeanA*X2(:,:,i) - Y1(:,:,j)*MeanB;
                diff2 = MeanB_inv/Y1(:,:,j) - X2(:,:,i)\MeanA_inv;
                cost2(i,j) = norm(diff1(:)) + norm(diff2(:));
            end
        end
        
        [M2, I2] = min(cost2(:));
        
        M2 = 100;
        
        if M1 <  M2
            
            [I_row, I_col] = ind2sub(size(cost1), I1);
            X = X1(:, :, I_row);
            Y = Y2(:, :, I_col);
            
        elseif M1 > M2
            
            [I_row, I_col] = ind2sub(size(cost2), I2);
            X = X2(:, :, I_row);
            Y = Y1(:, :, I_col);
            
        end
        
        %% ----- err analysis ------
        if boxplot        
        Err(counter,s,1) = roterror(X, XActual);
        Err(counter,s,2) = roterror(Y, YActual);
        Err(counter,s,3) = roterror(X3, XActual);
        Err(counter,s,4) = roterror(Y3, YActual);
        
        Err(counter,s,5) = tranerror(X, XActual);
        Err(counter,s,6) = tranerror(Y, YActual);
        Err(counter,s,7) = tranerror(X3, XActual);
        Err(counter,s,8) = tranerror(Y3, YActual);

        elseif lineplot
        Err(counter,1,s) = roterror(X, XActual);
        Err(counter,2,s) = roterror(Y, YActual);
        Err(counter,3,s) = roterror(X3, XActual);
        Err(counter,4,s) = roterror(Y3, YActual);
        
        Err(counter,5,s) = tranerror(X, XActual);
        Err(counter,6,s) = tranerror(Y, YActual);
        Err(counter,7,s) = tranerror(X3, XActual);
        Err(counter,8,s) = tranerror(Y3, YActual);
        end
        %% ------ plot X and Y ------
        try
            figure(5);
            trplot(X(:,:),'color','b');
            axis auto
            hold on
            
            figure(6);
            trplot(Y(:,:),'color','b');
            axis auto
            hold on
        catch
            X
        end
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
if boxplot
    
    figure
    boxplot(Err(:,:,1)', point)
    figure
    boxplot(Err(:,:,3)', point)
    figure
    boxplot(Err(:,:,2)', point)
    figure
    boxplot(Err(:,:,4)', point)
    
    figure
    boxplot(Err(:,:,5)', point)
    figure
    boxplot(Err(:,:,7)', point)
    figure
    boxplot(Err(:,:,6)', point)
    figure
    boxplot(Err(:,:,8)', point)
    
elseif lineplot
    
    figure
    plot(point, Err_Avg(:,[1,3]))
    legend('rot_{Xnew}','rot_X')

    figure
    plot(point, Err_Avg(:,[2,4]))
    legend('rot_{Ynew}','rot_Y')
    
    figure
    plot(point, Err_Avg(:,[5,7]))
    legend('tran_{Xnew}','tran_X')
    
    figure
    plot(point, Err_Avg(:,[6,8]))
    legend('tran_{Ynew}','tran_Y')
    
end
