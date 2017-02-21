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

point = 0:10:100;

num = 100; % number of simulations

Num  = 50; % number of data

boxplot = false;
lineplot = true;

for k = point
    
    counter = counter + 1;
    
    for s = 1:num
        %% ------ Trajectory Generation --------
        [A, B, XActual, YActual] =  ABGenerate(Num, 0, Mean, Cov);
        
        % Scramble data streams up to a certain percentage 
        PA = (1:size(A,3));
        PB = (1:size(B,3));
        
        for i = 1:length(PA)
            
            if rand <= 0.01*k
                index = randi(Num,1);
                PA([i index]) = PA([index i]);
            end
            
        end
        A_perm = A(:,:,PA);
        B_perm = B(:,:,PB);
        
        %% ------- add noise -------
%         gmean = [0; 0; 0; 0; 0; 0];	% Gaussian Noise Mean
%         nstd = 0.05;                % Gaussian Noise standard deviation Range
        
%         B = sensorNoise(B, gmean, nstd, 1);
        
        %% ------ Generate A^-1 and B^-1 -------
        A_inv = zeros(4, 4, Num);
        B_inv = zeros(4, 4, Num);
        for i = 1:Num
            A_inv(:,:,i) = inv(A_perm(:,:,i));
            B_inv(:,:,i) = inv(B_perm(:,:,i));
        end
        
        %% ------ using probility methods ------
        [ X1, Y1, MeanA, MeanB, SigA, SigB ] = batchAXXB(A_perm, B_perm);
        [ Y2_inv, X2_inv, MeanA_inv, MeanB_inv, ~, ~] = batchAXXB(B_inv, A_inv);
%         [ X3, Y3, X_can, Y_can, ~, ~, ~, ~ ] = batchAXYB(A_perm, B_perm);
        [X3, Y3] = shah(A_perm, B_perm);
%         [X, Y] = li(AA,BB);
        
        m = size(X1,3);
        X2 = zeros(4,4,m);
        Y2 = zeros(4,4,m);
        for i = 1:m
            X2(:,:,i) = inv(X2_inv(:,:,i));
            Y2(:,:,i) = inv(Y2_inv(:,:,i));
        end
        
       %% Find out the optimal (X1, Y2) that minimizes cost1
        cost1 = zeros(8,8);
        for i = 1:size(X1,3)
            for j = 1:size(Y2,3)
                diff1 = MeanA*X1(:,:,i) - Y2(:,:,j)*MeanB;
                diff2 = MeanB_inv/Y2(:,:,j) - X1(:,:,i)\MeanA_inv;
                cost1(i,j) = norm(diff1(:)) * norm(diff2(:));
            end
        end
        
        [M1, I1] = min(cost1(:));
             
        [I_row, I_col] = ind2sub(size(cost1), I1);
        X = X1(:, :, I_row);
        Y = Y2(:, :, I_col);
        
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
            display(X)
        end
    end
    
end

%%
figure(5)
trplot(XActual(:,:),'color','r');
axis auto

figure(6)
trplot(YActual(:,:),'color','r');
axis auto

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
    subplot(2,1,1)
    plot(point, Err_Avg(:,[1,3]),'o')
    len1 = legend('$E_{R_{X prob}}$','$E_{R_{X Li}}$');
    set(len1,'FontSize',14,'Interpreter','latex');
    
    
    subplot(2,1,2)
    plot(point, Err_Avg(:,[2,4]),'o')
    len2 = legend('$E_{R_{Y prob}}$','$E_{R_{Y Li}}$');
    set(len2,'FontSize',14,'Interpreter','latex');

    figure
    subplot(2,1,1)
    plot(point, Err_Avg(:,[5,7]),'o')
    len3 = legend('$E_{t_{X prob}}$','$E_{t_{X Li}}$');
    set(len3,'FontSize',12,'Interpreter','latex');
   
    subplot(2,1,2)
    plot(point, Err_Avg(:,[6,8]),'o')
    len4 = legend('$E_{t_{Y prob}}$','$E_{t_{Y Li}}$');
    set(len4,'FontSize',12,'Interpreter','latex');
    
end
