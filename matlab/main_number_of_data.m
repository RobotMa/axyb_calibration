clear
close all
clc

%% Add file dependencies
addpath ../../rvctools/robot
addpath ../../rvctools/common
addpath ../../axxb_calibration/matlab/new_mean/codegen/mex/distibutionPropsMex
addpath ../../axxb_calibration/matlab/Batch_Method_ED_KL_BS
addpath ../../kinematics/kinematics/util
addpath ../../kinematics/kinematics/screws
addpath ../../kinematics/kinematics/lie_group

%% Initialize Parameters
gmean = [0; 0; 0; 0; 0 ;0];

nstd = 0.001; % Gaussian Noise standard deviation Range

coeff = 0.1;

cov = coeff*eye(6,6);

point = 100:50:200;

num = 20;

boxplot = false;

lineplot = true;

counter = 0;

%% 
for k = point
    
    counter = counter + 1;
    for s = 1:num
        %% ------ Trajectory Generation --------
        [A, B, XActual, YActual] =  ABGenerate(k, 0, gmean, cov);
        
        %% ------- add noise -------
        gmean = [0; 0; 0; 0; 0; 0];	% Gaussian Noise Mean
        
        A_noise = sensorNoise(A, gmean, nstd, 2);
        B_noise = sensorNoise(B, gmean, nstd, 2);
        
        [X1, Y1] = batchAXYB(A_noise, B_noise, false, nstd, nstd);
        
        % ---- Current implementation of noise correction method 
        % -                doesn't work
        [X2, Y2] = batchAXYB(A_noise, B_noise, true, nstd, nstd);
        
        %% ----- err analysis ------
        if boxplot        
            Err(counter,s,1) = roterror(X, XActual);
            Err(counter,s,2) = roterror(Y, YActual);

            Err(counter,s,3) = tranerror(X, XActual);
            Err(counter,s,4) = tranerror(Y, YActual);

        elseif lineplot
            Err(counter, 1, s) = roterror(X1, XActual);
            Err(counter, 2, s) = roterror(Y1, YActual);
            Err(counter, 3, s) = roterror(X2, XActual);
            Err(counter, 4, s) = roterror(Y2, YActual);

            Err(counter, 5, s) = tranerror(X1, XActual);
            Err(counter, 6, s) = tranerror(Y1, YActual);
            Err(counter, 7, s) = tranerror(X2, XActual);
            Err(counter, 8, s) = tranerror(Y2, YActual);
        end
    end
    
end

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
    plot(point, Err_Avg(:,[1,3]))
    legend('rotX_{original}','rotX_{new}')

    subplot(2,1,2)
    plot(point, Err_Avg(:,[2,4]))
    legend('rotY_{original}','rotY_{new}')
    
    figure
    subplot(2,1,1)
    plot(point, Err_Avg(:,[5,7]))
    legend('tranX_{original}','tranX_{new}')
   
    subplot(2,1,2)
    plot(point, Err_Avg(:,[6,8]))
    legend('tranX_{original}','tranX_{new}')
    
end
