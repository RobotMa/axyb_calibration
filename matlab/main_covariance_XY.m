% This is the main file for probabilistic AX = YN problem. This file
% analyzes the covariance of X and Y in reflecting the cerntainty of the
% solved results
clear
close all
clc

%% Add file dependencies
addpath ../../rvctools/robot
addpath ../../rvctools/common
addpath ../../axxb_calibration/matlab/new_mean/codegen/mex/distibutionPropsMex
addpath ../../axxb_calibration/matlab/Batch_Method_ED_KL_BS
addpath ../../kinematics/kinematics/lie_group
addpath ../../kinematics/kinematics/util
addpath ../../kinematics/kinematics/screws

%% Initialize Parameters
counter = 0;

gmean = [0; 0; 0; 0; 0 ;0];

coeff1 = 0.1; % Scaling factor for the covariances

cov = eye(6,6); % coeff1*cov

coeff2 = 0.0; % Scaling factor for the covariances

cov_noise = eye(6,6); % coeff2*cov

num = 20; % number of simulations

Num = 50; % number of data % 100

n_sets = 1:1:10; % number of sets

optPlot = 'lineplot'; % Plot the averaged error : 'lineplot' & ''boxplot'

opt_XY = 1; % generate random X, Y and Z
[XActual, YActual] = InitializeXY(opt_XY);

optNoise = true;

% Error container initialization
Err1  = zeros(length(n_sets), 4, num);
Err2  = zeros(length(n_sets), 4, num);

%% Main Loop
for k = n_sets
    
    counter = counter + 1;
    
    for s = 1:num
        %% Generate data triples with different distributions
        optPDF = 1;
        [A1, B1] = ...
            generateMultiSetsOfAB(k, Num, optPDF, gmean, coeff1*cov, XActual, YActual);
        
        optPDF = 2;
        [A2, B2] = ...
            generateMultiSetsOfABC(k, Num, optPDF, gmean, coeff1*cov, XActual, YActual);
        
        %% Add noise to data
        if optNoise
            B1 = addSensorNoiseToSet(B1, gmean, coeff1*coeff2, 2);
            B1 = addSensorNoiseToSet(B1, gmean, coeff1*coeff2, 2);
        end
        
        %% Solve for X, Y and Z using probabilistic approaches
        
%         for p = 1:k
%             [X_f11, Y_f11, Z_f11] = axbyczProb1(A11(:,:,p),   B11(:,:,:,p), C11(:,:,:,p), ... 
%                                                 A21(:,:,:,p), B21(:,:,:,p), C21(:,:,p));
% 
%             [X_f12, Y_f12, Z_f12] = axbyczProb1(A12(:,:,p),   B12(:,:,:,p), C12(:,:,:,p), ...
%                                                 A22(:,:,:,p), B22(:,:,:,p), C22(:,:,p));
%                                             
%             %% ----- Error Analysis ------
%             % ------- Prob1 Error with Data of 1st Distribution ------ %
%             Err11(counter,:,s) = Err11(counter,:,s) + ...
%                 getErrorAXBYCZ(X_f11, Y_f11, Z_f11, XActual, YActual, ZActual);
% 
%             % ------- Prob2 Error with Data of 1st Distribution ------ %
%             Err21(counter,:,s) = Err21(counter,:,s) + ...
%                 getErrorAXBYCZ(X_f21, Y_f21, Z_f21, XActual, YActual, ZActual);
%             
%         end
        
%         Err11(counter,:,s) = Err11(counter,:,s)/k;
%         Err21(counter,:,s) = Err21(counter,:,s)/k;
        
        %% Solve for X, Y, Z using hybrid approach
        [ X_1, Y_1 ] = probWangAXYB( A1, B1 );
                                        
        [ X_2, Y_2 ] = probWangAXYB( A2, B2 );

        % ------- Mixed Prob-Wang method ------ %
        Err1(counter,:,s) = getErrorAXBYCZ(X_1, Y_1, XActual, YActual);
        
        % ------- Mixed Prob-Wang method ------ %
        Err2(counter,:,s) = getErrorAXBYCZ(X_2, Y_2, XActual, YActual);
    end
    
end

%% Plot the averaged error w.r.t. covariances
plotProbResults(Err11, Err21, n_sets, optPlot)
plotProbResults(Err12, Err22, n_sets, optPlot)

optYaxis = '';
plotProbResultsAll(Err11, Err21, ErrW1, n_sets, optPlot, optYaxis)
plotProbResultsAll(Err12, Err22, ErrW2, n_sets, optPlot, optYaxis)

