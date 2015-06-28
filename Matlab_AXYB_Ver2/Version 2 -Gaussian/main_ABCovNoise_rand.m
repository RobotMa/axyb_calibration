clear all
close all
clc
% [A,B] =  AABB(100);
% load invariantrandom.mat
% global xxx yyy bbb

gmean = [0;0;0;0;0;0];	%Gaussian Noise Mean
nstd = [0.01 0.02 0.04 0.06 0.08];
% nstd = [0.01 0.04 0.06 0.08];%[0.01 0.02 0.04 0.06 0.08 0.1];  %Gaussian Noise standard deviation Range

counter = 0;
trials = 5;
% sigArrayR =  0.3;
% sigArrayT =  0.3;%[0.1 0.2 0.3];
sigArray = [0.1 0.2 0.4 0.6 0.8 1 2];
Mean=[0;0;0;0;0;0;];

h = waitbar(0);

for n = 1:1:size(sigArray,2)
    
    for n_noise = 1:1:size(nstd,2)
    
    for m = 1:1:trials

        point = [100 10 100];
        for k = point(1):point(2):point(3)

            waitbar((trials * (n_noise - 1) + m + (n-1)*trials*size(nstd,2))/(size(nstd,2) * trials * size(sigArray,2)),h,sprintf('Now step %d in total %d ',(trials * (n_noise - 1) + m + (n-1)*trials*size(nstd,2)),size(nstd,2) * trials*size(sigArray,2)))
            % waitbar((k-point(1))/(point(3)-point(1)));
%             Cov=sigArray(n)*eye(6,6); 
%             CovR=sigArrayR(n)*eye(3,3); 
%             CovT=sigArrayT(1)*eye(3,3);
           %% --Trajectory Generation-------------------------
            [A,B,XActual,YActual] =  ABGenerate_rand(k,sigArray(n),0);
            % [A,B,XActual,YActual] =  ABGen(k,0);

           %% add noise
            B = sensorNoise(B, gmean, nstd(n_noise), 1);

           %% --using probility methods------------------------
            % [XX(:,:,k),YY(:,:,k),err(k,:)]= haiyuan(A,B,X,Y,k,0.1);
            [ X,Y, MeanA, MeanB, SigA, SigB ] = batchAXYB(A, B);
            % [XX(:,:,k)] = AXYB_LieGroup2( A, B );
            Y(4,1:3) = [0 0 0];
            %% err analysis
%             counter = counter + 1;
%             num(counter) = k;
%             Err(counter,1) = roterror(X,XActual);
%             Err(counter,2) = roterror(Y,YActual);
%             Err(counter,3) = tranerror(X,XActual);
%             Err(counter,4) = tranerror(Y,YActual);

        end

        rotErrX(m,n_noise,n) = roterror(X,XActual);
        rotErrY(m,n_noise,n) = roterror(Y,YActual);
        tranErrX(m,n_noise,n) = tranerror(X,XActual);
        tranErrY(m,n_noise,n) = tranerror(Y,YActual);
        
% figure(5)
% trplot(X,'color','b');
% hold on
% axis auto
% 
% figure(6)
% trplot(Y,'color','b');
% hold on
% axis auto

    end
    end
end

% figure(5)
% trplot(XActual(:,:),'color','r');
% % legend('solved and actual X');
% figure(6)
% trplot(YActual(:,:),'color','r');
% legend('solved and actual Y');
% figure
% info = {'rotErrX'; 'rotErrY'; 'tranErrX'; 'tranErrY'};
% for i = 1:1:4
%    subplot(4,1,i);
%    plot(num,Err(:,i),'-o');
%    axis auto
%    legend(info(i));
% end
% close(h);
h = msgbox('Operation Completed');
for i = 1:1:size(sigArray,2)
figure
subplot(4,1,1)
boxplot(rotErrX(:,:,i));
subplot(4,1,2)
boxplot(rotErrY(:,:,i));
subplot(4,1,3)
boxplot(tranErrX(:,:,i));
subplot(4,1,4)
boxplot(tranErrY(:,:,i));
end
