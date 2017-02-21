clear all
close all
clc
% [A,B] =  AABB(100);
% load invariantrandom.mat
% global xxx yyy bbb
counter = 0;
trials = 20;
sigArray = [0.1 0.2 0.4 0.6 0.8 1 2];

h = waitbar(0);
for n = 1:1:size(sigArray,2)
for m = 1:1:trials

point = [50 10 50];
for k = point(1):point(2):point(3)

waitbar((m + (n-1)*trials)/(trials * size(sigArray,2)),h,sprintf('Now step %d in total %d ',(m + (n-1)*trials),trials*size(sigArray,2)))
% waitbar((k-point(1))/(point(3)-point(1)));

%% --Trajectory Generation-------------------------
 [A,B,XActual,YActual] =  ABGenerate(k,sigArray(n),0);
% [A,B,XActual,YActual] =  ABGen(k,0);

%% add noise
gmean = [0;0;0;0;0;0];	%Gaussian Noise Mean
nstd = 0.01;  %Gaussian Noise standard deviation Range

B = sensorNoise(B, gmean, nstd, 1);

%% --using probility methods------------------------
% [XX(:,:,k),YY(:,:,k),err(k,:)]= haiyuan(A,B,X,Y,k,0.1);
[ X,Y, MeanA, MeanB, SigA, SigB ] = batchAXYB(A, B);
% [XX(:,:,k)] = AXYB_LieGroup2( A, B );
Y(4,1:3) = [0 0 0];
%% err analysis
counter = counter + 1;
num(counter) = k;
Err(counter,1) = roterror(X,XActual);
Err(counter,2) = roterror(Y,YActual);
Err(counter,3) = tranerror(X,XActual);
Err(counter,4) = tranerror(Y,YActual);

figure(1)
trplot(X,'color','r');
hold on
axis auto
% legend('solved and actual X');
figure(2)
trplot(Y,'color','r');
hold on
axis auto

end

rotErrX(m,n) = Err(counter,1);
rotErrY(m,n) = Err(counter,2);
tranErrX(m,n) = Err(counter,3);
tranErrY(m,n) = Err(counter,4);
end
end

figure(1)
trplot(XActual(:,:),'color','k');
% legend('solved and actual X');
figure(2)
trplot(YActual(:,:),'color','k');
% % legend('solved and actual k');
% figure
% info = {'rotErrX'; 'rotErrY'; 'tranErrX'; 'tranErrY'};
% for i = 1:1:4
%    subplot(4,1,i);
%    plot(num,Err(:,i),'-o');
%    axis auto
%    legend(info(i));
% end
close(h);
h = msgbox('Operation Completed');
figure
boxplot(rotErrX,sigArray);
figure
boxplot(rotErrY,sigArray);
figure
boxplot(tranErrX,sigArray);
figure
boxplot(tranErrY,sigArray);
