clear all
close all
clc
% [A,B] =  AABB(100);
% load invariantrandom.mat
% global xxx yyy bbb
counter = 0;
sig = 0.2;    
point = [80 10 100];
for k = point(1):point(2):point(3)
    waitbar((k-point(1))/(point(3)-point(1)));
    
%  if k==92
%      continue;
%  end
%% --Trajectory Generation-------------------------
 [A,B,XActual,YActual] =  ABGenerate(k, sig,0);
% [A,B,XActual,YActual] =  ABGen(k,0);

%% add noise
gmean = [0;0;0;0;0;0];	%Gaussian Noise Mean
nstd = 0.01;  %Gaussian Noise standard deviation Range

% B = sensorNoise(B, gmean, nstd, 1);

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
% figure(5);
% trplot(X(:,:),'color','b');
% axis auto
% hold on
% figure(6);
% trplot(Y(:,:),'color','b');
% axis auto
% hold on
% for n = 1:1:k
% 	figure(3);
%     trplot(A(:,:,n),'color','b');
%     hold on
% end
% for n = 1:1:k
% 	figure(4);
%     trplot(B(:,:,n),'color','b');
%     hold on
% end
end


figure(5)
trplot(XActual(:,:),'color','r');
% legend('solved and actual X');
figure(6)
trplot(YActual(:,:),'color','r');
% legend('solved and actual Y');
figure
info = {'rotErrX'; 'rotErrY'; 'tranErrX'; 'tranErrY'};
for i = 1:1:4
   subplot(4,1,i);
   plot(num,Err(:,i),'-o');
   axis auto
   legend(info(i));
end
