function errXY = shift_main(shift) 
% clear all
% close all
% clc
% [A,B] =  AABB(100);
% load invariantrandom.mat
% global xxx yyy bbb
counter = 1;
trials = 5;
Mean=[0;0;0;0;0;0;];
% point = [50 1 50];
% sum = zeros(1,4);
point = [210 5 210];
sum = zeros(1,4);
sigArray = 2;
% shift = 2;
for k = point(1):point(2):point(3)
for i = 1:1:trials
%     waitbar((k-point(1))/(point(3)-point(1)));
    
%  if k==92
%      continue;
%  end
%% --Trajectory Generation-------------------------
 [A,B,XActual,YActual] =  ABGenerate_rand(k,sigArray,0);
% [A,B,X,Y] =  ABGen(k,0);
% Cov=sigArray*eye(6,6);
% [A,B,XActual,YActual] =  ABGenerate(k, 0, Mean, Cov);
 length = size(A,3);

As = A(:,:,1:(length - shift));
Bs = B(:,:,(shift + 1):length);

%% add noise
gmean = [0;0;0;0;0;0];	%Gaussian Noise Mean
nstd = 0.01;  %Gaussian Noise standard deviation Range

% B = sensorNoise(B, gmean, nstd, 1);

%% --using probility methods------------------------
% [XX(:,:,k),YY(:,:,k),err(k,:)]= haiyuan(A,B,X,Y,k,0.1);
% [ X,Y, MeanA, MeanB, SigA, SigB ] = batchAXYB(A, B);
[ X,Y, MeanA, MeanB, SigA, SigB ] = batchAXYBshift(As, Bs);
% [XX(:,:,k)] = AXYB_LieGroup2( A, B );
Y(4,1:3) = [0 0 0];
%% err analysis

sum(counter,1) = sum(counter,1) + roterror(X,XActual);
sum(counter,2) = sum(counter,2) + roterror(Y,YActual);
sum(counter,3) = sum(counter,3) + tranerror(X,XActual);
sum(counter,4) = sum(counter,4) + tranerror(Y,YActual);
end

num(counter) = k;
counter = counter + 1;

%% plot 
% for j = 1:1:k
%     figure(11);
%     trplot(A(:,:,j),'color','r');
%     axis auto
%     hold on
%     figure(12);
%     trplot(B(:,:,j),'color','b');
%     axis auto
%     hold on
% end
% figure(3);
% legend('Generated A');
% figure(4);
% legend('Generated B');

%% --plot X and Y-------------------------------------------
% figure(7);
% trplot(X(:,:),'color','b');
% axis auto
% hold on
% figure(7);
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
errXY = sum/trials;
% 
% figure(7)
% trplot(XActual(:,:),'color','r');
% % legend('solved and actual X');
% figure(7)
% trplot(YActual(:,:),'color','r');
% legend('solved and actual Y');
% figure
% info = {'rotErrX'; 'rotErrY'; 'tranErrX'; 'tranErrY'};
% for i = 1:1:4
%    
%    plot(num,sum(:,i)/trials,'-o');
%    hold on 
%    legend(info(i));
% end
% legend('Error($R_X$)','Error($R_Y$)','Error($t_X$)','Error($t_Y$)');
end