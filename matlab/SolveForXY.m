function [outputX,outputY,outputError] =  SolveForXY(length, precision)
%% 

%% Times of simulation steps
len = length;
e = precision;
% digits(64);
%% data
Y = [-0.99908 -0.03266 0.02786 164.226/1000;
      0.02737  0.01553 0.99950 301.638/1000;
     -0.03308  0.99935 -0.01462 -962.841/1000;
     0.00000   0.00000 0.00000  1.00000;];
 X = [-0.97651 -0.09468 -0.19356 9.190/1000;
     0.06362 -0.98493 0.16082 5.397/1000;
     -0.20587 0.14473 0.96782 62.628/1000;
     0.00000 0.00000 0.00000 1.00000;];
 qz = [pi/2,0,pi/2,0,0,0];
 mdl_puma560;
 Binitial = p560.fkine(qz);
 Ainitial = Y * Binitial * X;
%  trplot(Ainitial,'color','b');
%  trplot(Y,'color','b');
%  hold on
% load invariantrandom.mat 
 for m = 1:1:len
     r1 = rand(6,1) - 0.5;
     B(:,:,m) = Binitial * expm(0.1 * (twist(r1)));
     A(:,:,m) = Y * B(:,:,m) * X;
     [OmegaA(:,:,m) thetaA(m)] = rotparam(A(1:3,1:3,m));
     dA(m) = dot(A(1:3,4,m),OmegaA(:,:,m));
%      trplot(A(:,:,m),'color','r');
 end

%% Calculat MA
sumMA = 0;
N = len;
for i = 1:1:N
    sumMA = sumMA + real(logm(A(:,:,i)));
    
end
MA = expm(sumMA/N);

while(1)
   %% calculate cost function
    sumMA = 0;
    for i = 1:1:N
        sumMA = sumMA + real(logm(inv(MA)*A(:,:,i)));
    end
    costFunA = abs(sumMA);
    if(sum(sum((costFunA.^2))) < e)
        break;
    end
    MA = MA * expm(sumMA/N);
    
end

%% Calculat MB
sumMB = 0;
N = len;
for i = 1:1:N
    sumMB = sumMB + real(logm(B(:,:,i)));
    
end
MB = expm(sumMB/N);

while(1)
   %% calculate cost function
    sumMB = 0;
    for i = 1:1:N
        sumMB = sumMB + real(logm(inv(MB)*B(:,:,i)));
    end
    costFunB = abs(sumMB);
    if(sum(sum((costFunB.^2)))  < e)
        break;
    end
    MB = MB * expm(sumMB/N);
end
%% Calculate Covariance of A
sumACov = 0;
for i = 1:1:N
aMatrix = real(logm(inv(MA)*A(:,:,i)));
aVect = twistcoords(aMatrix);
sumACov = sumACov + aVect * aVect.';
end
ACov = sumACov / N;

%% Calculate Covariance of B
sumBCov = 0;
for i = 1:1:N
    bMatrix = real(logm(inv(MB)*B(:,:,i)));
    bVect = twistcoords(bMatrix);
    sumBCov = sumBCov + bVect * bVect.';
end
BCov = sumBCov / N;

Omega(:,:,1) = [1 0 0; 0 1 0; 0 0 1;];
Omega(:,:,2) = [-1 0 0; 0 -1 0; 0 0 1;];
Omega(:,:,3) = [-1 0 0; 0 1 0; 0 0 -1;];
Omega(:,:,4) = [1 0 0; 0 -1 0; 0 0 -1;];

rACov = ACov(1:3,1:3);
rBCov = BCov(1:3,1:3);
tACov = ACov(1:3,4:6);
tBCov = BCov(1:3,4:6);

[VA,DA,WA] = eig(rACov);
[VB,DB,WB] = eig(rBCov);
if(det(WA) < 0)
    WA = - WA;
end
if(det(WB) < 0)
    WB = - WB;
end

for k = 1:1:4
    Rx(:,:,k) = WA * Omega(:,:,k) * WB.';
    temp = inv((Rx(:,:,k).') * rACov * Rx(:,:,k))*(tBCov - (Rx(:,:,k).') * tACov * Rx(:,:,k));
    tx(:,:,k) = Rx(:,:,k) * skewcoords(temp);
    X1(:,:,k) = [Rx(:,:,k) tx(:,:,k);0 0 0 1];
    Y1(:,:,k) = MA * X1(:,:,k) * inv(MB); 
    Y1(4,1:3,k)=0;

%     trplot(Y1(:,:,k),'color','r');
%      hold on
%      disp('a');
end
% index = (1:1:N)';
% color = ['b' 'c' 'r' 'm']
% h1 = figure(1);
% h2 = figure(2);

for k = 1:1:4
    for m = 1:1:N
        Bdot(:,:,m,k) = inv(X1(:,:,k)) * Y1(:,:,k) * B(:,:,m);
        [OmegaB(:,:,m) thetaB(m,k)] = rotparam(Bdot(1:3,1:3,m,k));
        dB(m,k) = dot(Bdot(1:3,4,m,k),OmegaB(:,:,m));
        
        delta (m,k) = thetaA(m) - thetaB(m,k);
        deltad (m,k) = dA(m) - dB(m,k);
    end
    normMin(k) = norm(delta (:,k)) + norm(deltad(:,k));
%     figure(1);
%     ax(k) = subplot(4,1,k);
%     
%     plot(ax(k),index,thetaB(:,k),color(k),'LineWidth',1);
%     
%     hold on;
%     plot(ax(k),index,thetaA,'k','LineWidth',2);
%     
%     figure(2);
%     ax(k) = subplot(4,1,k);
%     
%     plot(ax(k),index,dB(:,k),color(k),'LineWidth',1);
%     
%     hold on;
%     plot(ax(k),index,dA,'k','LineWidth',2);
 
end
[a indexMin] = min(normMin);
outputX = X1(:,:,indexMin);
outputY = Y1(:,:,indexMin);
outputError(1) = norm(skewcoords(logm(inv(outputX(1:3,1:3)) * X(1:3,1:3))));
outputError(2) = norm(skewcoords(logm(inv(outputY(1:3,1:3)) * Y(1:3,1:3))));
outputError(3) = norm(outputX(1:3,4) - X(1:3,4));
outputError(4) = norm(outputY(1:3,4) - Y(1:3,4));


% figure(3)
% figure(4)
% for k = 1:1:4
%  figure(3)
%  plot(index,delta(:,k),color(k),'LineWidth',2);
%  hold on
%  figure(4)
%  plot(index,deltad(:,k),color(k),'LineWidth',2);
%  hold on
% end

%% The theta of MA MB
% [OmegaMA thetaMA] = rotparam(MA(1:3,1:3));
% dMA = dot(MA(1:3,4),OmegaMA);
% for k = 1:1:4
% MBdot(:,:,k) = inv(X1(:,:,k)) * Y1(:,:,k) * MB;
% [OmegaMB(:,:,k) thetaMB(k)] = rotparam(MBdot(1:3,1:3,k));
% deltaMTheta(k) = thetaMA - thetaMB(k);
% dMB(k) = dot(MBdot(1:3,4,k),OmegaMB(:,:,k));
% deltaMd(k) = dMA - dMB(k);
% end
% figure
% [ax,p1,p2] = plotyy(1:4,deltaMd,1:4,deltaMTheta);
% ylabel(ax(1),'d of MA - MB') % label left y-axis
% ylabel(ax(2),'Theta of MA - MB') % label right y-axis
% xlabel(ax(2),'Index') % label x-axis
% grid(ax(1),'on')
% p1.Marker = 'o';
% p2.Marker = '*';
% p1.LineStyle = '--';
% p1.LineWidth = 2;
% p2.LineWidth = 2;
%     trplot(X);
end