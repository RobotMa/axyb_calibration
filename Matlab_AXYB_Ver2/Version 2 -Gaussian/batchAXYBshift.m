function [ X,Y, MeanA, MeanB, SigA, SigB ] = batchAXYBshift(A, B)

X_candidate = zeros(4,4,8);
Y_candidate = zeros(4,4,8);
thetaA = zeros(size(A,3),1);
thetaB = zeros(size(A,3),8);
dA = zeros(size(A,3),1);
dB = zeros(size(A,3),8);
deltaTheta = zeros(size(A,3),8);
deltaD = zeros(size(A,3),8);
normMin = zeros(8,1);

[ MeanA, SigA ] = distibutionProps(A);
[ MeanB, SigB ] = distibutionProps(B);

[ VA, ~ ] = eig( SigA(1:3,1:3) );
[ VB, ~ ] = eig( SigB(1:3,1:3) );

Q1 = eye(3);
Q2 = [-1 0 0; 0 -1 0; 0 0 1];
Q3 = [-1 0 0; 0 1 0; 0 0 -1];
Q4 = [1 0 0; 0 -1 0; 0 0 -1];

% There are 8 possiblities of Rx

Rx_solved(:,:,1) = VA*Q1*VB';
Rx_solved(:,:,2) = VA*Q2*VB';
Rx_solved(:,:,3) = VA*Q3*VB';
Rx_solved(:,:,4) = VA*Q4*VB';
Rx_solved(:,:,5) = VA*-Q1*VB';
Rx_solved(:,:,6) = VA*-Q2*VB';
Rx_solved(:,:,7) = VA*-Q3*VB';
Rx_solved(:,:,8) = VA*-Q4*VB';

Bnew = zeros(4,4,8);
for i = 1:1:8
    tx_temp = so3_vec(((Rx_solved(:,:,i)'*SigA(1:3,1:3)*Rx_solved(:,:,i))^-1*(SigB(1:3,4:6)-Rx_solved(:,:,i)'*SigA(1:3,4:6)*Rx_solved(:,:,i)))');
    tx = -Rx_solved(:,:,i)*tx_temp;
    X_candidate(:,:,i) = [Rx_solved(:,:,i) tx; [0 0 0] 1];
    Y_candidate(:,:,i) = MeanA * X_candidate(:,:,i) /(MeanB);
end
% use Fouier transformation to find the correspondence between A and X-1YB

for i = 1:1:size(A,3)
    [thetaA(i), ~, dA(i), ~] = param_extract(A(:,:,i));
end

for j = 1:1:8
    for i = 1:1:size(A,3)
        Bnew(:,:,i) = X_candidate(:,:,j)\Y_candidate(:,:,j)*B(:,:,i);
        [thetaB(i,j), ~, dB(i,j), ~] = param_extract(Bnew(:,:,i));
%         deltaTheta (i,j) = thetaA(i) - thetaB(i,j);
%         deltaD (i,j) = dA(i) - dB(i,j);
    end
    
%     normMin(j) = norm(deltaTheta (:,j)) + norm(deltaD(:,j));
%     pause;
end
averageA = mean(thetaA(:));
stdA = std(thetaA(:));
for k = 1:1:8
    averageB(k) = mean(thetaB(:,k));
    stdB(k) = std(thetaB(:,k));
end

length = size(thetaB,1);
shiftThetaB = zeros(length,8);
shiftDB = zeros(length,8);
% deltaTheta =zeros(length - shift,8);
% deltaD = zeros(length - shift,8);

for k = 1:1:8
s1 = (thetaA(:) - averageA) / stdA ;
s2 = (thetaB(:,k) - averageB(k)) / stdB(k);
[acor,lag] = xcorr(s2,s1);

[~,I] = max(abs(acor));
lagDiff(k) = lag(I);
figure(1)
% timeDiff = lagDiff/Fs;
subplot(8,1,k);
% stem(lag,acor,'.');

% plot(lag,acor,'k','LineWidth',1);
% hold on
shift = lag(I)

if shift < 0
    shiftThetaB((-shift + 1):length,k) = thetaB(1:(length + shift),k);
    shiftDB((-shift + 1):length,k) = dB(1:(length + shift),k);
else
    shiftThetaB(1:(length - shift),k) = thetaB((shift + 1):(length),k);
    shiftDB((1):(length - shift),k) = dB((shift + 1):(length),k);
end
% pause();
deltaTheta (:,k) = thetaA(:) - shiftThetaB(:,k);
deltaD (:,k) = dA(:) - shiftDB(:,k);

normMin(k) = norm(deltaTheta (:,k)) + norm(deltaD(:,k));

% stem(lag(I),acor(I),'ro','LineWidth',1);

% figure(2)
% subplot(8,1,k)
% plot(thetaA(:),'r');
% hold on
% plot(shiftThetaB(:,k),'b')
% figure(3)
% subplot(8,1,k)
% plot(dA(:),'r');
% hold on
% plot(shiftDB(:,k),'b')
end



[~, indexMin] = min(normMin);
X = X_candidate(:,:,indexMin);
Y = Y_candidate(:,:,indexMin);
% X=0;
% Y=0;
% [~, Na, ~, ~] = param_extract(MeanA);
% [~, Nb, ~, ~] = param_extract(MeanB);
% na = so3_vec(Na);
% nb = so3_vec(Nb);

% min = inf;
% for i = 1:8
%     if (abs(det(Rx_solved(:,:,i))-1)<0.001) && (norm(na-Rx_solved(1:3,1:3,i)*nb) < min)
%         min = norm(na-Rx_solved(1:3,1:3,i)*nb);
%         Rx = Rx_solved(:,:,i);
%    end
% end

%% plot invariant
% color = ['y','m','c','r','g','b','w','k'];
% figure
% for i = 1:1:8
%     
% %     abs(deltaTheta (:,i)) + abs(deltaD(:,i))
%     if color(i) == 'w'
%          plot(abs(deltaTheta (:,i)) + abs(deltaD(:,i)),'LineWidth',1,'color',[0.6 0.4 0.6]);
%     else
%     plot(abs(deltaTheta (:,i)) + abs(deltaD(:,i)),'LineWidth',1,'color',color(i));
%     end
%     hold on    
% 
% end
%     legend(color(1),color(2),color(3),color(4),color(5),color(6),color(7),color(8))
% figure
% for i = 1:1:8
%     plot(dA(:)-dB(:,i),color(i));
%     hold on
%     legend i
% end

% figure
% for i = 1:1:8
%    subplot(8,1,i);
%    plot(thetaA(:),'-r');
%    hold on
%    plot(thetaB(:,i),'-b');
% end
% axis auto
% figure
% hold on
% for i = 1:1:8
%    subplot(8,1,i);
%    plot(dA(:),'-r');
%    hold on
%    plot(dB(:,i),'-b');
% end
% axis auto

end