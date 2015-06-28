function [A,B,X,Y] = ABGen(len,shift)

numPair = len;
shift = 0;

A = zeros(4,4,numPair);
B = zeros(4,4,numPair);

num = numPair + 2;
t2 = (0:(2*pi)/((num + shift)):2*pi);
twist = 0.0*sin(16*t2);

ElipseParam = [10, 10, 10];

%% data
Y1 = [-0.99908 -0.03266 0.02786 164.226/1000;
      0.02737  0.01553 0.99950 301.638/1000;
     -0.03308  0.99935 -0.01462 -962.841/1000;
     0.00000   0.00000 0.00000  1.00000;];
 X1 = [-0.97651 -0.09468 -0.19356 9.190/1000;
     0.06362 -0.98493 0.16082 5.397/1000;
     -0.20587 0.14473 0.96782 62.628/1000;
     0.00000 0.00000 0.00000 1.00000;];

% x = randn(6,1); X = expm(se3_vec(x));
X=inv(X1);
Y = Y1;
trajParam = [.5, .5, .5, 0, 0];
B1 = createTraj(ElipseParam(1), ElipseParam(2), ElipseParam(3), trajParam(1), trajParam(2), trajParam(3), trajParam(4), trajParam(5), num, twist);
% [A1, B1] = AB_genTraj(X, ElipseParam, trajParam, num, twist);

trajParam = [.5, .5, .5, 0, 0.5*pi];
% [A2, B2] = AB_genTraj(X, ElipseParam, trajParam, num, twist);
B2 = createTraj(ElipseParam(1), ElipseParam(2), ElipseParam(3), trajParam(1), trajParam(2), trajParam(3), trajParam(4), trajParam(5), num/2, twist);

% A = cat(3, A1, A2);
B = cat(3, B1, B2);

for m = 1:1:size(B,3)
    A(:,:,m) = Y * B(:,:,m) / X;
%     figure(1);
%     trplot(A(:,:,m),'color','r');
%     axis auto
%     hold on
%     figure(2);
%     trplot(B(:,:,m),'color','b');
%     axis auto
%     hold on
%     pause();
% end
% X = inv(X);
end
end