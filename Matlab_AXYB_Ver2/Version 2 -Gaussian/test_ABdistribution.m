clear all
%close all
clc

counter = 0;

Mean=[0;0;0;0;0;0];

Cov=1.2*eye(6,6);   

point = [50 10 50];
k = 50;

[A,B,XActual,YActual] =  ABGenerate(k, 0, Mean, Cov);

for n = 1:1:k
	figure(5);
    trplot(A(:,:,n),'color','b');
    axis auto
    hold on
end
for n = 1:1:k
	figure(6);
    trplot(B(:,:,n),'color','b');
    axis auto
    hold on
end
