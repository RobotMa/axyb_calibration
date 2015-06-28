function [X,Y]=zhuang(A,B)
% Solves the problem AX=YB
% using the formulation of
%
% Simultaneous Robot/World and Tool/Flange Calibration 
% by Solving Homogeneous Transformation Equations of the 
% form AX=YB
% H. Zhuang, Z. S. Roth, R. Sudhakar
%
% Mili Shah
% July 2014

[m,n]=size(A);
n = n/4;

G = zeros(3*n,6);
c = zeros(3*n,1);
for i = 1:n
    RA = A(1:3,4*i-3:4*i-1);
    RB = B(1:3,4*i-3:4*i-1);
    qa = rot2q(RA);
    qb = rot2q(RB);
    
    Oa = [0 -qa(3) qa(2); qa(3) 0 -qa(1); -qa(2) qa(1) 0];
    Ob = [0 -qb(3) qb(2); qb(3) 0 -qb(1); -qb(2) qb(1) 0];
    
    G(3*i-2:3*i,:) = [qa(4)*eye(3)+Oa+qa(1:3)*qa(1:3)'/qa(4),...
        -qb(4)*eye(3)+Ob-qa(1:3)*qb(1:3)'/qa(4)];
    c(3*i-2:3*i,:) = qb(1:3)-qb(4)/qa(4)*qa(1:3);
end

w = G\c;
y = zeros(4,1);
x = zeros(4,1);

y(4) = (1+w(4)^2+w(5)^2+w(6)^2).^(-1/2);
y(1:3) = y(4)*w(4:6);
x(1:3) = y(4)*w(1:3);
x(4) = (1-x(1)^2-x(2)^2-x(3)^2).^(1/2);

X = q2rot(x);
Y = q2rot(y);

AA = zeros(3*n,6);
b = zeros(3*n,1);
for i = 1:n
    AA(3*i-2:3*i,:) = [-A(1:3,4*i-3:4*i-1) eye(3)];
    b(3*i-2:3*i,:) = A(1:3,4*i) - kron(B(1:3,4*i)',eye(3))*reshape(Y,9,1);
end
t = AA\b;

X = [X t(1:3);[0 0 0 1]];
Y = [Y t(4:6);[0 0 0 1]];
end

function q = rot2q(R)

% Converting a rotation matrix R to
% quaternion q from
%
% http://www.theworld.com/%7Esweetser/quaternions/ps/stanfordaiwp79-salamin.pdf
%
% APPLICATION OF QUATERNIONS TO COMPUTATION WITH ROTATIONS
% Working Paper, Stanford AI Lab, 19791
% by Eugene Salamin
%
% Mili Shah
% Aug 25, 2011

q = zeros(4,1);

q(4) = 1/2*sqrt(1+R(1,1)+R(2,2)+R(3,3));
q(1) = 1/2*sqrt(1+R(1,1)-R(2,2)-R(3,3));
q(2) = 1/2*sqrt(1-R(1,1)+R(2,2)-R(3,3));
q(3) = 1/2*sqrt(1-R(1,1)-R(2,2)+R(3,3));

[k, in] = max(q);

if in == 1
    q(2) = 1/4/q(1)*(R(1,2)+R(2,1));
    q(3) = 1/4/q(1)*(R(1,3)+R(3,1));
    q(4) = 1/4/q(1)*(R(3,2)-R(2,3));
end
if in == 2
    q(1) = 1/4/q(2)*(R(1,2)+R(2,1));
    q(3) = 1/4/q(2)*(R(2,3)+R(3,2));
    q(4) = 1/4/q(2)*(R(1,3)-R(3,1));
end
if in == 3
    q(1) = 1/4/q(3)*(R(1,3)+R(3,1));
    q(2) = 1/4/q(3)*(R(2,3)+R(3,2));
    q(4) = 1/4/q(3)*(R(2,1)-R(1,2));
end
if in == 4
    q(1) = 1/4/q(4)*(R(3,2)-R(2,3));
    q(2) = 1/4/q(4)*(R(1,3)-R(3,1));
    q(3) = 1/4/q(4)*(R(2,1)-R(1,2));
end
end

function R = q2rot(q)

% Converting a quaternion q to
% Rotation R from
%
% http://www.theworld.com/%7Esweetser/quaternions/ps/stanfordaiwp79-salamin.pdf
%
% APPLICATION OF QUATERNIONS TO COMPUTATION WITH ROTATIONS
% Working Paper, Stanford AI Lab, 19791
% by Eugene Salamin
%
% Mili Shah
% Aug 25, 2011

R = zeros(3,3);

R(1,1) = q(4)^2+q(1)^2-q(2)^2-q(3)^2;
R(2,2) = q(4)^2-q(1)^2+q(2)^2-q(3)^2;
R(3,3) = q(4)^2-q(1)^2-q(2)^2+q(3)^2;

R(1,2) = 2*(-q(4)*q(3)+q(1)*q(2));
R(2,1) = 2*(q(4)*q(3)+q(1)*q(2));

R(1,3) = 2*(q(4)*q(2)+q(1)*q(3));
R(3,1) = 2*(-q(4)*q(2)+q(1)*q(3));

R(2,3) = 2*(-q(4)*q(1)+q(2)*q(3));
R(3,2) = 2*(q(4)*q(1)+q(2)*q(3));
end

