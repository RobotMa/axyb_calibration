function [G] = Ad(g)
R = g(1:3,1:3);
t = g(1:3,4);
T = [0    -t(3)  t(2);
    t(3)  0      -t(1);
    -t(2) t(1)   0;]
G= [R zeros(3,3); T*R R];
end