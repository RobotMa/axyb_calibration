clear all
close all
clc
errXY=zeros(4,11);
for shift = 0:1:5
    errXY(:,shift+1) = shift_main(shift);
    
end
plot(errXY')