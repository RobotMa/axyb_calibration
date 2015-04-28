clear all
close all
figure(1);
for k = 1:1:100
[a(:,:,k),b(:,:,k),c(k,:)]=SolveForXY(100,0.1);
 trplot(a(:,:,k),'color','b');
 hold on
end
figure
for m = 1:1:4
   subplot(4,1,m);
   plot(c(:,m));
end

