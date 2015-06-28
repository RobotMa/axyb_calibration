for i = 1:1:size(sigArray,2)
figure
h1 = subplot(4,1,1)
plot(median(rotErrX(:,1:6,i)));
axis auto;
h2 = subplot(4,1,2)
plot(median(rotErrY(:,1:6,i)));
axis auto;
h3 = subplot(4,1,3)
plot(median(tranErrX(:,1:6,i)));

axis auto;
h4 = subplot(4,1,4)
plot(median(tranErrY(:,1:6,i)));
axis auto;
end