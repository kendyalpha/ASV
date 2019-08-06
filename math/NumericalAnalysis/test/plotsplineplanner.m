clear;
close all;

Path='./data/';

X=csvread(strcat(Path,'x.csv'),1,0);
Y=csvread(strcat(Path,'y.csv'),1,0);
rx=csvread(strcat(Path,'rx.csv'),1,0);
ry=csvread(strcat(Path,'ry.csv'),1,0);
k=csvread(strcat(Path,'k.csv'),1,0);
yaw=csvread(strcat(Path,'yaw.csv'),1,0);

figure(1)
plot(X,Y,'+');hold on;
plot(rx,ry,'-');hold on;

figure(2)
plot(k);
title('curvature')

figure(3)
plot(yaw);
title('yaw')

