clear;
close all;

Path='./data/';

X=csvread(strcat(Path,'x.csv'),1,0);
Y=csvread(strcat(Path,'y.csv'),1,0);
spline_X=csvread(strcat(Path,'spline_x.csv'),1,0);
spline_Y=csvread(strcat(Path,'spline_y.csv'),1,0);

figure(1)
plot(X,Y,'+');hold on;
plot(spline_X,spline_Y,'-');hold on;

