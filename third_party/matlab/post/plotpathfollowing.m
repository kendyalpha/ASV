clear;
close all;

Path='20190723/';
load(strcat(Path,'data.mat'));

m=length(estimator.timestamp);

timeselect=[10 300]; % second
index_path=find(indicator.indicator_controlmode==2);
index_path=index_path(44:end);
index_select=find(timeselect(1)<=estimator.timestamp ...
    & estimator.timestamp<= timeselect(2));
index_select_planner=find(timeselect(1)<=planner.timestamp ...
    & planner.timestamp<= timeselect(2));
index_select_indicator=find(timeselect(1)<=indicator.timestamp ...
    & indicator.timestamp<= timeselect(2));
    

timestamp=estimator.timestamp(index_path);

figure(1)
title('time series')
subplot(311);
plot(timestamp, estimator.state(index_path,3), '-r', 'linewidth', 2);hold on;
plot(timestamp, planner.setpoint(index_path,3), '--b', 'linewidth', 2);
ylabel('theta (deg)');
xlim([min(timestamp) max(timestamp)]);
legend('state','desired value');

subplot(312);
plot(estimator.timestamp(index_path),...
    estimator.state(index_path,4), '-r', 'linewidth', 2);hold on;
plot(planner.timestamp(index_path), ...
    planner.v_setpoint(index_path,1), '--b', 'linewidth', 2);
ylabel('u (m/s)');
xlim([min(timestamp) max(timestamp)]);
legend('state','desired value');


subplot(313);
plot(estimator.timestamp(index_path),...
    estimator.state(index_path,6), '-r', 'linewidth', 2);hold on;
plot(planner.timestamp(index_path), ...
    planner.v_setpoint(index_path,3), '--b', 'linewidth', 2);
ylabel('r (rad/s)');
xlim([min(timestamp) max(timestamp)]);
legend('state','desired value');


figure(2);
title('trajectory')
plot(estimator.state(index_path,2), ...
    estimator.state(index_path,1),'k.'); hold on;
plot(planner.waypoint0(:,2),planner.waypoint0(:,1),'r+');hold on;
plot(planner.waypoint1(:,2),planner.waypoint1(:,1),'ro'); hold on;

x_min=min(estimator.state(index_path,2));
x_max=max(estimator.state(index_path,2));
y_min=min(estimator.state(index_path,1));
y_max=max(estimator.state(index_path,1));
delta_x=x_max-x_min;
delta_y=y_max-y_min;
percent=0.1;
xlim([x_min-percent*delta_x, x_max+percent*delta_x]);
ylim([y_min-percent*delta_y, y_max+percent*delta_y]);
ylabel('x (m)');
xlabel('y (m)');
axis equal;


figure(3)
subplot(311);
plot(estimator.timestamp(index_path), ...
    estimator.perror(index_path,3), '-r', 'linewidth', 2);
ylabel('error-theta (rad)')

subplot(312);
plot(estimator.timestamp(index_path), ...
    estimator.verror(index_path,3), '-r', 'linewidth', 2);
ylabel('error-r (rad/s)')

subplot(313);
plot(controller.timestamp(index_path), ...
    controller.tau(index_path,3), '-r', 'linewidth', 2);
ylabel('tau-Mz (N*m)')