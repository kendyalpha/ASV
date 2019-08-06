clear;
close all;

Path='20190723/';
load(strcat(Path,'data.mat'));

m=size(controller.alpha,2);

% timeselect=[860 950]; % second
% timeselect=[200 280]; % second
timeselect=[430 520]; % second
index_select_estimator=find(timeselect(1)<=estimator.timestamp ...
    & estimator.timestamp<= timeselect(2));
% index_select_planner=find(timeselect(1)<=planner.timestamp ...
%     & planner.timestamp<= timeselect(2));

meanx=0;
meany=0;
figure(1); 
title('measurement')

subplot(611);
plot(estimator.timestamp(index_select_estimator),...
    estimator.measurement(index_select_estimator,1)-meanx, '-r', 'linewidth', 2);hold on;
plot(estimator.timestamp(index_select_estimator),...
    estimator.state(index_select_estimator,1)-meanx, ':k', 'linewidth', 2);hold on;
plot(gps.timestamp(index_select_estimator), ...
    gps.y(index_select_estimator)-meanx, '--b', 'linewidth', 2);
ylabel('x (m)');
xlim(timeselect);
legend('measurement', 'state','gps');

subplot(612); 
plot(estimator.timestamp(index_select_estimator), ...
    estimator.measurement(index_select_estimator,2)-meany, '-r', 'linewidth', 2);hold on;
plot(estimator.timestamp(index_select_estimator), ...
    estimator.state(index_select_estimator,2)-meany, ':k', 'linewidth', 2);
plot(gps.timestamp(index_select_estimator), ...
    gps.x(index_select_estimator)-meany, '--b', 'linewidth', 2);
xlim(timeselect);
ylabel('y (m)');
legend('measurement', 'state','gps');

subplot(613);
plot(estimator.timestamp(index_select_estimator), ...
    estimator.measurement(index_select_estimator,3)*180/pi, '-r', 'linewidth', 2);hold on;
plot(estimator.timestamp(index_select_estimator), ...
    estimator.state(index_select_estimator,3)*180/pi, ':k', 'linewidth', 2); hold on;
plot(gps.timestamp(index_select_estimator), ...
    restrictheading(gps.heading(index_select_estimator)), '--b', 'linewidth', 2);
xlim(timeselect);
ylabel('theta (deg)');
legend('measurement', 'state','gps');

subplot(614);
plot(estimator.timestamp(index_select_estimator), ...
    estimator.measurement(index_select_estimator,4), '-r', 'linewidth', 2);hold on;
plot(estimator.timestamp(index_select_estimator), ...
    estimator.state(index_select_estimator,4), ':k', 'linewidth', 2);hold on;
plot(gps.timestamp(index_select_estimator), gps.Vn(index_select_estimator), '--b', 'linewidth', 2);
xlim(timeselect);
ylabel('u (m/s)');
legend('measurement', 'state','gps(Vn)');

subplot(615);
plot(estimator.timestamp(index_select_estimator), ...
   estimator.measurement(index_select_estimator,5), '-r', 'linewidth', 2);hold on;
plot(estimator.timestamp(index_select_estimator), ...
    estimator.state(index_select_estimator,5), ':k', 'linewidth', 2);hold on;
plot(gps.timestamp(index_select_estimator), gps.Ve(index_select_estimator), '--b', 'linewidth', 2);
xlim(timeselect);
ylabel('v (m/s)');
legend('measurement', 'state','gps(Ve)');

subplot(616);
plot(estimator.timestamp(index_select_estimator), ...
    estimator.measurement(index_select_estimator,6), '-r', 'linewidth', 2);
hold on;
plot(estimator.timestamp(index_select_estimator), ...
    estimator.state(index_select_estimator,6), ':k', 'linewidth', 2);
ylabel('r (rad/s)');
xlim(timeselect);
legend('measurement', 'state');

    
figure(2);
title('trajectory')
plot(estimator.state(index_select_estimator,2), ...
    estimator.state(index_select_estimator,1),'linewidth', 2); hold on;
% plot([351063],[3433892],'ro'); 
% x_min=min(min(estimator.state(index_select_estimator,2)),min(planner.waypoint0(:,2)));
% x_min=min(x_min,min(planner.waypoint1(:,2)));
% x_max=max(max(estimator.state(index_select_estimator,2)),max(planner.waypoint0(:,2)));
% x_max=max(x_max,max(planner.waypoint1(:,2)));
% y_min=min(min(estimator.state(index_select_estimator,1)),min(planner.waypoint0(:,1)));
% y_min=min(y_min,min(planner.waypoint1(:,1)));
% y_max=max(max(estimator.state(index_select_estimator,1)),max(planner.waypoint0(:,1)));
% y_max=max(y_max,max(planner.waypoint1(:,1)));
% xlim([0.95*x_min 1.05*x_max]);
% ylim([0.95*y_min 1.05*y_max]);
ylabel('x (m)');
xlabel('y (m)');

axis equal;

% figure(3);
% for i=1:m
%     subplot(m,1,i);
%     plot(controller.timestamp, controller.rpm(:,i),'linewidth', 2); 
%     ylabel(['rotation(rpm) ',num2str(i)]);
% end



function head=restrictheading(heading)
    if heading>180
        head=heading-360;
    elseif heading<-180
        head=heading+360;
    else
        head =heading;  
    end
    
end

