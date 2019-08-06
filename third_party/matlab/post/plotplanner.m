close all
clear;

Path='20190723/';
load(strcat(Path,'data.mat'));

m=size(controller.alpha,2);

timeselect=[100 1000]; % second
index_select_planner=find(timeselect(1)<=planner.timestamp ...
    & planner.timestamp<= timeselect(2));


figure(1); 
title('setpoint')
% subplot(311);
% plot(planner.timestamp(index_select_planner), ...
%     planner.setpoint(index_select_planner,1), '-r', 'linewidth', 2);
% hold on;
% plot(planner.timestamp(index_select_controller), ...
%     planner.tau(index_select_controller,1), ':k', 'linewidth', 2);
% ylabel('taux(N)');
% legend('estimated force', 'desired force');
% subplot(312); 
% plot(controller.timestamp(index_select_controller), ...
%     controller.est(index_select_controller,2),'-r', 'linewidth', 2);
% hold on;
% plot(controller.timestamp(index_select_controller), ...
%    controller.tau(index_select_controller,2), ':k', 'linewidth', 2);
% ylabel('tauy(N)') 
% legend('estimated force', 'desired force') 
subplot(313);
plot(planner.timestamp(index_select_planner), ...
    planner.setpoint(index_select_planner,3), '-r', 'linewidth', 2);

ylabel('taun(N m)')