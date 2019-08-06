clear;
close all;

Path='20190723/';
load(strcat(Path,'data.mat'));

index_actuation=0; % underactuated=1
m=size(controller.alpha,2);


timeselect=[860 910]; % second
index_select_estimator=find(timeselect(1)<=estimator.timestamp ...
    & estimator.timestamp<= timeselect(2));
index_select_controller=find(timeselect(1)<=controller.timestamp ...
    & controller.timestamp<= timeselect(2));


figure(1); 
subplot(311);
plot(controller.timestamp(index_select_controller), ...
    controller.est(index_select_controller,1), '-r', 'linewidth', 2);
hold on;
plot(controller.timestamp(index_select_controller), ...
    controller.tau(index_select_controller,1), ':k', 'linewidth', 2);
ylabel('taux(N)');
legend('estimated force', 'desired force');
subplot(312); 
plot(controller.timestamp(index_select_controller), ...
    controller.est(index_select_controller,2),'-r', 'linewidth', 2);
hold on;
plot(controller.timestamp(index_select_controller), ...
   controller.tau(index_select_controller,2), ':k', 'linewidth', 2);
ylabel('tauy(N)') 
legend('estimated force', 'desired force') 
subplot(313);
plot(controller.timestamp(index_select_controller), ...
    controller.est(index_select_controller,3), '-r', 'linewidth', 2);
hold on;
plot(controller.timestamp(index_select_controller), ...
    controller.tau(index_select_controller,3), ':k', 'linewidth', 2);
legend('estimated force', 'desired force');
ylabel('taun(N m)')

    
figure(2);
for i=1:m
    subplot(m,1,i);
    plot(controller.timestamp(index_select_controller),...
        controller.alpha(index_select_controller,i),'linewidth', 2); 
    ylabel(['alpha(deg) ',num2str(i)]);
end

figure(3);
for i=1:m
    subplot(m,1,i);
    plot(controller.timestamp(index_select_controller), ...
        controller.rpm(index_select_controller,i),'linewidth', 2); 
    ylabel(['rotation(rpm) ',num2str(i)]);
end

figure(4)
subplot(311); 
plot(estimator.timestamp(index_select_estimator), ...
    estimator.perror(index_select_estimator,1), '-r', 'linewidth', 2);
ylabel('error-x(m)');
subplot(312); 
plot(estimator.timestamp(index_select_estimator), ...
    estimator.perror(index_select_estimator,2),'-r', 'linewidth', 2);
ylabel('error-y(m)') 
subplot(313);
plot(estimator.timestamp(index_select_estimator), ...
    estimator.perror(index_select_estimator,3), '-r', 'linewidth', 2);
ylabel('error-Mz (rad)')
