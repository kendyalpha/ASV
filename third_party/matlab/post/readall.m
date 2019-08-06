close all;
clear;


timeselect=[10 1500]; % second
Path='20190723/';
% read sql file
controllerdata=csvread(strcat(Path,'controller.csv'),1,0);
estimatordata=csvread(strcat(Path,'estimator.csv'),1,0);
plannerdata=csvread(strcat(Path,'planner.csv'),1,0);
indicatordata=csvread(strcat(Path,'indicator.csv'),1,0);
% gpsdata = csvread(strcat(Path,'gps.csv'),1,0);
gpsdata = readgpsnew(strcat(Path,'gps.csv'));
winddata = csvread(strcat(Path,'wind.csv'),1,0);

timestamp0=min(controllerdata(1,2),estimatordata(1,2));
timestamp0=min(timestamp0,plannerdata(1,2));
timestamp0=min(timestamp0,gpsdata.DATETIME(1));
timestamp0=min(timestamp0,indicatordata(1,2));
timestamp0=min(timestamp0,winddata(1,2));


%% resample 
resample_step=0.1;
resample_time=timeselect(1):0.1:timeselect(2);

%% controller
timestamp_controller=(controllerdata(:,2)-timestamp0)*86400.0;
rcontrollerdata=zeros(size(resample_time,2),18);
for i=3:20 
    rcontrollerdata(:,i-2)=interp1(timestamp_controller,controllerdata(:,i),resample_time,'linear'); 
end
controller.timestamp=resample_time;
controller.tau=rcontrollerdata(:,1:3);
controller.alpha=rcontrollerdata(:,4:9);
controller.rpm=rcontrollerdata(:,10:15);
controller.est=rcontrollerdata(:,16:18);


%% estimator
timestamp_estimator=(estimatordata(:,2)-timestamp0)*86400.0;
restimatordata=zeros(size(resample_time,2),18);
for i=3:20
    restimatordata(:,i-2)=interp1(timestamp_estimator,estimatordata(:,i),resample_time,'linear'); 
end
estimator.timestamp=resample_time;
estimator.measurement=restimatordata(:,1:6);
estimator.state=restimatordata(:,7:12);
estimator.perror=restimatordata(:,13:15);
estimator.verror=restimatordata(:,16:18);


%% GPS
timestamp_gps=(gpsdata.DATETIME-timestamp0)*86400.0;
gps.timestamp=resample_time;
gps.heading=interp1(timestamp_gps,gpsdata.heading,resample_time,'linear');
gps.pitch=interp1(timestamp_gps,gpsdata.pitch,resample_time,'linear');
gps.roll=interp1(timestamp_gps,gpsdata.roll,resample_time,'linear');
gps.Ve=interp1(timestamp_gps,gpsdata.Ve,resample_time,'linear');
gps.Vn=interp1(timestamp_gps,gpsdata.Vn,resample_time,'linear');
gps.x=interp1(timestamp_gps,gpsdata.UTM_x,resample_time,'linear');
gps.y=interp1(timestamp_gps,gpsdata.UTM_y,resample_time,'linear');


%% indicator
timestamp_indicator=(indicatordata(:,2)-timestamp0)*86400.0;
rindicatordata=zeros(size(resample_time,2),4);
for i=3:6
    rindicatordata(:,i-2)=interp1(timestamp_indicator,indicatordata(:,i),resample_time,'nearest'); 
end

indicator.timestamp=resample_time;
indicator.gui_connection=rindicatordata(:,1);
indicator.joystick_connection=rindicatordata(:,2);
indicator.indicator_controlmode=rindicatordata(:,3);
indicator.indicator_windstatus=rindicatordata(:,4);


%% planner
timestamp_planner=(plannerdata(:,2)-timestamp0)*86400.0;
rplannerdata=zeros(size(resample_time,2),13);
for i=3:15
    rplannerdata(:,i-2)=interp1(timestamp_planner,plannerdata(:,i),resample_time,'linear'); 
end
planner.timestamp=resample_time;
planner.setpoint=rplannerdata(:,1:3);
planner.v_setpoint=rplannerdata(:,4:6);
planner.command=rplannerdata(:,7:9);
planner.waypoint0=rplannerdata(:,10:11);
planner.waypoint1=rplannerdata(:,12:13);

%% wind 
timestamp_wind=(winddata(:,2)-timestamp0)*86400.0;
rwinddata=zeros(size(resample_time,2),2);
for i=3:4
    rwinddata(:,i-2)=interp1(timestamp_wind,winddata(:,i),resample_time,'linear'); 
end
wind.timestamp=resample_time;
wind.speed=rwinddata(:,1);
wind.orientation=rwinddata(:,2);
save(strcat(Path,'data.mat'),'controller','estimator','gps',...
                           'indicator','planner', 'wind');