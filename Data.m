classdef Data 
  properties
      record 
      step = 0.0167 
  end
  
    methods
 
     function plotpos(obj)
t = obj.record.frame .*0.0167;

%t = gett(obj);
figure( 'Name','x-z position Cartesian' ,'NumberTitle','off'), clf   
subplot(2,1,1)
plot(t,obj.record.x, 'b') 
hold on% position relative to global node 
grid on;
grid minor;
xlabel('time[s]');
ylabel('x-axis position [m] ');
legend('x','Location','southeast'); 
title('change of position in latitude  cartesian coordinates');

subplot(2,1,2)
plot(t,obj.record.z, 'k')
grid on;
grid minor;
xlabel('time[s]');
ylabel('z-axis position [m]');
legend('z','Location','southeast'); 
title('change of position in longitude cartesian coordinates');


figure( 'Name','y Cartesian' ,'NumberTitle','off'), clf   
plot(t,obj.record.y, 'b')
grid on;
grid minor;
xlabel('time[s]');
ylabel('y-axis position ');
legend('y', 'Location','southeast'); 
title('change of position in altitude cartesian coordinates');


figure( 'Name','WGS84 latitude and longitude' ,'NumberTitle','off'), clf   
subplot(3,1,1)
plot(t,rad2deg(obj.record.latitude), 'b') 
hold on% position relative to global node 
grid on;
grid minor;
xlabel('time[s]');
ylabel('latitude');
legend('latitude','Location','southeast'); 
title('WGS84 Latitude change');

subplot(3,1,2)
plot(t,rad2deg(obj.record.longitude), 'k')
grid on;
grid minor;
xlabel('time[s]');
ylabel('longitude');
legend('longitude','Location','southeast'); 
title('WGS84 Longitude change');

subplot(3,1,3)
plot(t,obj.record.height, 'k')
grid on;
grid minor;
xlabel('time[s]');
ylabel('height [m]');
legend('height','Location','southeast'); 
title('WGS84 Height change');

        end

        function plotVelocities(obj)
  

t = obj.record.frame .*0.0167;
init = [0];

longVel = diff(obj.record.z);
longVel = [init;longVel(1:end)];
longVel = longVel ./ 0.0167;

figure( 'Name','Vehicle velocity' ,'NumberTitle','off'), clf   
plot(t,obj.record.speed, 'm')
hold on 
grid on;
grid minor;
xlabel('time[s]');
ylabel(' velocity [m/s] ');
legend('VelocityS','Location','southeast'); 
title('Vehicle velocity');


latVel = diff(obj.record.x);
latVel = [init;latVel(1:end)];
latVel = latVel ./ 0.0167;

figure( 'Name','Vehicle lateral velocity' ,'NumberTitle','off'), clf   
plot(t,latVel, 'm')
hold on 
grid on;
grid minor;
xlabel('time[s]');
ylabel(' velocity [m/s] ');
legend('Velocity', 'Location','southeast'); 
title('Vehicle lateral velocity');
end 

function plotAngleRate(obj)
t = obj.record.frame .*0.0167;

% yaw --> thetaY 
% pitch --> thetaX
% roll --> thetaZ 

yaw = diff(obj.record.ThetaY); 
pitch = diff(obj.record.ThetaX);
roll = diff(obj.record.ThetaZ);

init = [0];
yaw = [init;yaw(1:end)]; 
pitch = [init;pitch(1:end)]; 
roll = [init;roll(1:end)]; 


figure( 'Name','Yaw rate' ,'NumberTitle','off'), clf   
plot(t,yaw, 'm')
hold on 
grid on;
grid minor;
xlabel('time[s]');
ylabel(' yaw rate [rad/s] ');
legend('yaw ', 'Location','southeast'); 
title('Vehicle yaw rate');



figure( 'Name','pitch rate' ,'NumberTitle','off'), clf   
plot(t,pitch, 'r')
hold on 
grid on;
grid minor;
xlabel('time[s]');
ylabel(' pitch rate [rad/s] ');
legend('pitch', 'Location','southeast'); 
title('Vehicle pitch rate');



figure( 'Name','roll rate' ,'NumberTitle','off'), clf   
plot(t,roll, 'b')
hold on 
grid on;
grid minor;
xlabel('time[s]');
ylabel(' roll rate [rad/s] ');
legend('roll', 'Location','southeast'); 
title('Vehicle roll rate');

end 

function plotRPM(obj)

t = obj.record.frame .*0.0167;

figure( 'Name','Motor RPM' ,'NumberTitle','off'), clf   
plot(t,obj.record.rpm, 'r')
grid on;
grid minor; 
xlabel('time[s]');
ylabel('motor rpm [rev/min]');
legend('rpm', 'Location','southeast'); 
title('Motor rpm');


figure( 'Name','All wheels RPMs' ,'NumberTitle','off'), clf   
plot (t, obj.record.rpmFL, 'b')
grid on;
grid minor;
hold on
plot(t, obj.record.rpmFR, 'k')
plot(t, obj.record.rpmRR, 'm')
plot(t, obj.record.rpmRL, 'g')
xlabel('time[s]');
ylabel('wheels rpm [rev/min]');
legend( 'rpmFL','rpmFR','rpmRR','rpmRL', 'Location','southeast'); 
title(' wheels rpm');

figure( 'Name','Front vs Rear wheels RPMs' ,'NumberTitle','off'), clf   
subplot(2,1,1)
plot(t,obj.record.rpmFL, 'r')
hold on
grid on;
grid minor;
plot(t, obj.record.rpmFR, 'k')
xlabel('time[s]');
ylabel('front wheels rpm [rev/min]');
legend('rpmFL','rpmFR', 'Location','southeast'); 
title('front wheels rpm');

subplot(2,1,2)
plot(t, obj.record.rpmRL,'g')
hold on 
grid on
grid minor 
plot(t, obj.record.rpmRR, 'm')
xlabel('time[s]');
ylabel('rearwheels angular speed [rev/min]');
legend('rpmRL','rpmRR', 'Location','southeast'); 
title('rear wheels rpm');
end 

function plotSteer(obj)

t = obj.record.frame .*0.0167;

figure( 'Name','Front wheels steering angles' ,'NumberTitle','off'), clf   
plot(t,obj.record.alphaFR, 'm')
hold on 
plot(t,obj.record.alphaFL, 'k')
grid on;
grid minor;
xlabel('time[s]');
ylabel(' steering angle [deg] ');
legend('Right', 'Left ', 'Location','southeast'); 
title('front wheels steering behavior');

figure( 'Name','Vehicle axle steering angle' ,'NumberTitle','off'), clf   
plot(t,obj.record.steer)
grid on;
grid minor;
xlabel('time[s]');
ylabel(' steering angle [deg] ');
legend('Axle angle', 'Location','southeast'); 
title('Axle steering angle');
end

function [lat, lon, h]=plotWGS84(obj)

t = obj.record.frame .*0.0167;
lat0 = 60.183841;
lon0 = 24.814339;
h0 = 9.0;

xNorth = -obj.record.z;
yEast = -obj.record.x;
zDown = -obj.record.y;

[lat,lon,h] = ned2geodetic(xNorth,yEast,zDown,lat0,lon0,h0,wgs84Ellipsoid);

figure( 'Name','WGS84 latitude and longitude Matlab built in' ,'NumberTitle','off'), clf   
subplot(3,1,1)
plot(t,lat, 'b') 
hold on% position relative to global node 
grid on;
grid minor;
xlabel('time[s]');
ylabel('latitude');
legend('latitude','Location','southeast'); 
title('WGS84 Latitude change matlab built in ');

subplot(3,1,2)
plot(t,lon, 'k')
grid on;
grid minor;
xlabel('time[s]');
ylabel('longitude');
legend('longitude','Location','southeast'); 
title('WGS84 Longitude change matlab built in');

subplot(3,1,3)
plot(t,h, 'k')
grid on;
grid minor;
xlabel('time[s]');
ylabel('height [m]');
legend('height','Location','southeast'); 
title('WGS84 Height change');


end 

end 

end

