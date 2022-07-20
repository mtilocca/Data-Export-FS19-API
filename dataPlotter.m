
clear all; 
close all;
clc;

%%  Importing the data 

record = import_ipessalog('simulatorlog.txt');
%% select what you want to plot 

pSteer = true;
pVel= true;
pPositions = true;
pRPM = true;
pAngleRate = true;

%% passing the data to plot 
Testdata = Data;
Testdata.record = record;

%% WGS84 conversion

lat0 = 39.233317;
lon0 = 9.117279;
lat0 = deg2rad(lat0);
lon0 = deg2rad(lon0);
h0 = 75.2;

xNorth = -record.z;
yEast = -record.x;
zDown = -record.y;

ned = [xNorth, yEast, zDown];
ref = [lat0, lon0, h0];

curv= nFrame2curvilinearECEF([xNorth, yEast, zDown]',[lat0, lon0, h0]');

[lat, lon, h]=Testdata.plotWGS84;


%% plots 

if pSteer
Testdata.plotSteer;
end 

if pVel
    Testdata.plotVelocities;
end 

if pPositions
Testdata.plotpos;
end 

if pRPM
Testdata.plotRPM;
end 

if pAngleRate
    Testdata.plotAngleRate;
end 

