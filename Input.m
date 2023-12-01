
%%Imput files
load('multilayerMap.mat');

%Creating separate layers
road = map(:,:,1);
speedLimit = map(:,:,2);
trafficIntensity = map(:,:,3);
obstacleCost = map(:,:,4);

%creating the struct for the algorithm
mapped =struct();
mapped.road = road;
mapped.speedLimit = speedLimit;
mapped.trafficIntensity = trafficIntensity;
mapped.obstacleCost = obstacleCost;


map1 = occupancyMap(road);
