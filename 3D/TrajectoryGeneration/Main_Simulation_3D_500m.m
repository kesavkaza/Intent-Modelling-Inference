clear
close all
clc

% Define 3D Environment which contains Geofence, Obstacles
% Obstacles are represented by 1 and empty space by 0

GRIDSIZE = [500,500,100]; %[1000,1000,200]%
EMBSIZE = 10; %round(0.05*GRIDSIZE(1)); % We are adding embankments at the end in two directions

Edim = 3;        % environment dimension, 2D

Grange.x = [401, 500]; %[round(GRIDSIZE(1)*0.8), GRIDSIZE(1)]; % Geofence x range
Grange.y = [401, 500]; %[round(GRIDSIZE(2)*0.8), GRIDSIZE(2)]; % Geofence y range
Grange.z = [1, 25]; %[1, round(0.25*GRIDSIZE(3))]; % Geofence z range


O3_range.x = [101,200]; %[round(GRIDSIZE(1)*0.2),round(GRIDSIZE(1)*0.4)];
O3_range.y = [101,200]; %[round(GRIDSIZE(2)*0.2),round(GRIDSIZE(2)*0.4)];
O3_range.z = [1, 25]; %[1, round(0.25*GRIDSIZE(3))];

% Oranges = {O1_range, O2_range, O3_range};  % ranges of obstacles - dictionary

%load Env_3D_500m.mat 

map3D = occupancyMap3D;
%Create a ground plane and set occupancy values to 0.
[xGround,yGround,zGround] = meshgrid(0:GRIDSIZE(1),0:GRIDSIZE(2),0:GRIDSIZE(3));
xyzGround = [xGround(:) yGround(:) zGround(:)];
occval = 0;
setOccupancy(map3D,xyzGround,occval)

%Create obstacles in specific world locations of the map.
[xBuilding1,yBuilding1,zBuilding1] = meshgrid(O3_range.x(1):O3_range.x(2),O3_range.y(1):O3_range.y(2),O3_range.z(1):O3_range.z(2));

xyzBuildings = [xBuilding1(:) yBuilding1(:) zBuilding1(:)];
obs = 1;
updateOccupancy(map3D,xyzBuildings,obs)
show(map3D)

inflate(map3D,15)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
import navigation.*

% we are testing rrt algorithm

 %ss = stateSpaceSE3;
 ss = stateSpaceSE3([-10 GRIDSIZE(1); -10 GRIDSIZE(2); -10 GRIDSIZE(3); -Inf Inf; -Inf Inf; -Inf Inf; -Inf Inf]);
 %sv = validatorOccupancyMap3d(ss);
 sv = validatorOccupancyMap3D(ss, Map = map3D, ValidationDistance = 1.5);
 %map3D = occupancyMap3D;
% 
 planner = plannerRRT(ss,sv,MaxConnectionDistance=1.5);

% start = quat(1,1,20,1); %[1, 1, 1, quat(1,1,1,0)];
% %goal = quat(randi([Grange.x(1)+5, Grange.x(2)-10],1), randi([Grange.y(1)+5, Grange.y(2)-10]), randi([Grange.z(1)+5, Grange.z(2)-5]),1)
% goal = quat(180,180,40,1);%quat(180,180,30,1); %[180, 180, 30, quat(180,180,30,0)];
% % 
% tic
% [pthObj, solnInfo] = plan(planner,start,goal);
% toc


% show(map3D_all)
% axis equal
% view([10 15])
% hold on
% % Start state
% scatter3(start(1,1),start(1,2),start(1,3),"g","filled")
% % Goal state
% scatter3(goal(1,1),goal(1,2),goal(1,3),"r","filled")
% % Path
% plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3),"r-",LineWidth=2)
% hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% Generating Trajectories for various intents %%%%

%% Direct Attack

vel_range_DA_1 = [5, 11];
vel_range_DA_2 = [12, 15];
gran_init = 10;
Nint = 3; % number of critical waypoints
Nz = 8; % internal parameter, Nz/2 z values are randomly selected for each x,y, see trajectory generation code below
plannerDA = plannerRRT(ss, sv, MaxConnectionDistance = 0.1*vel_range_DA_2(2));
tic
[DirectAttackTrajectories, traj_lensDA, init_vels, changed_vels, vel_changepoints] = generateTrajectoriesDirectAttack_3D_500m(plannerDA, Nint, Nz, Grange, GRIDSIZE, gran_init, vel_range_DA_1, vel_range_DA_2);
toc
Trajectories = DirectAttackTrajectories;
save DirectAttackTrajectories_3D_500m Trajectories map3D Grange init_vels changed_vels vel_changepoints vel_range_DA_1 vel_range_DA_2

%% Harmless

vel_range_HL_1 = [5, 15];
vel_range_HL_2 = [5, 15];

gran_init = 10;
Nint = 3; % number of critical waypoints
plannerHL = plannerRRT(ss, sv, MaxConnectionDistance = 0.1*vel_range_HL_2(2));
Nz = 8;
[HarmlessTrajectories, traj_lensHL, init_vels, changed_vels, vel_changepoints] = generateTrajectoriesHarmless_3D_500m(plannerHL, Nint, Nz, GRIDSIZE, gran_init, vel_range_HL_1, vel_range_HL_2);

Trajectories = HarmlessTrajectories;
save HarmlessTrajectories_3D_500m Trajectories map3D Grange init_vels changed_vels vel_changepoints vel_range_HL_1 vel_range_HL_2 

%% Surveillance

vel_range_SL_1 = [5, 12];
vel_range_SL_2 = [9, 15];
gran_init = 10;
Nint = 3; % number of critical waypoints
Nz = 8; % internal parameter, Nz/2 z values are randomly selected for each x,y, see trajectory generation code below
plannerSL = plannerRRT(ss, sv, MaxConnectionDistance = 0.1*vel_range_SL_2(2));
% 
[SurveillanceTrajectories, traj_lensSur, init_vels, changed_vels, vel_changepoints] = generateTrajectoriesSurveillance_3D_500m(plannerSL, Nint, Nz, GRIDSIZE,  gran_init, vel_range_SL_1, vel_range_SL_2);

Trajectories = SurveillanceTrajectories;
save SurveillanceTrajectories_3D_500m Trajectories map3D Grange init_vels changed_vels vel_changepoints vel_range_SL_1 vel_range_SL_2 

%% Visualization

map3D_all = copy(map3D);
[xGfence, yGfence, zGfence] = meshgrid(Grange.x(1):Grange.x(2), Grange.y(1):Grange.y(2), Grange.z(1):Grange.z(2));
xyzGfence = [xGfence(:), yGfence(:), zGfence(:)];
occval = 0.8;
setOccupancy(map3D_all, xyzGfence, occval)

load DirectAttackTrajectories_3D_500m.mat
visualize_trajectories_3D(map3D_all, Trajectories, 'Direct Attack Trajectories')

load HarmlessTrajectories_3D_500m.mat
visualize_trajectories_3D(map3D_all, Trajectories, 'Harmless Trajectories')

load SurveillanceTrajectories_3D_500m.mat
visualize_trajectories_3D(map3D_all, Trajectories, 'Surveillance Trajectories')