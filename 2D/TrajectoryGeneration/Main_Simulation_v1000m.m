%% Main simulation file for 2D environment - 1000 X 1000 meters

clear
close all
clc

% Define 2D Environment which contains Geofence, Obstacles
% Obstacles are represented by 1 and empty space by 0

GRIDSIZE = 1000;
EMBSIZE = 10; %round(0.01*GRIDSIZE); % We are adding embankments at the end on two sides

%lmin = -40; % We start at -40m, so the radar tracker can stabilize before reaching the positive quadrant
Edim = 2;        % environment dimension, 2D
Erange.x = [1, GRIDSIZE+EMBSIZE]; % range along xaxis
Erange.y = [1, GRIDSIZE+EMBSIZE]; 
Egrid = zeros(Erange.y(2)-Erange.y(1), Erange.x(2)-Erange.x(1));  % Initialize the grid

Grange.x = [round(GRIDSIZE*0.8), GRIDSIZE]; % Geofence x range
Grange.y = [round(GRIDSIZE*0.8), GRIDSIZE]; % Geofence y range

Gpoints_all = [];   % list of all geofence points
for i = Grange.x(1):Grange.x(2)
  for j = Grange.y(1):Grange.y(2)
    Gpoints_all = [Gpoints_all; i j];
    %Egrid(i,j) = 1;
  end
end

O1_range.x = [1, GRIDSIZE+EMBSIZE]; % Embankment 1
O1_range.y = [GRIDSIZE, GRIDSIZE+EMBSIZE];
O2_range.x = [GRIDSIZE, GRIDSIZE+EMBSIZE]; % Embankment 2
O2_range.y = [1,GRIDSIZE];
O3_range.x = [round(GRIDSIZE*0.2),round(GRIDSIZE*0.4)];
O3_range.y = [round(GRIDSIZE*0.2),round(GRIDSIZE*0.4)];

Oranges = {O1_range, O2_range, O3_range};  % ranges of obstacles - dictionary

Opoints_all = [];
for obs = 1:length(Oranges)
  for i = Oranges{obs}.x(1):Oranges{obs}.x(2)
    for j = Oranges{obs}.y(1):Oranges{obs}.y(2)
      Egrid(i,j) = 1;
      Opoints_all = [Opoints_all; i j];
    end
  end
end

Grid = Egrid; % grid contains artificially expanded obstacles.
              % This makes the algorithm maintain a minimum distance from the obstacle.
rdist = 25;
for obs = 1:2
  for i = max([Oranges{obs}.x(1)-rdist,1]):min([Oranges{obs}.x(2)+rdist,GRIDSIZE+EMBSIZE])
    for j = max([Oranges{obs}.y(1)-rdist,1]):min([Oranges{obs}.y(2)+rdist,GRIDSIZE+EMBSIZE])
      %for k = max([Oranges{obs}.z(1)-5,1]):min([Oranges{obs}.z(2)+5,GRIDSIZE])
      Grid(i,j) = 1;
      %end
    end
  end
end

radius = 1.2*(O3_range.x(2)-O3_range.x(1))/sqrt(2);
O3center = [ (O3_range.x(1)+O3_range.x(2))/2 , (O3_range.y(1)+O3_range.y(2))/2 ];
for i = 1:(GRIDSIZE+EMBSIZE)
  for j = 1:(GRIDSIZE+EMBSIZE)
    if sqrt((O3center(1)-i)^2 + (O3center(2)-j)^2 ) <= radius
      Grid(i,j) = 1;
    end
  end
end

Grid=rot90(Grid);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
import navigation.*

% % we are testing rrt algorithm
% ss = stateSpaceSE2;
% sv = validatorOccupancyMap(ss);
% map=occupancyMap(logical(Grid),1);
% sv.Map = map;
% %show(map)
% sv.ValidationDistance = 1;
% ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
% % planner = plannerRRT(ss,sv,MaxConnectionDistance=1);
% planner = plannerRRT(ss,sv);
% planner.MaxConnectionDistance = 1;
% 

%%%% Generating Trajectories for various intents %%%%

rng("shuffle")

%% Direct Attack
vel_range_DA_1 = [5, 11]; vel_range_DA_2 = [12, 15];
gran_init = 10;
Nint = 3;
mode = 0;
%[DirectAttackTrajectories, traj_lensDA, init_vels_DA] = generateTrajectoriesDirectAttack_2Dv1000(Grid, Nint, gran_init, vel_range_DA, mode);
[DirectAttackTrajectories, traj_lensDA, init_vels_DA, changed_vels, vel_changepoints] = generateTrajectoriesDirectAttack_2Dv1000_variablespeed(Grid, Nint, gran_init, vel_range_DA_1, vel_range_DA_2, mode);

%load DirectAttackTrajectory_2D_1000m_varspeed_indirect.mat
visualize_trajectories_scatter(Trajectories, Gpoints_all, Opoints_all, 'Direct Attack Trajectories')

Trajectories = DirectAttackTrajectories;
if mode==1
    % mode 1 - attack pattern will be CR1 to CR2 to G
    save DirectAttackTrajectory_2D_1000m_varspeed_indirect Trajectories Gpoints_all Opoints_all init_vels_DA changed_vels vel_changepoints
else
    % mode != 1 - attack pattern CR1 to G
    save DirectAttackTrajectory_2D_1000m_varspeed_direct Trajectories Gpoints_all Opoints_all init_vels_DA changed_vels vel_changepoints
end
%% Harmless
vel_range_HL = [5, 15];
%vel_range_HL_2 = [5, 15];

gran_init = 10;
Nint = 3; % number of critical waypoints
%[HarmlessTrajectories, traj_lensHL, init_vels_HL] = generateTrajectoriesHarmless_2Dv1000(Grid, Nint, gran_init, vel_range_HL);
[HarmlessTrajectories, traj_lensHL, init_vels_HL, changed_vels, vel_changepoints] = generateTrajectoriesHarmless_2Dv1000_variablespeed(Grid, Nint, gran_init, vel_range_HL, vel_range_HL);

%load HarmlessTrajectory_2D_1000m_varspeed.mat
visualize_trajectories_scatter(Trajectories, Gpoints_all, Opoints_all, 'Harmless Trajectories')

Trajectories = HarmlessTrajectories;
save HarmlessTrajectory_2D_1000m_varspeed Trajectories Gpoints_all Opoints_all init_vels_HL changed_vels vel_changepoints
% %save HarmlessTrajectory_2D_1000m Trajectories Gpoints_all Opoints_all init_vels_HL

%% Surveillance
vel_range_SL_1 = [5, 15]; vel_range_SL_2 = [12, 15];
gran_init = 10;
Nint = 3; % number of critical waypoints
%[SurveillanceTrajectories, traj_lensSur, init_vels_SL] = generateTrajectoriesSurveillance_2Dv1000_constspeed(Grid, Nint, gran_init, vel_range_SL);
[SurveillanceTrajectories, traj_lensSur, init_vels_SL, changed_vels_SL, vel_changepoints_SL] = generateTrajectoriesSurveillance_2Dv1000_variablespeed(Grid, Nint, gran_init, vel_range_SL_1, vel_range_SL_2);

%load SurveillanceTrajectory_2D_1000m_varspeed.mat
visualize_trajectories_scatter(Trajectories, Gpoints_all, Opoints_all, 'Surveillance Trajectories')

Trajectories = SurveillanceTrajectories;
save SurveillanceTrajectory_2D_1000m_varspeed Trajectories Gpoints_all Opoints_all init_vels_SL changed_vels_SL vel_changepoints_SL
% % save SurveillanceTrajectory_2D_1000m_constspeed Trajectories Gpoints_all Opoints_all init_vels_SL
