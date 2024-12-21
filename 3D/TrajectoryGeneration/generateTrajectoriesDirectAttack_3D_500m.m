function [alltrajectories, traj_lens, initial_vels, changed_vels, vel_changepoints] = generateTrajectoriesDirectAttack_3D_500m(planner, Nint, Nz, Grange, GRIDSIZE, gran_init, vel_range1, vel_range2)

% To generate harmless trajectories in a 500 x 500 x 100m environment.
% Each trajectory has maintain fixed average speed in the first segment,
% A new speed will be chosen at the second critical waypoint.

%%%%%%%%%%% Initial point regions %%%%%%%%%%%
% To avoid early intent recognition, all intent will have the same ranges
% for initial points
%Initial point regions % Not the same as critical regions
% x=1; y=1:gran_init:475; z is randomly sampled from [5,100]
% x=1:gran_init:475; y=1; z is randomly sampled from [5,100]


% Define the critical waypoint regions for harmless intent

Range1.x = [500, 900]/2; %[100, 160];
Range1.y = [40, 240]/2;  %[20, 50];
Range1.z = [20, 80];

Range2.x = [500, 900]/2; %[20, 50];
Range2.y = [530, 600]/2; %[100, 160];
Range2.z = [30, 60];

Range3.x = [530, 600]/2; %[100, 125];
Range3.y = [500, 900]/2; %[100, 190];
Range3.z = [30, 60];

Range4.x = [40, 240]/2;  %[1, 200]; %[1, 50];
Range4.y = [500, 900]/2; %[100, 190];
Range4.z = [20, 80];

%Zones = {Range1, Range2, Range3, Range4};
CROS1 = {Range1, Range2, Grange};
CROS2 = {Range4, Range3, Grange};

% using navigation toolbox
% ss = stateSpaceSE2;
% sv = validatorOccupancyMap(ss);
% map=occupancyMap(logical(grid),1);
% sv.Map = map;
% show(map)
% sv.ValidationDistance = 1;
% ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
% planner = plannerRRT(ss,sv,MaxConnectionDistance=1);

%[pthObj, solnInfo] = plan(planner, [1,1], [180,180]);


alltrajectories = {};
initial_vels = {};
changed_vels = {};
vel_changepoints = {};
traj_lens = [];

nzx = floorDiv(Nz,2);

for x = 2:gran_init:9*GRIDSIZE(1)/10
    y = 1;

    for zz = 1:nzx %5:gran_init:40
        z = randi([20, 100]);
        [x,y,z]
        goals = {};
        path = []; %{};

        ttx = randi([1,360]);
        strt = quat(x, y, z, ttx);

        for n = 1:Nint
            Zone = CROS1{n};%Zones{n};
            goal1 = quat(randi([Zone.x(1), Zone.x(2)]), randi([Zone.y(1), Zone.y(2)]), randi([Zone.z(1), Zone.z(2)]), randi([1,90]));
            goals{end+1} = goal1;
        end
        
        vinit = vel_range1(1) + diff(vel_range1)*rand;
        vnew = vel_range2(1) + diff(vel_range2)*rand;

        for i = 1:length(goals)
            goal1 = goals{i};
            %path = [astar(grid, strt, goal1), path];
            [pthObj, ~] = plan(planner, strt, goal1);
            path = [path; pthObj.States(:,1:3)];
            strt = goal1;
        end
    
        % path = flip(path);
        alltrajectories{end+1} = path;
        length(path)
        traj_lens = [traj_lens, length(path)];
        [roll, pitch, yaw] = calculateEulerAngles(strt(1), strt(2), strt(3));
        initial_vels{end+1} = [vinit*cos(roll), vinit*cos(pitch), vinit*cos(yaw)];
        vel_changepoints{end+1} = goals{2}; 
        changed_vels{end+1} = [vnew*cosd(goals{2}(4)), vnew*sind(goals{2}(4))];

    end
end

for y = 2:gran_init:9*GRIDSIZE(2)/10
    x = 1;
    for zz = 1:nzx %5:gran_init:40
        z= randi([20,100]);
        [x,y,z]
        goals = {};
        path = []; %{};

        for n = 1:Nint %Nint:-1:1
            Zone = CROS2{n}; %Zones{n};
            goal1 = quat(randi([Zone.x(1), Zone.x(2)]), randi([Zone.y(1), Zone.y(2)]),randi([Zone.z(1), Zone.z(2)]),randi([0,90]));
            goals{end+1} = goal1;
        end
    
        tty = randi([1,360]);
        vinit = vel_range1(1) + diff(vel_range1)*rand;
        vnew = vel_range2(1) + diff(vel_range2)*rand;

        strt = quat(x, y, z, tty);
        for i = 1:length(goals)
            goal1 = goals{i};
            %path = [astar(grid, strt, goal1), path];
            [pthObj, ~] = plan(planner, strt, goal1);
            path = [path; pthObj.States(:,1:3)];
            strt = goal1;
        end
    
        %path = flip(path);            
        alltrajectories{end+1} = path;
        length(path)
        traj_lens = [traj_lens, length(path)];
        [roll, pitch, yaw] = calculateEulerAngles(strt(1), strt(2), strt(3));
        initial_vels{end+1} = [vinit*cos(roll), vinit*cos(pitch), vinit*cos(yaw)];
        vel_changepoints{end+1} = goals{2}; 
        changed_vels{end+1} = [vnew*cosd(goals{2}(4)), vnew*sind(goals{2}(4))];

    end
end

end