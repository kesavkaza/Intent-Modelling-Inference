function [alltrajectories, traj_lens, initial_vels, changed_vels, vel_changepoints] = generateTrajectoriesSurveillance_2Dv1000_variablespeed(grid, Nint, gran_init, vel_range1, vel_range2)
% Generate trajectories, the average speed changes at one critical waypoint.
% However, each segment of trajectory has a constant average speed picked from vel_range
% Nint <= Number of critical regions (see Zones variable below)
    
% Define the intermediate destination regions for surveillance intent
    
    Range1.x = [500, 950]; %[500, 950]; %[100, 190];
    Range1.y = [50, 250]; %[1, 200]; %[1, 50];
    Range2.x = [500, 950]; %[100, 190];
    Range2.y = [550, 650]; %[100, 125];
    Range3.x = [550, 650]; %[100, 125];
    Range3.y = [500, 950]; %[100, 190];
    Range4.x = [50, 250]; %[1, 200]; %[1, 50];
    Range4.y = [500, 950]; %[100, 190];
    Zones = {Range1, Range2, Range3, Range4};
    %Zones = {Range4, Range3, Range2, Range1};

    GRIDSIZE = size(grid, 1);
   
    % using navigation toolbox
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    map=occupancyMap(logical(grid),1);
    sv.Map = map;
    show(map)
    sv.ValidationDistance = 1;
    ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
    planner = plannerRRT(ss, sv, MaxConnectionDistance = vel_range2(2)/10);

    %[pthObj, solnInfo] = plan(planner, [1,1], [180,180]);

    alltrajectories = {};
    initial_vels = {};
    changed_vels = {};
    vel_changepoints = {};
    traj_lens = [];
    

    for x = 1:gran_init:9*GRIDSIZE/10
        goals = {};
        path = []; %{};
        y = 1;
        for n = 1:Nint
            zone = Zones{n};
            goal1 = [randi([zone.x(1), zone.x(2)]), randi([zone.y(1), zone.y(2)]), randi([-90,90])];
            goals{n} = goal1;
            %goals{end+1} = goal1;
        end

        ttx = randi([0,90]);
        vinit = vel_range1(1) + diff(vel_range1)*rand;
        if vinit< vel_range2(1)
            vnew = vel_range2(1) + diff(vel_range2)*rand;
        else
            vnew = vinit;
        end

        strt = [x, y, ttx];
        for i = 1:length(goals)
            goal1 = goals{i};
            %path = [astar(grid, strt, goal1), path];
            [pthObj, solnInfo] = plan(planner, strt, goal1);
            path = [path; pthObj.States(:,1:2)];
            strt = goal1;
        end

        %path = flip(path);
        alltrajectories{end+1} = path;
        traj_lens = [traj_lens, length(path)];
        initial_vels{end+1} = [vinit*cosd(ttx), vinit*sind(ttx)];
        vel_changepoints{end+1} = goals{2}; 
        changed_vels{end+1} = [vnew*cosd(goals{2}(3)), vnew*sind(goals{2}(3))];

    end

    for y = 1:gran_init:9*GRIDSIZE/10
        goals = {};
        path = []; %{};
        x = 1;
        for n = 1:Nint %Nint:-1:1
            zone = Zones{numel(Zones)-n+1}; %Zones{n};
            goal1 = [randi([zone.x(1), zone.x(2)]), randi([zone.y(1), zone.y(2)]), randi([-90,90])];
            goals{n} = goal1;
            %goals{end+1} = goal1;
        end

        tty = randi([0,90]);
        vinit = vel_range1(1) + diff(vel_range1)*rand;
        if vinit < vel_range2(1)
            vnew = vel_range2(1) + diff(vel_range2)*rand;
        else
            vnew = vinit;
        end

        strt = [x, y, tty];
        for i = 1:length(goals)
            goal1 = goals{i};
            %path = [astar(grid, strt, goal1), path];
            [pthObj, solnInfo] = plan(planner, strt, goal1);
            path = [path; pthObj.States(:,1:2)];
            strt = goal1;
        end

        %path = flip(path);
        alltrajectories{end+1} = path;
        traj_lens = [traj_lens, length(path)];
        initial_vels{end+1} = [vinit*cosd(tty), vinit*sind(tty)];
        vel_changepoints{end+1} = goals{2}; 
        changed_vels{end+1} = [vnew*cosd(goals{2}(3)), vnew*sind(goals{2}(3))];
        
    end
end

