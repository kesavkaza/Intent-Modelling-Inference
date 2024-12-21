function [alltrajectories, traj_lens, initial_vels, changed_vels, vel_changepoints] = generateTrajectoriesDirectAttack_2Dv1000_variablespeed(grid, Nint, gran_init, vel_range1, vel_range2, mode)

    % Range1.x = [500, 950]; %[100, 160];
    % Range1.y = [50, 600]; %[20, 50];
    % Range2.x = [50, 600]; %[20, 50];
    % Range2.y = [500, 950]; %[100, 160];
    % Zones = {Range1, Range2};
    
    GRIDSIZE = size(grid, 1);
    
    Range1.x = [450, 950]; %[500, 950]; %[100, 190];
    Range1.y = [40, 240];  %[1, 200];  %[1, 50];
    Range2.x = [450, 900]; %[100, 190];
    Range2.y = [550, 650]; %[100, 125];
    Range3.x = [550, 650]; %[100, 125];
    Range3.y = [450, 900]; %[100, 190];
    Range4.x = [40, 240];  %[1, 200];  %[1, 50];
    Range4.y = [450, 950]; %[100, 190];
    
    %Grange.x = [round(0.8*GRIDSIZE), GRIDSIZE-10];
    %Grange.y = [round(0.8*GRIDSIZE), GRIDSIZE-10];
    Grange.x = [800, 900]; %[round(0.8*GRIDSIZE), round(0.99*GRIDSIZE)-1];
    Grange.y = [800, 900]; %[round(0.8*GRIDSIZE), round(0.99*GRIDSIZE)-1];
    
    if mode==1
        Zones1 = {Range1, Range2, Grange};
        Zones2 = {Range4, Range3, Grange};
    else
        Zones1 = {Range1, Grange};
        Zones2 = {Range4, Grange};
        Nint = 2;
    end
    
    % Generate trajectories which end in the Geofence
    %rng('shuffle')
    % using navigation toolbox
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    map=occupancyMap(logical(grid),1);
    sv.Map = map;
    show(map)
    sv.ValidationDistance = 1;
    ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
    planner = plannerRRT(ss,sv,MaxConnectionDistance=vel_range2(2)/10);

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
            zone = Zones1{n};
            goal1 = [randi([zone.x(1), zone.x(2)]), randi([zone.y(1), zone.y(2)]), randi([0,90])];
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
            strt, goal1
            [pthObj, solnInfo] = plan(planner, strt, goal1);
            path = [path; pthObj.States(:,1:2)];
            strt = goal1;
        end

        %path = flip(path);
        alltrajectories{end+1} = path;
        traj_lens = [traj_lens, length(path)];
        initial_vels{end+1} = [vinit*cosd(ttx), vinit*sind(ttx)];
        if mode==1
            vel_changepoints{end+1} = goals{2}; 
        else 
            vel_changepoints{end+1} = goals{1};
        end
        changed_vels{end+1} = [vnew*cosd(goals{2}(3)), vnew*sind(goals{2}(3))];
    end

    for y = 1:gran_init:9*GRIDSIZE/10
        goals = {};
        path = []; %{};
        x = 1;
        for n = 1:Nint %Nint:-1:1
            zone = Zones2{n};
            goal1 = [randi([zone.x(1), zone.x(2)]), randi([zone.y(1), zone.y(2)]), randi([0,90])];
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
            strt, goal1
            [pthObj, solnInfo] = plan(planner, strt, goal1);
            path = [path; pthObj.States(:,1:2)];
            strt = goal1;
        end

        %path = flip(path);
        alltrajectories{end+1} = path;
        traj_lens = [traj_lens, length(path)];
        initial_vels{end+1} = [vinit*cosd(tty), vinit*sind(tty)];
        if mode==1
            vel_changepoints{end+1} = goals{2}; 
            changed_vels{end+1} = [vnew*cosd(goals{2}(3)), vnew*sind(goals{2}(3))];

        else 
            vel_changepoints{end+1} = goals{1};
            changed_vels{end+1} = [vnew*cosd(goals{1}(3)), vnew*sind(goals{1}(3))];
        end

    end
end