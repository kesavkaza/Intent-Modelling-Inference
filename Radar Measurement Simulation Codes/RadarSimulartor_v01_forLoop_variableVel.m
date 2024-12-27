%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code written in Matlab 2021b
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc,clear all
close all
warning off
%% Use fixed random seed for simulation repeatablity.
rng(0)
w = cd;
%datafolder = [w,'\Intent Inference 2D - 1000m-20241212\'];

datafolder = [w,'/Intent Inference 2D - 1000m/'];
%% Extracting the XYZ coordinates of the drones of the AirSim simulation
load([datafolder,'DirectAttackTrajectory_2D_1000m_varspeed_direct.mat'])

% load([datafolder,'HarmlessTrajectory_2D_1000m_varspeed.mat'])
% load([datafolder,'SurveillanceTrajectory_2D_1000m_varspeed.mat'])

ntraj = length(Trajectories);
for pathIndex = 1:ntraj
    tic
    constant_velocity = sqrt(sum(abs(init_vels_DA{pathIndex}).^2));% m/s
    %     constant_velocity = sqrt(sum(abs(init_vels_HL{pathIndex}).^2));% m/s
    %     constant_velocity = sqrt(sum(abs(init_vels_SL{pathIndex}).^2));% m/s
    traj = Trajectories{pathIndex};
    x_target = traj(:, 1);
    y_target = traj(:, 2);
    z_target = 10 * ones(size(x_target));
    deltaR = sqrt(diff(x_target).^2+diff(y_target).^2+diff(z_target).^2);
    deltaR(deltaR==0) = mean(deltaR(deltaR~=0));

    %% variable speed
    new_vel_point = vel_changepoints{pathIndex};
    %     new_vel_point = vel_changepoints_SL{pathIndex};
    changed_index = find(x_target == new_vel_point(1) & y_target == new_vel_point(2));
    changed_index = changed_index(1);
    new_vel_value = sqrt(sum(abs(changed_vels{pathIndex}).^2));
    %     new_vel_value = sqrt(sum(abs(changed_vels_SL{pathIndex}).^2));

    estimatedTs_part1 = deltaR(1:changed_index)./constant_velocity;
    estimatedTs_part2 = deltaR(changed_index+1:end)./new_vel_value;
    estimatedTimeVec = cumsum([estimatedTs_part1;estimatedTs_part2]).';
    estimatedTimeVec = [estimatedTimeVec estimatedTimeVec(end) + estimatedTs_part2(end)];
    x_radar = 1000 * ones(size(x_target));
    y_radar = 1000 * ones(size(x_target));
    z_radar = 10 * ones(size(x_target));

    fpstmp = 2*10;
    startindex = 0;
    endindex = round((estimatedTimeVec(end) + estimatedTs_part2(end))*fpstmp)-1;10*size(traj,1);
    simTimetmp = (endindex-startindex)/fpstmp;

    % Create a scenario
    s = uavScenario("StopTime",simTimetmp,"UpdateRate",fpstmp);

    % Create a quadrotor target
    target = uavPlatform("Target",s,"Trajectory",waypointTrajectory([x_target y_target z_target],"TimeOfArrival",estimatedTimeVec));
    updateMesh(target,"quadrotor", {15}, [1 0 0], eul2tform([0 0 pi]));

    % Create a fixed (hovering) quadrotor that carries the radar
    radarHolder = uavPlatform("RadarHolder",s,"Trajectory",waypointTrajectory([x_radar y_radar z_radar],"TimeOfArrival",estimatedTimeVec));
    updateMesh(radarHolder,"quadrotor",{15},[0 0 0],eul2tform([0 0 pi]));


    % Mount a radar on the "radarHolder" quadrotor.
    radarSensor = radarDataGenerator("no scanning","SensorIndex",1,"UpdateRate",fpstmp,...
        "FieldOfView",[90 90],...
        "HasElevation", true,...
        "ElevationResolution", 1,...
        "AzimuthResolution", 1, ...
        "RangeResolution", 1, ... meters
        "RangeRateResolution",0.91,...
        "TargetReportFormat","Tracks",...
        "TrackCoordinates","Scenario",...
        "HasINS", true,...
        "HasFalseAlarms",false,...
        "FalseAlarmRate",1e-6,...
        "HasRangeRate",true,...
        "ReferenceRCS",0,...
        "ReferenceRange",2500,...
        "RangeLimits",[0 2500],...
        "CenterFrequency",24.55e9,...
        "Bandwidth",45e6,...
        "HasNoise",1);

    % Create the sensor. ExampleHelperUAVRadar inherits from the uav.SensorAdaptor class.
    radar = uavSensor("Radar",radarHolder,ExampleHelperUAVRadar(radarSensor),"MountingAngles", [225 0 0]);
    % % %     [ax,plotFrames] = show3D(s);
    % % %     % ylim([min(min([x_target;x_radar]))-50,max(max([x_target;x_radar]))+50]);
    % % %     % xlim([min(min([y_target;y_radar]))-50,max(max([y_target;y_radar]))+50]);
    % % %     % zlim([min(min(-[z_target;z_radar]))-50,max(max(-[z_target;z_radar]))+50]);
    % % %     hold on;grid on

    % % %     % Add detection and sensor field of view to the plot.
    % % %     trackSquare = plot3(plotFrames.NED,nan,nan,nan,"-k");
    % % %
    % % %     radarDirection = hgtransform("Parent",plotFrames.RadarHolder.Radar,"Matrix",eye(4));
    % % %     coverageAngles = linspace(-radarSensor.FieldOfView(1)/360*pi, radarSensor.FieldOfView(1)/360*pi,128);
    % % %     coveragePatch = patch([0 radarSensor.RangeLimits(2)*cos(coverageAngles) 0], ...
    % % %         [0 radarSensor.RangeLimits(2)*sin(coverageAngles) 0],...
    % % %         "blue","FaceAlpha",0.3,...
    % % %         "Parent",radarDirection);
    % % %     hold(ax,"off");

    % Start simulation.
    setup(s);
    cntr = 0;
    det_id = 0;
    col_num_blocks = [];col_sys_day=[];col_sys_msec=[];
    col_last_upd_days = []; col_last_upd_msec = [];
    col_lifetime = [];
    col_pos_x = []; col_pos_y = []; col_pos_z = [];
    col_vel_x = [];col_vel_y = [];col_vel_z = [];
    col_prob_uav = [];
    col_track_id = []; col_state = [];
    trackSquare.XData = [];
    trackSquare.YData = [];
    trackSquare.ZData = [];
    while advance(s)
        cntr = cntr + 1;
        disp(['Path index: ',num2str(pathIndex),', cntr: ',num2str(cntr)])
        % Update sensor readings and read data.
        updateSensors(s);

        % Plot updated radar FOV.
        egoPose = read(radarHolder);
        radarFOV = coverageConfig(radarSensor, egoPose(1:3),quaternion(egoPose(10:13)));
        radarDirection.Matrix = eul2tform([radarFOV.LookAngle(1)/180*pi 0 0]);

        % Obtain detections from the radar and visualize them.
        [isUpdated,time,confTracks,numTracks,config] = read(radar);
        %     [simTime,~,detections,~,~,~,~,~] = read(radar);
        if numTracks > 0
            trackSquare.XData = [trackSquare.XData,confTracks(1).State(1)];
            trackSquare.YData = [trackSquare.YData,confTracks(1).State(3)];
            trackSquare.ZData = [trackSquare.ZData,confTracks(1).State(5)];

            for ii = 1:numTracks
                col_pos_x = [col_pos_x;confTracks(ii, 1).State(1)];
                col_pos_y = [col_pos_y;confTracks(ii, 1).State(3)];
                col_pos_z = [col_pos_z;confTracks(ii, 1).State(5)];
                col_vel_x = [col_vel_x;confTracks(ii, 1).State(2)];
                col_vel_y = [col_vel_y;confTracks(ii, 1).State(4)];
                col_vel_z = [col_vel_z;confTracks(ii, 1).State(6)];
                det_id = det_id + 1;
            end
            % % %             drawnow limitrate
        end
        % % %         title(['Radar output (number of detections: ',num2str(numTracks),')'])
        % % %         show3D(s,"FastUpdate", true,"Parent",ax);
        pause(0.1);
    end
    toc
    T = table(col_pos_x, col_pos_y, col_pos_z, col_vel_x, col_vel_y, col_vel_z);
    T.Properties.VariableNames ={'pos_x','pos_y','pos_z','vel_x','vel_y','vel_z'};
    writetable(T,[datafolder,'VarSpeed_radaroutput_Tracks_DA_',num2str(pathIndex),'.csv'])
    %     writetable(T,[datafolder,'VarSpeed_radaroutput_Tracks_HA_',num2str(pathIndex),'.csv'])
    %     writetable(T,[datafolder,'VarSpeed_radaroutput_Tracks_SU_',num2str(pathIndex),'.csv'])
end
