classdef ExampleHelperUAVRadar < uav.SensorAdaptor
    %ExampleHelperUAVRadar Adatpts the radarDataGenerator to UAV Scenario usage
    
    %   Copyright 2020 The MathWorks, Inc.
    
    properties
        %UpdateRate Sensor Update Rate in Hz
        UpdateRate
    end
    
    methods
        function s = get.UpdateRate(obj)
            s = obj.SensorModel.UpdateRate;
        end
        
        function set.UpdateRate(obj, s)
            obj.SensorModel.UpdateRate = s;
        end
    end
    
    methods
        function obj = ExampleHelperUAVRadar(sensorModel)
            %uavRadar
            
            obj@uav.SensorAdaptor(sensorModel);
        end
        
        
        function setup(~, ~, ~)
            %setup Prepare sensor for simulation
            
            % For radar sensor, setup is no-op
        end
        
        
        function [dets,numDets,config] = read(obj, scene, egoPlatform, sensor, t)
            %read Generate sensor readings based on platforms in scene
            
            % Build inputs for the radarDataGenerator step methods, i.e
            % - targets : an array of struct with the following fields
            %             - PlatformID
            %             - ClassID
            %             - Position ( in scenario coordinates)
            %             - Velocity 
            %             - Acceleration
            %             - Orientation (rotation matrix or quaternion)
            %             - Angular Velocity
            %             - Dimensions
            %             - Signatures
            %
            % - egoPose : a struct containing the pose of the mounting uav
            % - t : simulation time
            
            targetTemplate =  struct( ...
                'PlatformID', 0, ...
                'ClassID', 0, ...
                'Position', zeros(1,3), ...
                'Velocity', zeros(1,3), ...
                'Acceleration', zeros(1,3), ...
                'Orientation', quaternion(1,0,0,0), ...
                'AngularVelocity', zeros(1,3),...
                'Dimensions', struct( ...
                             'Length', 0, ...
                             'Width', 0, ...
                             'Height', 0, ...
                             'OriginOffset', [0 0 0]), ...
                'Signatures', {{rcsSignature}});

            numTargets = numel(scene.Platforms) - 1;
            targetPoses = repmat(targetTemplate, 1, numTargets);
            
            tgtid = 0;
            for idx = 1:numel(scene.Platforms)
                id = scene.Platforms(idx).Name;
                % Skip ego
                if ~strcmp(id, egoPlatform.Name)
                    tgtid = tgtid+1;
                    tgtpose = scene.Platforms(idx).read();
                    targetPoses(tgtid).PlatformID = idx;
                    targetPoses(tgtid).Position = tgtpose(1:3);
                    targetPoses(tgtid).Velocity = tgtpose(4:6);
                    targetPoses(tgtid).Orientation = quaternion(tgtpose(10:13));
                end
            end
            
            % Define pose of the ego uav
            egoPoseVector = obj.getMotion(scene, egoPlatform, sensor, t);
            egoPose = struct;
            egoPose.Position = egoPoseVector(1:3);
            egoPose.Velocity = egoPoseVector(4:6);
            egoPose.Orientation = quaternion(egoPoseVector(10:13));
            egoPose.AngularVelocity = zeros(1,3);
            
            % Convert target poses from scenario frame to ego body frame
            Rego = rotmat(egoPose.Orientation, 'frame');
            targets = targetPoses;
            for i=1: numel(targetPoses)
                thisTgt = targetPoses(i);
                pos = Rego*(thisTgt.Position(:) - egoPose.Position(:));
                vel = Rego*(thisTgt.Velocity(:) - egoPose.Velocity(:)) - cross(egoPose.AngularVelocity(:),pos(:));
                angVel = thisTgt.AngularVelocity(:) - egoPose.AngularVelocity(:);
                orient = egoPose.Orientation' * thisTgt.Orientation;

                % store into target structure array
                targets(i).Position(:) = pos;
                targets(i).Velocity(:) = vel;
                targets(i).AngularVelocity(:) = angVel;
                targets(i).Orientation = orient;
            end
            
                       
            [dets,numDets,config] = obj.SensorModel(targets,egoPose,t);
        end
        
        
        function out = getEmptyOutputs(~)
            %getNumOutput Provide outputs when sensor is not updated
            out = {nan, nan, nan};
        end
        
        
        function reset(obj)
            %reset Reset sensor states
            obj.SensorModel.reset();
        end
    end
end

