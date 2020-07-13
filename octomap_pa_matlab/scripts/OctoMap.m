%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% OctoMap.m                                                                   %
% =========                                                                   %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% Repository:                                                                 %
%   https://github.com/TUC-ProAut/ros_octomap                                 %
%                                                                             %
% Chair of Automation Technology, Technische Universität Chemnitz             %
%   https://www.tu-chemnitz.de/etit/proaut                                    %
%                                                                             %
% Author:                                                                     %
%   Peter Weissig                                                             %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% New BSD License                                                             %
%                                                                             %
% Copyright (c) 2020 TU Chemnitz                                              %
% All rights reserved.                                                        %
%                                                                             %
% Redistribution and use in source and binary forms, with or without          %
% modification, are permitted provided that the following conditions are met: %
%    * Redistributions of source code must retain the above copyright notice, %
%      this list of conditions and the following disclaimer.                  %
%    * Redistributions in binary form must reproduce the above copyright      %
%      notice, this list of conditions and the following disclaimer in the    %
%      documentation and/or other materials provided with the distribution.   %
%    * Neither the name of the copyright holder nor the names of its          %
%      contributors may be used to endorse or promote products derived from   %
%      this software without specific prior written permission.               %
%                                                                             %
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS         %
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   %
% TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR  %
% PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR           %
% CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       %
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         %
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; %
% OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    %
% WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     %
% OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF      %
% ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                  %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Interface-class to access ROS OctoMap (original and ProAut derived)
classdef OctoMap < handle

    properties
        %% degrading of voxels
        % duration how long the outdated voxels will be kept (60s)
        degrading_time          (1,1) double  = 60;

        % turns on automatic degrading
        auto_degrading          (1,1) logical = true;

        % intervall for automatic degrading
        % (only based on header timestamps)
        auto_degrading_intervall(1,1) double  = 2;


        %% pointcloud insertion
        % use pcl-transform instead of octomap-transform (speeds up)
        pcd_explicit_transform(1,1) logical =  true;

        % use voxel-filter (speeds up)
        pcd_voxel_active      (1,1) logical =  true;

        % use pcl-filter instead of octomap-filter (slows down)
        pcd_voxel_explicit    (1,1) logical =  false;
        % relative resolution of pcl-filter
        pcd_voxel_explicit_relative_resolution(1,1) double =  0.5;

        % probalitity that a positive measurement relates to a occupied voxel
        map_prob_hit (1,1) double ...
          {mustBeNonnegative, mustBeLessThan(map_prob_hit ,1)} = 0.7;

        % probalitity that a negative measurement relates to a occupied voxel
        map_prob_miss(1,1) double ...
          {mustBeNonnegative, mustBeLessThan(map_prob_miss,1)} = 0.4;


        %% general octomap settings
        % threshold for binary evaluation of single voxels (occupied or free)
        map_prob_threshold(1,1) double ...
          {mustBeNonnegative, mustBeLessThan(map_prob_threshold,1)} = 0.5;

        % clamping values (to promote prunning)
        map_clamp_min(1,1) double ...
          {mustBeNonnegative, mustBeLessThan(map_clamp_min,1)} = 0.12;
        map_clamp_max(1,1) double ...
          {mustBeNonnegative, mustBeLessThan(map_clamp_max,1)} = 0.97;

    end

    properties (SetAccess = private)
        %% general octomap settings (only changed on reset)
        % frame of the octomap (coordinate system for insertion and output)
        output_frame  (1,:) char   = 'map';

        % resolution of octomap (side length of one voxel)
        map_resolution(1,1) double {mustBeNonnegative} = 0.1;

        %% ros communication
        maxRosServiceTimeOut(1,1) double {mustBeNonnegative} = 10;
    end

    properties (Access = private)
        %% ros interfaces
        rosservice_getconfig           = [];
        rosservice_setconfig_degrading = [];
        rosservice_setconfig_insertion = [];

        rosservice_clear = [];
        rosservice_reset = [];

        rosservice_addcloud = [];
        rosservice_getcloud = [];
    end


    % constructor & destructor
    methods
        function obj = OctoMap()
            %Construct an instance of this class

            % ... nothing todo ^^
        end
    end


    % config
    methods
        % get config
        function result = getConfig(obj)

            % init result and internal variables
            result = false;

            % get client and call service
            serviceclient = obj.getServiceGetConfig();
            if (isempty(serviceclient)); return; end
            request = rosmessage(serviceclient);
            response = call(serviceclient, request);

            % check result
            if (isempty(response)); return; end

            % store response
            obj.auto_degrading = response.Config.Degrading.AutoDegrading;
            obj.auto_degrading_intervall = ...
              response.Config.Degrading.AutoDegradingIntervall;

            obj.map_clamp_max  = response.Config.Insertion.MapClampMax;
            obj.map_clamp_min  = response.Config.Insertion.MapClampMin;
            obj.map_prob_hit   = response.Config.Insertion.MapProbHit;
            obj.map_prob_miss  = response.Config.Insertion.MapProbMiss;
            obj.map_prob_threshold = ...
              response.Config.Insertion.MapProbThreshold;

            obj.map_resolution = response.Config.Base.MapResolution;
            obj.output_frame   = response.Config.Base.OutputFrame;

            % return ok
            result = true;
        end

        % set config
        function result = setConfigDegrading(obj)

            % init result and internal variables
            result = false;

            % get client
            serviceclient = obj.getServiceSetConfigDegrading();
            if (isempty(serviceclient)); return; end

            % create request message
            request = rosmessage(serviceclient);
            request.Config.AutoDegrading = obj.auto_degrading;
            request.Config.AutoDegradingIntervall = ...
              obj.auto_degrading_intervall;

            % call service
            response = call(serviceclient, request);

            % check result
            if (isempty(response)); return; end

            % return ok
            result = response.Ok;
        end

        % set config
        function result = setConfigInsertion(obj)

            % init result and internal variables
            result = false;

            % get client
            serviceclient = obj.getServiceSetConfigInsertion();
            if (isempty(serviceclient)); return; end

            % create request message
            request = rosmessage(serviceclient);
            request.Config.MapClampMax = obj.map_clamp_max;
            request.Config.MapClampMin = obj.map_clamp_min;
            request.Config.MapProbHit  = obj.map_prob_hit;
            request.Config.MapProbMiss = obj.map_prob_miss;
            request.Config.MapProbThreshold = obj.map_prob_threshold;

            % call service
            response = call(serviceclient, request);

            % check result
            if (isempty(response)); return; end

            % return ok
            result = response.Ok;
        end

        % clear & reset
        function result = clear(obj)

            % init result and internal variables
            result = false;

            % get client and call service
            serviceclient = obj.getServiceClear();
            if (isempty(serviceclient)); return; end
            request = rosmessage(serviceclient);
            response = call(serviceclient, request);

            % check result
            if (isempty(response)); return; end

            % return ok
            result = true;
        end
        function result = reset(obj, resolution, output_frame)

            % init result and internal variables
            result = false;
            if (nargin < 2)
                resolution = obj.map_resolution;
            end
            if (nargin < 3)
                output_frame = obj.output_frame;
            end

            % get client
            serviceclient = obj.getServiceReset();
            if (isempty(serviceclient)); return; end

            % create request message
            request = rosmessage(serviceclient);
            request.Config.MapResolution = resolution;
            request.Config.OutputFrame   = output_frame;

            % call service
            response = call(serviceclient, request);

            % check result
            if (isempty(response)); return; end

            % return ok
            result = response.Ok;
        end
    end

    % xyz = readXYZ(pcloud)
    % add pointcloud & get octomap
    methods
        function result = addCloud(obj, data, transform)

            % init result and internal variables
            result = false;
            if (nargin < 3)
                transform = eye(4);
            end

            % convert data if necessary
            if (isnumeric(data))
                data = OctoMap.mat2PointCloud2(data);
                data.Header.FrameId = obj.output_frame;
            end
            if (isnumeric(transform) && (size(transform, 1) == 4) && ...
              (size(transform, 2) == 4))
                transform = obj.tform2rostf(transform);
            end

            % get client
            serviceclient = obj.getServiceAddCloud();
            if (isempty(serviceclient)); return; end

            % create request message
            request = rosmessage(serviceclient);
            request.Cloud     = data;
            request.Transform = transform;

            % call service
            response = call(serviceclient, request);

            % check result
            if (isempty(response)); return; end

            % return ok
            result = response.Ok;
        end

        function result = getCloud(obj, occupied)

            % init result and internal variables
            result = [];
            if (nargin < 2)
                occupied = true;
            end

            % get client and call service
            serviceclient = obj.getServiceGetCloud();
            if (isempty(serviceclient)); return; end
            request = rosmessage(serviceclient);
            request.Occupied = occupied;
            response = call(serviceclient, request);

            % check result
            if (isempty(response)); return; end

            % return result
            result = response.Cloud;
        end

        function result = getCloudAsMatrix(obj, occupied)

            if (nargin < 2)
                result = readXYZ(obj.getCloud());
            else
                result = readXYZ(obj.getCloud(occupied));
            end
        end

    end


    %% ros-interface
    % internal (creating static variable
    methods (Static, Access=private)
        function out = setRosNode(data)

            persistent rosnode;
            if (nargin > 0)
                rosnode = data;
            end
            out = rosnode;
        end

    end

    % half internal (hidden)
    methods (Static, Hidden)
        function out = getRosNode()

            out = OctoMap.setRosNode();
            if (isempty(out))
                node_name = 'octomap_for_matlab';
                fprintf(['Creating ROS-node "', node_name, '"\n']);
                out = OctoMap.setRosNode(robotics.ros.Node(node_name));
            end
        end

        function resetRosNode()

            OctoMap.setRosNode([]);
        end

        function startGlobalRosNodeIfNecessary()

            if (~robotics.ros.internal.Global.isNodeActive)
                rosinit();
            end
        end

        function nodename = getOctomapNodeName(selectFirst)

            % init result
            nodename = '';

            % get list of all rosnodes
            OctoMap.startGlobalRosNodeIfNecessary();
            list = rosnode('list');

            % check for octomap node
            mask = ~contains(list, 'matlab') & ...
                contains(list, 'octomap') & contains(list, 'pa');
            list = list(mask);

            % check if list is empty
            if (isempty(list)); return; end

            % check for multiple results
            if (~isscalar(list))
                if ((nargin < 1) || ~selectFirst)
                    return;
                end
            end

            % return first entry
            nodename = char(list(1));
        end

        function client = createRosServiceClient(service_name, timeout)

            % init result
            client = '';

            % check for octomap node
            nodename_octomap = OctoMap.getOctomapNodeName();
            if (isempty(nodename_octomap))
                warning('Can''t locate octomap node. Is it launch ?');
                return
            end

            % get own node name
            nodename_self = OctoMap.getRosNode();

            % get full service name
            service_fullname=[nodename_octomap, '/', service_name];

            % print what this function does
            fprintf(['Creating ROS-service client for octomap ', ...
              'service "', service_name, '"\n']);

            if (nargin < 2)
              client = robotics.ros.ServiceClient( ...
                nodename_self, ...
                service_fullname);
            else
              client = robotics.ros.ServiceClient( ...
                nodename_self, ...
                service_fullname, 'Timeout', timeout);
            end
        end

        function pointcloud2 = mat2PointCloud2(mat)
            % function based on matlab answer
            %    https://de.mathworks.com/matlabcentral/answers/395620
            %    "ROS create PointCloud2 from MATLAB pointCloud"
            %    Author: Sebastian Castro

            pointcloud2 = rosmessage('sensor_msgs/PointCloud2');

            % Calculate number of points
            numPts = size(mat,1);

            % Assign metadata
            pointcloud2.Height    = uint32(1);
            pointcloud2.Width     = uint32(numPts);
            pointcloud2.PointStep = uint32(12);
            pointcloud2.RowStep   = uint32(12);

            % Assign point field data
            fieldNames = {'x','y','z'};
            pointcloud2.Data = zeros( ...
              numPts * pointcloud2.RowStep,1,'uint8');

            for idx = 1:3
                pointcloud2.Fields(idx) = ...
                  rosmessage('sensor_msgs/PointField');
                fName = fieldNames{idx};
                pointcloud2.Fields(idx).Name(1:numel(fName)) = ...
                  uint8(fName);
                pointcloud2.Fields(idx).Offset   = uint32((idx-1)*4);
                pointcloud2.Fields(idx).Datatype = uint8(7);
                pointcloud2.Fields(idx).Count    = uint32(1);
            end

            % convert data to float
            mat = single(mat);

            % Assign raw point cloud data in uint8 format
            for idx = 1:numPts
               startIdx = (idx-1) * pointcloud2.RowStep + 1;
               pointcloud2.Data(startIdx:startIdx+11) = ...
                   typecast(mat(idx,:),'uint8');
            end
        end

        function tf = tform2rostf(mat)

            % init result
            tf = rosmessage('geometry_msgs/Transform');

            % set rotation
            quat = tform2quat(mat);
            tf.Rotation.W = quat(1);
            tf.Rotation.X = quat(2);
            tf.Rotation.Y = quat(3);
            tf.Rotation.Z = quat(4);

            % set translation
            vec = tform2trvec(mat);
            tf.Translation.X = vec(1);
            tf.Translation.X = vec(2);
            tf.Translation.X = vec(3);
        end

    end

    % internal
    methods (Access=private)
        function serviceclient = getServiceGetConfig(obj)

            % init result
            serviceclient = ''; %#ok<NASGU>

            if (isempty(obj.rosservice_getconfig))
                obj.rosservice_getconfig = ...
                  obj.createRosServiceClient('getconfig');
            end

            % return result
            serviceclient = obj.rosservice_getconfig;
        end

        function serviceclient = getServiceSetConfigDegrading(obj)

            % init result
            serviceclient = ''; %#ok<NASGU>

            if (isempty(obj.rosservice_setconfig_degrading))
                obj.rosservice_setconfig_degrading = ...
                  obj.createRosServiceClient('setconfig_degrading');
            end

            % return result
            serviceclient = obj.rosservice_setconfig_degrading;
        end

        function serviceclient = getServiceSetConfigInsertion(obj)

            % init result
            serviceclient = ''; %#ok<NASGU>

            if (isempty(obj.rosservice_setconfig_insertion))
                obj.rosservice_setconfig_insertion = ...
                  obj.createRosServiceClient('setconfig_insertion');
            end

            % return result
            serviceclient = obj.rosservice_setconfig_insertion;
        end

        function serviceclient = getServiceClear(obj)

            % init result
            serviceclient = ''; %#ok<NASGU>

            if (isempty(obj.rosservice_clear))
                obj.rosservice_clear = ...
                  obj.createRosServiceClient('clear');
            end

            % return result
            serviceclient = obj.rosservice_clear;
        end

        function serviceclient = getServiceReset(obj)

            % init result
            serviceclient = ''; %#ok<NASGU>

            if (isempty(obj.rosservice_reset))
                obj.rosservice_reset = ...
                  obj.createRosServiceClient('reset');
            end

            % return result
            serviceclient = obj.rosservice_reset;
        end

        function serviceclient = getServiceAddCloud(obj)

            % init result
            serviceclient = ''; %#ok<NASGU>

            if (isempty(obj.rosservice_addcloud))
                obj.rosservice_addcloud = ...
                  obj.createRosServiceClient('addcloudtf');
            end

            % return result
            serviceclient = obj.rosservice_addcloud;
        end

        function serviceclient = getServiceGetCloud(obj)

            % init result
            serviceclient = ''; %#ok<NASGU>

            if (isempty(obj.rosservice_getcloud))
                obj.rosservice_getcloud = ...
                  obj.createRosServiceClient('getcloud');
            end

            % return result
            serviceclient = obj.rosservice_getcloud;
        end
    end

    % external (user interface)
    methods
        function resetRosInterfaces(obj)

            % reset node
            obj.resetRosNode();

            % reset services
            obj.rosservice_getconfig           = [];
            obj.rosservice_setconfig_degrading = [];
            obj.rosservice_setconfig_insertion = [];

            obj.rosservice_clear = [];
            obj.rosservice_reset = [];

            obj.rosservice_addcloud = [];
            obj.rosservice_getcloud = [];
        end
    end
end
