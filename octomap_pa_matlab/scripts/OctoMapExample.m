%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% OctoMapExample.m                                                            %
% ================                                                            %
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
% Copyright (c) 2019-2021 TU Chemnitz                                         %
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

% Before calling this script make sure that the octomap node is running
% If there are still problems with the ros interface
%    (e.g. may happen if the ros node was restarted)
%    call octo.resetRosInterfaces

% This is a simple 2d example - the third dimension (z-axis) is neglected.

%% instantiate object of OctoMap Class
octo = OctoMap;

%% change some settings
octo.map_prob_hit  = 0.75;
octo.map_prob_miss = 0.35;

octo.setConfigInsertion();

%% reset octomap
% only through this function it is allowed to change the resolution
% or the output frame
octo.reset(0.05, 'octomap');


%% iterate to show effect of propabilistic updates
for i = 1:100

    %% add some measured points (distributed along a circle)
    POINTCOUNT = 50;
    radius = randn(POINTCOUNT, 1) * 0.10 + 1;
    angle  = rand (POINTCOUNT, 1) * 2*pi;
    points = radius .* [cos(angle), sin(angle)];
    points(:,3) = 0;

    octo.addCloud(points);


    %% read current octomap
    occupied_voxel = octo.getCloudAsMatrix();
    free_voxel     = octo.getCloudAsMatrix(false);

    %% plot everything
    hold off

    % plot origin
    plot(0, 0, 'k+', 'MarkerSize', 15);
    hold on;

    % added pointcloud (circle)
    plot(points(:,1), points(:,2), 'rx', 'MarkerSize', 8);

    % occupied (red) and free voxel (black dots)
    plot(occupied_voxel(:,1), occupied_voxel(:,2), 'b.', 'MarkerSize', 6);
    plot(free_voxel    (:,1),     free_voxel(:,2), 'g.', 'MarkerSize', 6);

    % setup
    xlim([-1.5,1.5]);
    ylim([-1.5,1.5]);
    axis equal
    title('octomap example - inserting circular shaped pointclouds');
    legend('origin', 'current point cloud', 'occupied voxel', 'free voxels');

    % wait and display
    drawnow;
    pause(1);
end
