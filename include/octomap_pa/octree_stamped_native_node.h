/******************************************************************************
*                                                                             *
* octree_stamped_native_node.h                                                *
* ============================                                                *
*                                                                             *
*******************************************************************************
*                                                                             *
* github repository                                                           *
*   https://github.com/TUC-ProAut/ros_octomap                                 *
*                                                                             *
* Chair of Automation Technology, Technische Universität Chemnitz             *
*   https://www.tu-chemnitz.de/etit/proaut                                    *
*                                                                             *
*******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2015-2018, Peter Weissig, Technische Universität Chemnitz     *
* All rights reserved.                                                        *
*                                                                             *
* Redistribution and use in source and binary forms, with or without          *
* modification, are permitted provided that the following conditions are met: *
*     * Redistributions of source code must retain the above copyright        *
*       notice, this list of conditions and the following disclaimer.         *
*     * Redistributions in binary form must reproduce the above copyright     *
*       notice, this list of conditions and the following disclaimer in the   *
*       documentation and/or other materials provided with the distribution.  *
*     * Neither the name of the Technische Universität Chemnitz nor the       *
*       names of its contributors may be used to endorse or promote products  *
*       derived from this software without specific prior written permission. *
*                                                                             *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" *
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   *
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
* ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY      *
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR          *
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER  *
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT          *
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY   *
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH *
* DAMAGE.                                                                     *
*                                                                             *
******************************************************************************/

#ifndef __OCTREE_STAMPED_NATIVE_NODE_H
#define __OCTREE_STAMPED_NATIVE_NODE_H

// local headers
#include "octomap_pa/octree_stamped_native_ros.h"
#include "octomap_pa/octree_base_pa_node_parameter.h"
#include "octomap_pa/addcloud_parameter.h"

#include "octomap_pa/OctomapPaFileName.h"
#include "octomap_pa/OctomapPaGetSize.h"

// ros headers
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>

#include <octomap_msgs/Octomap.h>

#include <parameter_pa/parameter_pa_ros.h>

// additional libraries
#include <pcl/point_types.h>

#include <octomap/octomap.h>

// standard headers
#include <string>
#include <vector>

//**************************[cOctreeStampedNativeNode]*************************
class cOctreeStampedNativeNode : public cOctreeStampedNativeRos {
  public:
    //! default constructor
    cOctreeStampedNativeNode();

    //! default destructor
    ~cOctreeStampedNativeNode();

    //! function for publishing the octomap
    void publishOctomap(void);

  protected:

    //! parameters
    cOctreeBasePaNodeParameter nodeparams_;
    cAddCloudParameter         addparams_ ;

    //! number of inserted pointclouds
    int count_cloud_;
    //! number of inserted pointclouds (old format)
    int count_cloud_old_;
    //! number of inserted laser scans
    int count_laser_;

    //! node handler for topic subscription and advertising
    ros::NodeHandle nh_;

    //! transformation of different frames
    tf::TransformListener tf_listener_;


    //! subscriber for a pointcloud
    ros::Subscriber sub_cloud_;
    //! callback function for receiving a pointcloud
    void addPointcloudCallbackSub(
        const sensor_msgs::PointCloud2ConstPtr &msg);

    //! subscriber for a pointcloud (old format)
    ros::Subscriber sub_cloud_old_;
    //! callback function for receiving a pointcloud (old format)
    void addPointcloudOldCallbackSub(
        const sensor_msgs::PointCloudConstPtr &msg);

    //! subscriber for a laserscan
    ros::Subscriber sub_laser_;
    //! callback function for receiving a laserscan
    void addLaserCallbackSub(const sensor_msgs::LaserScanConstPtr &msg);

    //! puplisher for the octomap (binary data)
    ros::Publisher pub_octomap_;
    //! puplisher for the octomap (full data)
    ros::Publisher pub_octomap_full_;
    //! puplisher for free voxels as pointcloud
    ros::Publisher pub_cloud_free_;
    //! puplisher for occupied voxels as pointcloud
    ros::Publisher pub_cloud_occupied_;


    //! service for clearing the octomap
    ros::ServiceServer srv_clear_;
    bool clearCallbackSrv(std_srvs::Empty::Request  &req,
      std_srvs::Empty::Response &res);

    //! service for receiving the size of the octomap
    ros::ServiceServer srv_getsize_;
    bool getSizeCallbackSrv(
      octomap_pa::OctomapPaGetSize::Request  &req,
      octomap_pa::OctomapPaGetSize::Response &res);
    //! service for saving the octomap
    ros::ServiceServer srv_save_;
    bool saveCallbackSrv(
      octomap_pa::OctomapPaFileName::Request  &req,
      octomap_pa::OctomapPaFileName::Response &res);
    //! service for loading a octomap
    ros::ServiceServer srv_load_;
    bool loadCallbackSrv(
      octomap_pa::OctomapPaFileName::Request  &req,
      octomap_pa::OctomapPaFileName::Response &res);
};

#endif // __OCTREE_STAMPED_NATIVE_NODE_H
