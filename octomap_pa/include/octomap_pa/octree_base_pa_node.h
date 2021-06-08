/******************************************************************************
*                                                                             *
* octree_base_pa_node.h                                                       *
* =====================                                                       *
*                                                                             *
*******************************************************************************
*                                                                             *
* Repository:                                                                 *
*   https://github.com/TUC-ProAut/ros_octomap                                 *
*                                                                             *
* Chair of Automation Technology, Technische Universität Chemnitz             *
*   https://www.tu-chemnitz.de/etit/proaut                                    *
*                                                                             *
* Author:                                                                     *
*   Peter Weissig                                                             *
*                                                                             *
*******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2015-2021 TU Chemnitz                                         *
* All rights reserved.                                                        *
*                                                                             *
* Redistribution and use in source and binary forms, with or without          *
* modification, are permitted provided that the following conditions are met: *
*    * Redistributions of source code must retain the above copyright notice, *
*      this list of conditions and the following disclaimer.                  *
*    * Redistributions in binary form must reproduce the above copyright      *
*      notice, this list of conditions and the following disclaimer in the    *
*      documentation and/or other materials provided with the distribution.   *
*    * Neither the name of the copyright holder nor the names of its          *
*      contributors may be used to endorse or promote products derived from   *
*      this software without specific prior written permission.               *
*                                                                             *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS         *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR  *
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR           *
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       *
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         *
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    *
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     *
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF      *
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                  *
*                                                                             *
******************************************************************************/

#ifndef __BASECLASS_BASE_PA_NODE_H
#define __BASECLASS_BASE_PA_NODE_H

// local headers
#include "octomap_pa/addcloud_parameter.h"
#include "octomap_pa/octree_base_pa_node_parameter.h"

#include "octomap_pa_msgs/Reset.h"
#include "octomap_pa_msgs/SetConfigInsertion.h"
#include "octomap_pa_msgs/SetConfigDegrading.h"
#include "octomap_pa_msgs/GetConfig.h"

#include "octomap_pa_msgs/AddCloud.h"
#include "octomap_pa_msgs/AddCloudTf.h"
#include "octomap_pa_msgs/GetCloud.h"
#include "octomap_pa_msgs/GetSize.h"

#include "octomap_pa_msgs/FileName.h"

// ros headers
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>

#include <octomap_msgs/Octomap.h>

#include <tf/transform_datatypes.h>

// additional libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>

//**************************[cOctreeBasePaNode]********************************
template <typename BASECLASS>
class cOctreeBasePaNode : public BASECLASS {
  public:

    typedef BASECLASS                    TypeBase;
    typedef cOctreeBasePaNode<BASECLASS> TypeFull;

    //! default constructor
    cOctreeBasePaNode(const std::string nodename,
      const double resolution = 0.1,
      const ros::Duration tf_listener_buffersize = ros::Duration(20));

    //! default destructor
    virtual ~cOctreeBasePaNode();

    //! function for publishing the octomap
    void publishOctomap(void);

    //! function for retrieving all current configs
    virtual octomap_pa_msgs::Config getConfig(void);
    //! function for setting config for adding pointclouds
    bool setConfigInsertion(const octomap_pa_msgs::ConfigInsertion &config);
    //! function for setting degrading configs (does nothing by default
    virtual bool setConfigDegrading(
      const octomap_pa_msgs::ConfigDegrading &config);

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
    //! service for resetting the octomap
    //! (clearing & changing basic properties)
    ros::ServiceServer srv_reset_;
    bool resetCallbackSrv(
      octomap_pa_msgs::Reset::Request  &req,
      octomap_pa_msgs::Reset::Response &res);

    //! service for setting the degrading config of the octomap
    ros::ServiceServer srv_setconfig_degrading_;
    bool setConfigDegradingCallbackSrv(
      octomap_pa_msgs::SetConfigDegrading::Request  &req,
      octomap_pa_msgs::SetConfigDegrading::Response &res);
    //! service for setting the insertion config of the octomap
    ros::ServiceServer srv_setconfig_insertion_;
    bool setConfigInsertionCallbackSrv(
      octomap_pa_msgs::SetConfigInsertion::Request  &req,
      octomap_pa_msgs::SetConfigInsertion::Response &res);
    //! service for receiving all config of the octomap
    ros::ServiceServer srv_getconfig_;
    bool getConfigCallbackSrv(
      octomap_pa_msgs::GetConfig::Request  &req,
      octomap_pa_msgs::GetConfig::Response &res);

    //! service for adding a new pointcloud to the octomap
    ros::ServiceServer srv_addcloud_;
    bool addCloudCallbackSrv(
      octomap_pa_msgs::AddCloud::Request  &req,
      octomap_pa_msgs::AddCloud::Response &res);
    //! service for adding a new pointcloud to the octomap (by passing a tf)
    ros::ServiceServer srv_addcloudtf_;
    bool addCloudTfCallbackSrv(
      octomap_pa_msgs::AddCloudTf::Request  &req,
      octomap_pa_msgs::AddCloudTf::Response &res);
    //! service for receiving the current octomap as pointcloud
    ros::ServiceServer srv_getcloud_;
    bool getCloudCallbackSrv(
      octomap_pa_msgs::GetCloud::Request  &req,
      octomap_pa_msgs::GetCloud::Response &res);
    //! service for receiving the current size of the octomap
    ros::ServiceServer srv_getsize_;
    bool getSizeCallbackSrv(
      octomap_pa_msgs::GetSize::Request  &req,
      octomap_pa_msgs::GetSize::Response &res);

    //! service for saving the octomap
    ros::ServiceServer srv_save_;
    bool saveCallbackSrv(
      octomap_pa_msgs::FileName::Request  &req,
      octomap_pa_msgs::FileName::Response &res);
    //! service for loading a octomap
    ros::ServiceServer srv_load_;
    bool loadCallbackSrv(
      octomap_pa_msgs::FileName::Request  &req,
      octomap_pa_msgs::FileName::Response &res);

    //! helper function to update/check timestamps & check/retrieve TF
    bool updateTimeAndGetTF(const std_msgs::Header header,
      tf::Transform &transform);

    //! virtual overload for additional updates
    virtual void internal_node_update(void) = 0;


  private:
    //! official node name used for ros info messages
    std::string nodename_;
};

#include "octomap_pa/octree_base_pa_node.hxx"

#endif // __BASECLASS_BASE_PA_NODE_H
