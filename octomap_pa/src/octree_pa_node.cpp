/******************************************************************************
*                                                                             *
* octree_pa_node.cpp                                                          *
* ==================                                                          *
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
* Copyright (c) 2015-2020 TU Chemnitz                                         *
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

// local headers
#include "octomap_pa/octree_pa_node.h"

// ros headers
#include <parameter_pa/parameter_pa_ros.h>

// standard headers
#include <string>

//**************************[main]*********************************************
int main(int argc, char **argv) {

    ros::init(argc, argv, "octree_pa_node");
    cOctreePaNode octomap;

    ros::spin();

    return 0;
}

//**************************[cOctreePaNode]************************************
cOctreePaNode::cOctreePaNode() :
  cOctreePaRos(0.1), tf_listener_(ros::Duration(20), true) {

    cParameterPaRos paramloader;

    paramloader.load("~/output_frame", rosparams_base_.output_frame_);

    // octomap parameter
    double temp;
    temp = 0.1 ;
    paramloader.load("~/map_resolution"    , temp); setResolution      (temp);
    temp = 0.5 ;
    paramloader.load("~/map_prob_threshold", temp); setOccupancyThres  (temp);
    temp = 0.12;
    paramloader.load("~/map_clamp_min"     , temp); setClampingThresMin(temp);
    temp = 0.97;
    paramloader.load("~/map_clamp_max"     , temp); setClampingThresMax(temp);

    // pointcloud insertion parameter
    paramloader.load("~/map_prob_hit"      , addparams_.map_prob_hit_      );
    paramloader.load("~/map_prob_miss"     , addparams_.map_prob_miss_     );
    paramloader.load("~/pcd_voxel_active"  , addparams_.pcd_voxel_active_  );
    paramloader.load("~/pcd_voxel_explicit", addparams_.pcd_voxel_explicit_);
    paramloader.load("~/pcd_voxel_explicit_relative_resolution",
      addparams_.pcd_voxel_explicit_relative_resolution_);
    paramloader.load("~/pcd_explicit_transform",
      addparams_.pcd_explicit_transform_);

    // topics in
    paramloader.loadTopic("~/topic_in_cloud"    ,
      nodeparams_.topic_in_cloud_    );
    paramloader.loadTopic("~/topic_in_cloud_old",
      nodeparams_.topic_in_cloud_old_);
    paramloader.loadTopic("~/topic_in_laser"    ,
      nodeparams_.topic_in_laser_    );

    // topics out
    paramloader.loadTopic("~/topic_out_octomap"       ,
      nodeparams_.topic_out_octomap_);
    paramloader.loadTopic("~/topic_out_octomap_full"  ,
      nodeparams_.topic_out_octomap_full_);
    paramloader.loadTopic("~/topic_out_cloud_free"    ,
      nodeparams_.topic_out_cloud_free_);
    paramloader.loadTopic("~/topic_out_cloud_occupied",
      nodeparams_.topic_out_cloud_occupied_);


    // Subscriber for pointclouds
    if (nodeparams_.topic_in_cloud_ != "") {
        sub_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          nodeparams_.topic_in_cloud_, 10,
          &cOctreePaNode::addPointcloudCallbackSub, this);
    }

    // Subscriber for pointclouds (old format)
    if (nodeparams_.topic_in_cloud_old_ != "") {
        sub_cloud_old_ = nh_.subscribe<sensor_msgs::PointCloud>(
          nodeparams_.topic_in_cloud_old_, 10,
          &cOctreePaNode::addPointcloudOldCallbackSub, this);
    }
    // Subscriber for laserscans
    if (nodeparams_.topic_in_laser_ != "") {
        sub_laser_ = nh_.subscribe<sensor_msgs::LaserScan>(
          nodeparams_.topic_in_laser_, 10,
          &cOctreePaNode::addLaserCallbackSub, this);
    }


    // puplisher for the octomap (binary data)
    pub_octomap_        = nh_.advertise<octomap_msgs::Octomap>(
      nodeparams_.topic_out_octomap_, 10, true);
    // puplisher for the octomap (full data)
    pub_octomap_full_   = nh_.advertise<octomap_msgs::Octomap>(
      nodeparams_.topic_out_octomap_full_, 10, true);

    // puplisher for free voxels as pointcloud
    pub_cloud_free_     = nh_.advertise<sensor_msgs::PointCloud2>(
      nodeparams_.topic_out_cloud_free_, 10, true);
    // puplisher for occupied voxels as pointcloud
    pub_cloud_occupied_ = nh_.advertise<sensor_msgs::PointCloud2>(
      nodeparams_.topic_out_cloud_occupied_, 10, true);

    std::string str_service("~/");
    str_service = paramloader.resolveRessourcename(str_service);
    // service for clearing the octomap
    srv_clear_ = nh_.advertiseService(str_service + "clear",
      &cOctreePaNode::clearCallbackSrv, this);
    // service for receiving the size of the octomap
    srv_getsize_ = nh_.advertiseService(str_service + "getsize",
      &cOctreePaNode::getSizeCallbackSrv, this);
    // service for saving the octomap
    srv_save_ = nh_.advertiseService(str_service + "save",
      &cOctreePaNode::saveCallbackSrv, this);
    // service for loading a octomap
    srv_load_ = nh_.advertiseService(str_service + "load",
      &cOctreePaNode::loadCallbackSrv, this);

    // count number of inserted pointclouds
    count_cloud_     = 0;
    count_cloud_old_ = 0;
    count_laser_     = 0;
}

//**************************[~cOctreePaNode]***********************************
cOctreePaNode::~cOctreePaNode() {

}

//**************************[publishOctomap]***********************************
void cOctreePaNode::publishOctomap() {

    if (pub_octomap_.getNumSubscribers() > 0) {
        pub_octomap_.publish(getOctomap());
    }
    if (pub_octomap_full_.getNumSubscribers() > 0) {
        pub_octomap_.publish(getOctomapFull());
    }

    if (pub_cloud_occupied_.getNumSubscribers() > 0) {
        pub_cloud_occupied_.publish(getOctomapPcd());
    }
    if (pub_cloud_free_.getNumSubscribers() > 0) {
        pub_cloud_free_.publish(getOctomapPcdFree());
    }
}

//**************************[addPointcloudCallbackSub]*************************
void cOctreePaNode::addPointcloudCallbackSub(
  const sensor_msgs::PointCloud2ConstPtr &msg) {

    if (!updateTime(msg->header.stamp)) {
        tf_listener_.clear();
        return;
    }

    tf::StampedTransform transform;

    try {
        tf_listener_.waitForTransform(rosparams_base_.output_frame_,
          msg->header.frame_id, msg->header.stamp, ros::Duration(0.2));
        tf_listener_.lookupTransform(rosparams_base_.output_frame_,
          msg->header.frame_id, msg->header.stamp, transform);
    } catch (tf::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return;
    }

    if (addCloud(*msg, addparams_, transform)) {
        count_cloud_++;
        publishOctomap();
    }
}

//**************************[addPointcloudCallbackSub]*************************
void cOctreePaNode::addPointcloudOldCallbackSub(
  const sensor_msgs::PointCloudConstPtr &msg) {

    if (!updateTime(msg->header.stamp)) {
        tf_listener_.clear();
        return;
    }

    tf::StampedTransform transform;

    try {
        tf_listener_.waitForTransform(rosparams_base_.output_frame_,
          msg->header.frame_id, msg->header.stamp, ros::Duration(0.2));
        tf_listener_.lookupTransform(rosparams_base_.output_frame_,
          msg->header.frame_id, msg->header.stamp, transform);
    } catch (tf::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return;
    }

    if (addCloud(*msg, addparams_, transform)) {
        count_cloud_old_++;
        publishOctomap();
    }
}

//**************************[addLaserCallbackSub]******************************
void cOctreePaNode::addLaserCallbackSub(
  const sensor_msgs::LaserScanConstPtr &msg) {

    if (!updateTime(msg->header.stamp)) {
        tf_listener_.clear();
        return;
    }

    tf::StampedTransform transform;

    try {
        tf_listener_.waitForTransform(rosparams_base_.output_frame_,
          msg->header.frame_id, msg->header.stamp, ros::Duration(0.2));
        tf_listener_.lookupTransform(rosparams_base_.output_frame_,
          msg->header.frame_id, msg->header.stamp, transform);
    } catch (tf::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return;
    }

    updateTime(msg->header.stamp);
    if (addCloud(*msg, addparams_, transform)) {
        count_laser_++;
        publishOctomap();
    }
}


//**************************[clearCallbackSrv]*********************************
bool cOctreePaNode::clearCallbackSrv(std_srvs::Empty::Request  &req,
  std_srvs::Empty::Response &res) {

    ROS_INFO("cOctreePaNode::clear()");

    count_cloud_     = 0;
    count_cloud_old_ = 0;
    count_laser_     = 0;

    clear();
    tf_listener_.clear();

    return true;
}

//**************************[getSizeCallbackSrv]*******************************
bool cOctreePaNode::getSizeCallbackSrv (
  octomap_pa::OctomapPaGetSize::Request  &req,
  octomap_pa::OctomapPaGetSize::Response &res) {

    ROS_INFO("cOctreePaNode::getsize()");

    res.size = size();
    res.memoryusage = (int64_t) memoryUsage();

    res.count_cloud     = count_cloud_    ;
    res.count_cloud_old = count_cloud_old_;
    res.count_laser     = count_laser_    ;

    return true;
}

//**************************[saveCallbackSrv]**********************************
bool cOctreePaNode::saveCallbackSrv(
  octomap_pa::OctomapPaFileName::Request  &req,
  octomap_pa::OctomapPaFileName::Response &res) {

    ROS_INFO_STREAM("cOctreePaNode::save(" << req.filename << ")");

    std::string filename;
    filename = req.filename;
    cParameterPaRos par;
    par.replaceFindpack(filename);

    //res.ok = writeBinary(filename);
    res.ok = this->write(filename);

    return res.ok;
}

//**************************[loadCallbackSrv]**********************************
bool cOctreePaNode::loadCallbackSrv(
  octomap_pa::OctomapPaFileName::Request  &req,
  octomap_pa::OctomapPaFileName::Response &res) {

    ROS_INFO_STREAM("cOctreePaNode::load(" << req.filename << ")");

    std::string filename;
    filename = req.filename;
    cParameterPaRos par;
    par.replaceFindpack(filename);

    // res.ok = readBinary(filename);
    res.ok = readFull(filename);

    publishOctomap();
    return res.ok;
}
