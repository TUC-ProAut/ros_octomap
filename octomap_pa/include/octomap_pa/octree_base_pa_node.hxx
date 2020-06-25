/******************************************************************************
*                                                                             *
* octree_base_pa_node.hxx                                                     *
* =======================                                                     *
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
#include "octomap_pa/octree_base_pa_node.h"

// ros headers
#include <parameter_pa/parameter_pa_ros.h>
#include <pcl_conversions/pcl_conversions.h>

// standard headers
#include <string>

//**************************[cOctreeBasePaNode]********************************
template <typename BASECLASS>
  cOctreeBasePaNode<BASECLASS>::cOctreeBasePaNode(
  const std::string nodename, const double resolution,
  const ros::Duration tf_listener_buffersize) :

  nodename_(nodename), BASECLASS(resolution),
  tf_listener_(tf_listener_buffersize, true) {

    cParameterPaRos paramloader;

    paramloader.load("~/output_frame",
      BASECLASS::rosparams_base_.output_frame_);

    // octomap parameter
    double temp;

    temp = 0.1 ;
    paramloader.load("~/map_resolution"    , temp);
    BASECLASS::setResolution      (temp);

    temp = 0.5 ;
    paramloader.load("~/map_prob_threshold", temp);
    BASECLASS::setOccupancyThres  (temp);

    temp = 0.12;
    paramloader.load("~/map_clamp_min"     , temp);
    BASECLASS::setClampingThresMin(temp);

    temp = 0.97;
    paramloader.load("~/map_clamp_max"     , temp);
    BASECLASS::setClampingThresMax(temp);

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
          &cOctreeBasePaNode<BASECLASS>::addPointcloudCallbackSub, this);
    }

    // Subscriber for pointclouds (old format)
    if (nodeparams_.topic_in_cloud_old_ != "") {
        sub_cloud_old_ = nh_.subscribe<sensor_msgs::PointCloud>(
          nodeparams_.topic_in_cloud_old_, 10,
          &cOctreeBasePaNode<BASECLASS>::addPointcloudOldCallbackSub, this);
    }
    // Subscriber for laserscans
    if (nodeparams_.topic_in_laser_ != "") {
        sub_laser_ = nh_.subscribe<sensor_msgs::LaserScan>(
          nodeparams_.topic_in_laser_, 10,
          &cOctreeBasePaNode<BASECLASS>::addLaserCallbackSub, this);
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
      &cOctreeBasePaNode<BASECLASS>::clearCallbackSrv, this);
    // service for resetting the octomap
    // (clearing & changing basic properties)
    srv_reset_ = nh_.advertiseService(str_service + "reset",
      &cOctreeBasePaNode<BASECLASS>::resetCallbackSrv, this);

    // service for setting the degrading config of the octomap
    srv_setconfig_degrading_   =
      nh_.advertiseService(str_service + "setconfig_degrading",
      &cOctreeBasePaNode<BASECLASS>::setConfigDegradingCallbackSrv, this);
    // service for setting the insertion config of the octomap
    srv_setconfig_insertion_ =
      nh_.advertiseService(str_service + "setconfig_insertion",
      &cOctreeBasePaNode<BASECLASS>::setConfigInsertionCallbackSrv, this);
    // service for receiving all config of the octomap
    srv_getconfig_ = nh_.advertiseService(str_service + "getconfig",
      &cOctreeBasePaNode<BASECLASS>::getConfigCallbackSrv, this);

    // service for adding a new pointcloud to the octomap
    srv_addcloud_ = nh_.advertiseService(str_service + "addcloud",
      &cOctreeBasePaNode<BASECLASS>::addCloudCallbackSrv, this);
    // service for receiving the current octomap as pointcloud
    srv_getcloud_ = nh_.advertiseService(str_service + "getcloud",
      &cOctreeBasePaNode<BASECLASS>::getCloudCallbackSrv, this);
    // service for receiving the size of the octomap
    srv_getsize_ = nh_.advertiseService(str_service + "getsize",
      &cOctreeBasePaNode<BASECLASS>::getSizeCallbackSrv, this);

    // service for saving the octomap
    srv_save_ = nh_.advertiseService(str_service + "save",
      &cOctreeBasePaNode<BASECLASS>::saveCallbackSrv, this);
    // service for loading a octomap
    srv_load_ = nh_.advertiseService(str_service + "load",
      &cOctreeBasePaNode<BASECLASS>::loadCallbackSrv, this);

    // count number of inserted pointclouds
    count_cloud_     = 0;
    count_cloud_old_ = 0;
    count_laser_     = 0;
}

//**************************[~cOctreeBasePaNode]*******************************
template <typename BASECLASS>
  cOctreeBasePaNode<BASECLASS>::~cOctreeBasePaNode() {
}



//**************************[publishOctomap]***********************************
template <typename BASECLASS>
  void cOctreeBasePaNode<BASECLASS>::publishOctomap() {

    if (pub_octomap_.getNumSubscribers() > 0) {
        pub_octomap_.publish(BASECLASS::getOctomap());
    }
    if (pub_octomap_full_.getNumSubscribers() > 0) {
        pub_octomap_.publish(BASECLASS::getOctomapFull());
    }

    if (pub_cloud_occupied_.getNumSubscribers() > 0) {
        pub_cloud_occupied_.publish(BASECLASS::getOctomapPcd());
    }
    if (pub_cloud_free_.getNumSubscribers() > 0) {
        pub_cloud_free_.publish(BASECLASS::getOctomapPcdFree());
    }
}

//**************************[getConfig]****************************************
template <typename BASECLASS>
  octomap_pa_msgs::Config cOctreeBasePaNode<BASECLASS>::getConfig() {

    // init variables
    octomap_pa_msgs::Config result;

    // base config
    result.base.map_resolution = BASECLASS::getResolution();
    result.base.output_frame   = BASECLASS::rosparams_base_.output_frame_;

    // config for adding pointclouds
    result.insertion.map_clamp_min = BASECLASS::getClampingThresMin();
    result.insertion.map_clamp_max = BASECLASS::getClampingThresMax();

    result.insertion.map_prob_hit       = addparams_.map_prob_hit_ ;
    result.insertion.map_prob_miss      = addparams_.map_prob_miss_;
    result.insertion.map_prob_threshold = BASECLASS::getOccupancyThres();

    result.insertion.pcd_voxel_active       = addparams_.pcd_voxel_active_;
    result.insertion.pcd_voxel_explicit     = addparams_.pcd_voxel_explicit_;
    result.insertion.pcd_explicit_transform =
      addparams_.pcd_explicit_transform_;
    result.insertion.pcd_voxel_explicit_relative_resolution =
      addparams_.pcd_voxel_explicit_relative_resolution_;

    // config for degrading (not used by default)
    result.degrading.auto_degrading           = false;
    result.degrading.auto_degrading_intervall = -1;
    result.degrading.degrading_time           = -1;

    // return result
    return result;
}

//**************************[setConfigInsertion]*******************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::setConfigInsertion(
  const octomap_pa_msgs::ConfigInsertion &config) {

    BASECLASS::setClampingThresMin(config.map_clamp_min);
    BASECLASS::setClampingThresMax(config.map_clamp_max);

    addparams_.map_prob_hit_   = config.map_prob_hit ;
    addparams_.map_prob_miss_  = config.map_prob_miss;
    BASECLASS::setOccupancyThres(config.map_prob_threshold);

    addparams_.pcd_voxel_active_       = config.pcd_voxel_active  ;
    addparams_.pcd_voxel_explicit_     = config.pcd_voxel_explicit;
    addparams_.pcd_explicit_transform_ = config.pcd_explicit_transform;
    addparams_.pcd_voxel_explicit_relative_resolution_ =
      config.pcd_voxel_explicit_relative_resolution;

    return true;
}

//**************************[setConfigDegrading]*******************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::setConfigDegrading(
  const octomap_pa_msgs::ConfigDegrading &config) {

    ROS_INFO_STREAM(nodename_ << " degrading config has no effect!");
    return false;
}


//**************************[addPointcloudCallbackSub]*************************
template <typename BASECLASS>
  void cOctreeBasePaNode<BASECLASS>::addPointcloudCallbackSub(
  const sensor_msgs::PointCloud2ConstPtr &msg) {

    // update/check timing & try to get current transform
    tf::Transform transform;
    if (!updateTimeAndGetTF(msg->header, transform)) {
        return;
    }

    // insert pointcloud
    if (BASECLASS::addCloud(*msg, addparams_, transform)) {
        count_cloud_++;
        internal_node_update();
        publishOctomap();
    }
}

//**************************[addPointcloudCallbackSub]*************************
template <typename BASECLASS>
  void cOctreeBasePaNode<BASECLASS>::addPointcloudOldCallbackSub(
  const sensor_msgs::PointCloudConstPtr &msg) {

    // update/check timing & try to get current transform
    tf::Transform transform;
    if (!updateTimeAndGetTF(msg->header, transform)) {
        return;
    }

    // insert pointcloud
    if (BASECLASS::addCloud(*msg, addparams_, transform)) {
        count_cloud_old_++;
        internal_node_update();
        publishOctomap();
    }
}

//**************************[addLaserCallbackSub]******************************
template <typename BASECLASS>
  void cOctreeBasePaNode<BASECLASS>::addLaserCallbackSub(
  const sensor_msgs::LaserScanConstPtr &msg) {

    // update/check timing & try to get current transform
    tf::Transform transform;
    if (!updateTimeAndGetTF(msg->header, transform)) {
        return;
    }

    // insert pointcloud
    if (BASECLASS::addCloud(*msg, addparams_, transform)) {
        count_laser_++;
        internal_node_update();
        publishOctomap();
    }
}



//**************************[clearCallbackSrv]*********************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::clearCallbackSrv(
  std_srvs::Empty::Request  &req,
  std_srvs::Empty::Response &res) {

    ROS_INFO_STREAM(nodename_ << "::clear()");

    count_cloud_     = 0;
    count_cloud_old_ = 0;
    count_laser_     = 0;

    BASECLASS::clear();
    tf_listener_.clear();

    return true;
}

//**************************[resetCallbackSrv]*********************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::resetCallbackSrv(
  octomap_pa_msgs::Reset::Request  &req,
  octomap_pa_msgs::Reset::Response &res) {

    // init result
    res.ok = false;

    // call clear callback
    std_srvs::Empty msg;
    if (!clearCallbackSrv(msg.request, msg.response)){
        return false;
    }

    // print infos for reset
    ROS_INFO_STREAM(nodename_ <<
      "::reset(resolution = " << req.config.map_resolution <<
      "; frame = '" << req.config.output_frame << "')");

    // set internal parametrs
    BASECLASS::setResolution(req.config.map_resolution);
    BASECLASS::rosparams_base_.output_frame_ = req.config.output_frame;

    // store parameters to parameterserver
    //nh_.setParam("~/map_resolution", req.config.map_resolution);
    //nh_.setParam("~/output_frame", req.config.output_frame  );

    // return result
    res.ok = true;
    return true;
}



//**************************[setConfigDegradingCallbackSrv]********************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::setConfigDegradingCallbackSrv(
  octomap_pa_msgs::SetConfigDegrading::Request  &req,
  octomap_pa_msgs::SetConfigDegrading::Response &res) {

    res.ok = setConfigDegrading(req.config);
    return res.ok;
}

//**************************[setConfigDegradingCallbackSrv]********************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::setConfigInsertionCallbackSrv(
  octomap_pa_msgs::SetConfigInsertion::Request  &req,
  octomap_pa_msgs::SetConfigInsertion::Response &res) {

    res.ok = setConfigInsertion(req.config);
    return res.ok;
}

//**************************[getConfigCallbackSrv]*****************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::getConfigCallbackSrv(
  octomap_pa_msgs::GetConfig::Request  &req,
  octomap_pa_msgs::GetConfig::Response &res) {

    res.config = getConfig();
    return true;
}



//**************************[addCloudCallbackSrv]******************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::addCloudCallbackSrv(
  octomap_pa_msgs::AddCloud::Request  &req,
  octomap_pa_msgs::AddCloud::Response &res) {

    // init result
    res.ok = false;

    // update/check timing & try to get current transform
    tf::Transform transform;
    if (!updateTimeAndGetTF(req.cloud.header, transform)) {
        return false;
    }

    // insert pointcloud
    if (BASECLASS::addCloud(req.cloud, addparams_, transform)) {
        res.ok = true;
        count_cloud_++;
        internal_node_update();
        // no publishing
    }

    return res.ok;
}

//**************************[addCloudTfCallbackSrv]****************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::addCloudTfCallbackSrv(
  octomap_pa_msgs::AddCloudTf::Request  &req,
  octomap_pa_msgs::AddCloudTf::Response &res) {

    // init result
    res.ok = false;

    // update timing (to be consistent with the other addcloud-callbacks)
    BASECLASS::updateTime(req.cloud.header.stamp);

    // convert transform from geometry to TF
    tf::Transform tf_transform;
    tf::transformMsgToTF(req.transform, tf_transform);

    // insert pointcloud
    if (BASECLASS::addCloud(req.cloud, addparams_, tf_transform)) {
        res.ok = true;
        count_cloud_++;
        internal_node_update();
        // no publishing
    }

    return res.ok;
}

//**************************[getCloudCallbackSrv]******************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::getCloudCallbackSrv(
  octomap_pa_msgs::GetCloud::Request  &req,
  octomap_pa_msgs::GetCloud::Response &res) {

    // init variables for shallow copy (moving data without copying)
    pcl::PCLPointCloud2 pcl_cloud;

    // select between occupied or free voxels
    if (req.occupied) {
        pcl_conversions::moveToPCL(*BASECLASS::getOctomapPcd()    , pcl_cloud);
    } else {
        pcl_conversions::moveToPCL(*BASECLASS::getOctomapPcdFree(), pcl_cloud);
    }

    // move data back to ros msg
    pcl_conversions::moveFromPCL(pcl_cloud, res.cloud);


    // return config
    res.config = getConfig();
    return true;
}

//**************************[getSizeCallbackSrv]*******************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::getSizeCallbackSrv (
  octomap_pa_msgs::GetSize::Request  &req,
  octomap_pa_msgs::GetSize::Response &res) {

    ROS_INFO_STREAM(nodename_ << "::getsize()");

    res.size = BASECLASS::size();
    res.memoryusage = (int64_t) BASECLASS::memoryUsage();

    res.count_cloud     = count_cloud_    ;
    res.count_cloud_old = count_cloud_old_;
    res.count_laser     = count_laser_    ;

    return true;
}



//**************************[saveCallbackSrv]**********************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::saveCallbackSrv(
  octomap_pa_msgs::FileName::Request  &req,
  octomap_pa_msgs::FileName::Response &res) {

    ROS_INFO_STREAM(nodename_ << "::save(" << req.filename << ")");

    std::string filename;
    filename = req.filename;
    cParameterPaRos par;
    par.replaceFindpack(filename);

    //res.ok = writeBinary(filename);
    res.ok = BASECLASS::write(filename);

    return res.ok;
}

//**************************[loadCallbackSrv]**********************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::loadCallbackSrv(
  octomap_pa_msgs::FileName::Request  &req,
  octomap_pa_msgs::FileName::Response &res) {

    ROS_INFO_STREAM(nodename_ << "::load(" << req.filename << ")");

    std::string filename;
    filename = req.filename;
    cParameterPaRos par;
    par.replaceFindpack(filename);

    // res.ok = readBinary(filename);
    res.ok = BASECLASS::readFull(filename);

    publishOctomap();
    return res.ok;
}



//**************************[updateTimeAndGetTF]*******************************
template <typename BASECLASS>
  bool cOctreeBasePaNode<BASECLASS>::updateTimeAndGetTF(
  const std_msgs::Header header,
  tf::Transform &transform) {

    // update & check timing
    if (!BASECLASS::updateTime(header.stamp)) {
        tf_listener_.clear();
        return false;
    }

    // get current transform
    tf::StampedTransform stamped_transform;

    try {
        // wait for transform
        tf_listener_.waitForTransform(
          BASECLASS::rosparams_base_.output_frame_,
          header.frame_id, header.stamp, ros::Duration(0.2));
        // load transform
        tf_listener_.lookupTransform(
          BASECLASS::rosparams_base_.output_frame_,
          header.frame_id, header.stamp, stamped_transform);
    } catch (tf::TransformException &ex){
        ROS_WARN("%s", ex.what());
        return false;
    }

    // return result
    transform = stamped_transform;
    return true;
  }
