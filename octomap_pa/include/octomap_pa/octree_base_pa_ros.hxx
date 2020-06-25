/******************************************************************************
*                                                                             *
* octree_base_pa_ros.hxx                                                      *
* ======================                                                      *
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
#include "octomap_pa/octree_base_pa_ros.h"

// ros headers
#include <pcl_ros/point_cloud.h>

#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

// additional libraries
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>

// standard headers
#include <string>
#include <sstream>
#include <vector>

//**************************[cOctreeBasePaRos]*********************************
template <typename OCTREE>
  cOctreeBasePaRos<OCTREE>::cOctreeBasePaRos(double resolution) :
  OCTREE(resolution) {

}

//**************************[~cOctreeBasePaRos]********************************
template <typename OCTREE>
  cOctreeBasePaRos<OCTREE>::~cOctreeBasePaRos() {
}

//**************************[clear]********************************************
template <typename OCTREE>
  void cOctreeBasePaRos<OCTREE>::clear() {

    setLastInsertionTime(ros::Time());
    setOutputTime(ros::Time());

    OCTREE::clear();
}

//**************************[addCloud]*****************************************
template <typename OCTREE>
  bool cOctreeBasePaRos<OCTREE>::addCloud(
  const sensor_msgs::PointCloud2 &cloud,
  const cAddCloudParameter &params,
  const tf::Transform transform) {

    PclPointCloudPtr cloud_pcl(new PclPointCloud);
    pcl::fromROSMsg(cloud, *cloud_pcl);

    return addCloud(cloud_pcl, params, transform);
}

//**************************[addCloud]*****************************************
template <typename OCTREE>
  bool cOctreeBasePaRos<OCTREE>::addCloud(
  const sensor_msgs::PointCloud &cloud,
  const cAddCloudParameter &params,
  const tf::Transform transform) {

    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

    return addCloud(cloud2, params, transform);
}

//**************************[addCloud]*****************************************
template <typename OCTREE>
  bool cOctreeBasePaRos<OCTREE>::addCloud(
  const sensor_msgs::LaserScan &cloud,
  const cAddCloudParameter &params,
  const tf::Transform transform) {

    laser_geometry::LaserProjection projector;

    sensor_msgs::PointCloud2 cloud2;
    projector.projectLaser(cloud, cloud2, -1,
      laser_geometry::channel_option::None);

    return addCloud(cloud2, params, transform);
}

//**************************[getOctomap]***************************************
template <typename OCTREE>
  octomap_msgs::OctomapPtr cOctreeBasePaRos<OCTREE>::getOctomap() const {

    // create message
    octomap_msgs::OctomapPtr result(new octomap_msgs::Octomap);
    result->header.frame_id = rosparams_base_.output_frame_;
    result->header.stamp    = getOutputTime();

    octomap_msgs::binaryMapToMsg(*this, *result);

    return result;
}

//**************************[getOctomapFull]***********************************
template <typename OCTREE>
  octomap_msgs::OctomapPtr cOctreeBasePaRos<OCTREE>::getOctomapFull() const {

    // create message
    octomap_msgs::OctomapPtr result(new octomap_msgs::Octomap);
    result->header.frame_id = rosparams_base_.output_frame_;
    result->header.stamp    = getOutputTime();

    octomap_msgs::fullMapToMsg(*this, *result);

    return result;
}

//**************************[getOctomapPcd]************************************
template <typename OCTREE>
  sensor_msgs::PointCloud2Ptr cOctreeBasePaRos<OCTREE>::getOctomapPcd(
  const int tree_depth, const bool expand) const {

    // create pcl-cloud
    PclPointCloudPtr result_pcl(new PclPointCloud);
    result_pcl->header.frame_id = rosparams_base_.output_frame_;
    result_pcl->header.stamp = pcl_conversions::toPCL(
        getOutputTime());
    result_pcl->points.reserve(OCTREE::size());

    // tree depth
    int depth = tree_depth;
    if (tree_depth < 1) { depth+= OCTREE::tree_depth;}

    if (depth <                0) { depth =                0;}
    if (depth > OCTREE::tree_depth) { depth = OCTREE::tree_depth;}
    int level_min = OCTREE::tree_depth - depth;

    // iterate over all leafs
    const typename OCTREE::leaf_iterator it_end = OCTREE::end_leafs();
    for (typename OCTREE::leaf_iterator it = OCTREE::begin_leafs(depth);
      it != it_end; ++it) {

        if (OCTREE::isNodeOccupied(*it)){
            int level_local = OCTREE::tree_depth - it.getDepth();
            if (expand && (level_local != level_min)) {
                getOctomapPcdSub(it.getKey(), level_local,
                  level_min, *result_pcl);
            } else {
                result_pcl->push_back(pcl::PointXYZ(it.getX(), it.getY(),
                  it.getZ()));
            }
        }
    }

    // create message
    sensor_msgs::PointCloud2Ptr result (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*result_pcl,*result);
    return result;
}

//**************************[getOctomapPcdFree]********************************
template <typename OCTREE>
  sensor_msgs::PointCloud2Ptr cOctreeBasePaRos<OCTREE>::getOctomapPcdFree(
  const int tree_depth, const bool expand) const {

    // create pcl-cloud
    PclPointCloudPtr result_pcl(new PclPointCloud);
    result_pcl->header.frame_id = rosparams_base_.output_frame_;
    result_pcl->header.stamp = pcl_conversions::toPCL(
        getOutputTime());
    result_pcl->points.reserve(OCTREE::size());

    // tree depth
    int depth = tree_depth;
    if (tree_depth < 1) { depth+= OCTREE::tree_depth;}

    if (depth <                0) { depth =                0;}
    if (depth > OCTREE::tree_depth) { depth = OCTREE::tree_depth;}
    int level_min = OCTREE::tree_depth - depth;

    // iterate over all leafs
    const typename OCTREE::leaf_iterator it_end = OCTREE::end_leafs();
    for (typename OCTREE::leaf_iterator it = OCTREE::begin_leafs(depth);
      it != it_end; ++it) {

        if (! OCTREE::isNodeOccupied(*it)){
            int level_local = OCTREE::tree_depth - it.getDepth();
            if (expand && (level_local != level_min)) {
                getOctomapPcdSub(it.getKey(), level_local,
                  level_min, *result_pcl);
            } else {
                result_pcl->push_back(pcl::PointXYZ(it.getX(), it.getY(),
                  it.getZ()));
            }
        }
    }

    // create message
    sensor_msgs::PointCloud2Ptr result (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*result_pcl,*result);
    return result;
}

//**************************[getLastInsertionTime]*****************************
template <typename OCTREE>
  ros::Time cOctreeBasePaRos<OCTREE>::getLastInsertionTime(void) const {

    return last_insertion_time_;
}

//**************************[setLastInsertionTime]*****************************
template <typename OCTREE>
  void cOctreeBasePaRos<OCTREE>::setLastInsertionTime(const ros::Time &time) {

    last_insertion_time_ = time;
}

//**************************[getOutputTime]************************************
template <typename OCTREE>
  ros::Time cOctreeBasePaRos<OCTREE>::getOutputTime(void) const {

    return current_output_time_;
}

//**************************[setOutputTime]************************************
template <typename OCTREE>
  void cOctreeBasePaRos<OCTREE>::setOutputTime(const ros::Time &time) {

    current_output_time_ = time;
}

//**************************[updateTime]***************************************
template <typename OCTREE>
  bool cOctreeBasePaRos<OCTREE>::updateTime(const ros::Time &time) {

    setLastInsertionTime(time);

    if (time > getOutputTime()) {
        // everything is in order :-)
        setOutputTime(time);
        return true;
    }

    if (getOutputTime() - time >
      rosparams_base_.timejump_detection_duration) {
        // big time jump - reset output time
        setOutputTime(time);
        return false;
    }

    // minor time jumps (e.g. due to message delay)
    // --> create new timestamp to mimic everything is in order
    setOutputTime(getOutputTime() + rosparams_base_.timejump_increment);
    return true;
}

//**************************[pointToKey]***************************************
template <typename OCTREE>
  typename cOctreeBasePaRos<OCTREE>::OctKey
  cOctreeBasePaRos<OCTREE>::pointToKey(const geometry_msgs::Point &point)
  const {

    OctKey key;
    if (! OCTREE::coordToKeyChecked(
      point.x, point.y, point.z, key)) {
        key[0] = key[1] = key[2] = 0;
    }

    return key;
}

//**************************[keyToPoint]***************************************
template <typename OCTREE>
  geometry_msgs::PointPtr cOctreeBasePaRos<OCTREE>::keyToPoint(const
  OctKey &key) const {

    geometry_msgs::PointPtr result(new geometry_msgs::Point);

    keyToPoint(key, result->x, result->y, result->z);

    return result;
}

//**************************[keyToPoint]***************************************
template <typename OCTREE>
  void cOctreeBasePaRos<OCTREE>::keyToPoint(const OctKey &key,
    double &x, double &y, double &z) const {

    x = OCTREE::keyToCoord(key[0]);
    y = OCTREE::keyToCoord(key[1]);
    z = OCTREE::keyToCoord(key[2]);
}

//**************************[getChildKey]**************************************
template <typename OCTREE>
  bool cOctreeBasePaRos<OCTREE>::getChildKey(const OctKey &current,
  const int current_level, OctKey &child, const int child_pos) const {

    // nodes at voxel level have no children
    if (current_level < 1) {
        return false;
    }

    // root has no parents
    if (current_level >  OCTREE::tree_depth) {
        return false;
    }

    getChildKeySimple(current, current_level, child, child_pos);

    return true;
}

//**************************[getParentKey]*************************************
template <typename OCTREE>
  bool cOctreeBasePaRos<OCTREE>::getParentKey(const OctKey &current,
  const int current_level, OctKey &parent) const {

    // nodes at voxel level have no children
    if (current_level < 0) {
        return false;
    }

    // root has no parents
    if (current_level >=  OCTREE::tree_depth) {
        return false;
    }

    getParentKeySimple(current, current_level, parent);

    return true;
}

//**************************[readFull]*****************************************
template <typename OCTREE>
  bool cOctreeBasePaRos<OCTREE>::readFull(const std::string& filename) {

    // clear current octree
    clear();

    // loading tree from file
    ::octomap::AbstractOcTree* tree_temp;
    try {
        tree_temp = ::octomap::AbstractOcTree::read(filename);
    } catch (std::exception &ex){
        return false;
    }

    // check tree type
    if (OCTREE::getTreeType() != tree_temp->getTreeType()) {
        delete tree_temp;
        return false;
    }

    // get resolution
    OCTREE::setResolution(tree_temp->getResolution());

    // try to convert tree
    OCTREE* tree_temp2;
    try {
        tree_temp2 = dynamic_cast< OCTREE *> (tree_temp);
    } catch (std::exception &ex){
        delete tree_temp;

        return false;
    }

    // swap trees (content and size wil be swapped)
    OCTREE::swapContent(*tree_temp2);

    // delete temp tree
    delete tree_temp;
}

//**************************[addCloud]*****************************************
template <typename OCTREE>
  bool cOctreeBasePaRos<OCTREE>::addCloud(const PclPointCloudPtr &cloud,
  const cAddCloudParameter &params,
  const tf::Transform &transform) {

    // explicit transform
    if (params.pcd_explicit_transform_) {
        pcl_ros::transformPointCloud(*cloud, *cloud, transform);
    }

    PclPointCloudPtr pcd_temp;
    // voxel filter
    if (params.pcd_voxel_explicit_ && params.pcd_voxel_active_) {
        pcd_temp = PclPointCloudPtr(new PclPointCloud);
        pcl::VoxelGrid<pcl::PointXYZ> voxelfilter;

        double voxel_res = OCTREE::getResolution() *
          params.pcd_voxel_explicit_relative_resolution_;

        voxelfilter.setInputCloud (cloud);
        voxelfilter.setLeafSize (voxel_res, voxel_res, voxel_res);
        voxelfilter.filter(*pcd_temp);
    } else {
        pcd_temp = cloud;
    }

    // change type of pointcloud (from pcl:: to ::octomap::)
    ::octomap::Pointcloud pcd_octo;
    sensor_msgs::PointCloud2 pcd_rosmsg;

    #if ROS_VERSION_MINIMUM(1, 11, 0)
        // everything starting with ROS indigo
        pcl::toROSMsg(*pcd_temp, pcd_rosmsg);
        ::octomap::pointCloud2ToOctomap(pcd_rosmsg, pcd_octo);
    #else //#if ROS_VERSION_MINIMUM(1, 11, 0)
        // everything up to ROS hydro
        ::octomap::pointcloudPCLToOctomap(*pcd_temp, pcd_octo);
    #endif //#if ROS_VERSION_MINIMUM(1, 11, 0)

    // change type of transform (from ros:: to ::octomap::)
    ::octomap::point3d p(
        transform.getOrigin().x(),
        transform.getOrigin().y(),
        transform.getOrigin().z());

    octomath::Quaternion q(
        transform.getRotation().w(),
        transform.getRotation().x(),
        transform.getRotation().y(),
        transform.getRotation().z());

    octomath::Pose6D pose(p,q);


    // set internal parameters
    OCTREE::setProbHit (params.map_prob_hit_);
    OCTREE::setProbMiss(params.map_prob_miss_);

    //setTimestamp(timeFromRos(pcl_conversions::fromPCL(cloud->header.stamp)));
    // do insertion
    if (!params.pcd_explicit_transform_) {
        OCTREE::insertPointCloud(pcd_octo, ::octomap::point3d(), pose, -1, false,
            params.pcd_voxel_active_ && !params.pcd_voxel_explicit_);
    } else {
        OCTREE::insertPointCloud(pcd_octo, pose.trans(), -1, false,
            params.pcd_voxel_active_ && !params.pcd_voxel_explicit_);
    }

    //checkDegrading();
    return true;
}

//**************************[getOctomapPcdSub]*********************************
template <typename OCTREE>
  void cOctreeBasePaRos<OCTREE>::getOctomapPcdSub(const OctKey &key,
  const int current_level, const int min_level, PclPointCloud &cloud)
  const {

    if (current_level <= min_level) {
        double x,y,z;
        x = OCTREE::keyToCoord(key[0]);
        y = OCTREE::keyToCoord(key[1]);
        z = OCTREE::keyToCoord(key[2]);

        cloud.push_back(pcl::PointXYZ(x,y,z));
        return;
    } else {
        for (int i = 0; i < 8; i++) {
            OctKey child;
            getChildKeySimple(key, current_level, child, i);
            getOctomapPcdSub(child, current_level - 1, min_level, cloud);
        }
    }
}

//**************************[getChildKeySimple]********************************
template <typename OCTREE>
  void cOctreeBasePaRos<OCTREE>::getChildKeySimple(const OctKey &current,
  const int current_level, OctKey &child, const int child_pos) const {

    int key_max         = 1 << OCTREE::tree_depth;
    int key_mask_depth  = 1 << current_level;
    int key_mask_filter = key_max - key_mask_depth;
    int key_mask_child  = key_mask_depth >> 1;
    int key_mask_set    = key_mask_child >> 1;

    for (int i = 0; i < 3; i++) {
        int temp = current[i] & key_mask_filter;

        if (child_pos & (1 << i)) {
            temp|= key_mask_child;
        }
        temp|= key_mask_set;

        child[i] = temp;
    }
}

//**************************[getParentKeySimple]*******************************
template <typename OCTREE>
  void cOctreeBasePaRos<OCTREE>::getParentKeySimple(const OctKey &current,
  const int current_level, OctKey &parent) const {

    int key_max         = 1 << OCTREE::tree_depth;
    int key_mask_depth  = 1 << (current_level + 1);
    int key_mask_filter = key_max - key_mask_depth;
    int key_mask_set    = key_mask_depth >> 1;

    for (int i = 0; i < 3; i++) {
        int temp = current[i] & key_mask_filter;
        temp|= key_mask_set;

        parent[i] = temp;
    }
}
