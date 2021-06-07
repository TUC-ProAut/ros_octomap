/******************************************************************************
*                                                                             *
* octree_base_pa_ros.h                                                        *
* ====================                                                        *
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
* Copyright (c) 2015-2021, Peter Weissig, Technische Universität Chemnitz     *
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

#ifndef __OCTREE_BASE_PA_ROS_H
#define __OCTREE_BASE_PA_ROS_H

// local headers
#include "octomap_pa/addcloud_parameter.h"
#include "octomap_pa/octree_base_pa_ros_parameter.h"

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

//**************************[cOctreeBasePaRos]*********************************
template <typename OCTREE>
class cOctreeBasePaRos : public OCTREE {
  public:

    typedef OCTREE                   TreeTypeBase;
    typedef cOctreeBasePaRos<OCTREE> TreeTypeFull;

    typedef          pcl::PointCloud<pcl::PointXYZ>      PclPointCloud;
    typedef typename pcl::PointCloud<pcl::PointXYZ>::Ptr PclPointCloudPtr;
    typedef typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr
        PclPointCloudConstPtr;

    typedef ::octomap::OcTreeKey OctKey;

    //! default constructor
    cOctreeBasePaRos(double resolution);

    //! default destructor
    virtual ~cOctreeBasePaRos();

    //! clear local timestamps with octomap
    virtual void clear(void);

    //! function for filtering and adding a pointcloud
    //! (call updateTime() before)
    bool addCloud(const sensor_msgs::PointCloud2ConstPtr &cloud,
      const cAddCloudParameter &params,
      const tf::Transform transform = tf::Transform::getIdentity());
    //! function for filtering and adding a pointcloud (old format)
    //! (call updateTime() before)
    bool addCloud(const sensor_msgs::PointCloudConstPtr &cloud,
      const cAddCloudParameter &params,
      const tf::Transform transform = tf::Transform::getIdentity());
    //! function for filtering and adding a pointcloud (laserscan)
    //! (call updateTime() before)
    bool addCloud(const sensor_msgs::LaserScanConstPtr &cloud,
      const cAddCloudParameter &params,
      const tf::Transform transform = tf::Transform::getIdentity());

    //! function for getting the binary octomap
    octomap_msgs::OctomapPtr getOctomap(void) const;
    //! function for getting the full octomap
    octomap_msgs::OctomapPtr getOctomapFull(void) const;

    //! function for getting the pointcloud equivalent of the octomap
    sensor_msgs::PointCloud2Ptr getOctomapPcd(
      const int tree_depth = 0, const bool expand = false) const;
    //! similar to getOctomapPcd, but only returning just empty voxels
    sensor_msgs::PointCloud2Ptr getOctomapPcdFree(
      const int tree_depth = 0, const bool expand = false) const;

    //! function for updating all timestamps (Insertion & Output)
    //! returns false if an time jump backwards is detected
    //! this might be used for resetting tf_listener
    bool updateTime(const ros::Time &time);
    //! function for returning the time the octomap was last updated
    //! (instead of this function use updateTime() )
    virtual ros::Time getLastInsertionTime(void) const;
    //! function for setting the time the octomap was last updated
    //! (instead of this function use updateTime() )
    virtual void setLastInsertionTime(const ros::Time &time);
    //! function for returning the time of output messages
    //! (instead of this function use updateTime() )
    ros::Time getOutputTime(void) const;
    //! function for setting the time of output messages
    //! (instead of this function use updateTime() )
    void setOutputTime(const ros::Time &time);

    //! functions for converting from point (geometry_msg) to key
    OctKey pointToKey(const geometry_msgs::Point &point) const;
    //! function for converting from key to point (geometry_msg)
    geometry_msgs::PointPtr keyToPoint(const OctKey &key) const;
    //! function for converting from key to real coordinates
    void keyToPoint(const OctKey &key, double &x, double &y, double &z) const;

    //! helper function for calculating child key
    //! child_pos(bitmask) 0(x)[0=neg;1=pos] | 1(y)[0=neg;1=pos] |
    //!   2(z)[0=neg;1=pos]
    //! child_pos is the same as for octomap::OctreeNode::getChild()
    //! returns true if child key exists
    bool getChildKey(const OctKey &current, const int current_level,
      OctKey &child, const int child_pos) const;
    //! helper function for calculating parent key
    //! returns true if parent key exists
    bool getParentKey(const OctKey &current, const int current_level,
      OctKey &parent) const;

    //! trying to read the given file into the current OcTree
    bool readFull(const std::string& filename);

    //! parameters
    cOctreeBasePaRosParameter rosparams_base_;

  protected:
    //! internal variable for storing last insertion time
    ros::Time last_insertion_time_;
    //! internal variable for storing current output time
    ros::Time current_output_time_;

    //! internal function for adding a pointlcoud -
    //! the cloud will be manipulated !
    bool addCloud(const PclPointCloudPtr &cloud,
      const cAddCloudParameter &params,
      const tf::Transform &transform);

    //! helper function for getOctomapPcd...
    void getOctomapPcdSub(const OctKey &key, const int current_level,
      const int min_level, PclPointCloud &cloud) const;

    //! helper function for getChildKey
    void getChildKeySimple(const OctKey &current, const int current_level,
      OctKey &child, const int child_pos) const;
    //! helper function for getParentKey
    void getParentKeySimple(const OctKey &current, const int current_level,
      OctKey &parent) const;
};

#include "octomap_pa/octree_base_pa_ros.hxx"

#endif // __OCTREE_BASE_PA_ROS_H
