/******************************************************************************
*                                                                             *
* octree_stamped_pa_ros.h                                                     *
* =======================                                                     *
*                                                                             *
*******************************************************************************
*                                                                             *
* github repository                                                           *
*   https://github.com/peterweissig/ros_octomap                               *
*                                                                             *
* Chair of Automation Technology, Technische Universität Chemnitz             *
*   https://www.tu-chemnitz.de/etit/proaut                                    *
*                                                                             *
*******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2015-2016, Peter Weissig, Technische Universität Chemnitz     *
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

#ifndef __OCTREE_STAMPED_PA_ROS_H
#define __OCTREE_STAMPED_PA_ROS_H

// local headers
#include "time_pa.h"
#include "octree_base_pa_ros.h"
#include "octree_stamped_pa.h"
#include "octree_stamped_pa_ros_parameter.h"

// ros headers
#include <ros/ros.h>

//**************************[cOctreeStampedPaRos]******************************
class cOctreeStampedPaRos : public cOctreeBasePaRos<cOcTreeStampedPa> {
  public:
    typedef cOctreeBasePaRos<cOcTreeStampedPa> TreeTypeBase;

    //! default constructor
    cOctreeStampedPaRos(const double resolution);

    //! default destructor
    virtual ~cOctreeStampedPaRos();

    //! degrading outdated nodes
    void degradeOutdatedNodes(void);

    //! function for returning the time the octomap was last updated
    ros::Time getLastInsertionTime(void) const;
    //! function for setting the time the octomap was last updated
    void setLastInsertionTime(const ros::Time &time);

    //! function for converting from cTimePa to ros::Time
    ros::Time timeToRos(const cTimePa &time) const;
    //! function for converting from ros::Time to cTimePa
    cTimePa timeFromRos(const ros::Time &time) const;

    //! parameters
    cOctreeStampedPaRosParameter rosparams_;

  protected:
    ros::Time last_degrading_time_;

    //! helper function for automatic degrading
    void checkDegrading(void);
};

#endif // __OCTREE_STAMPED_PA_ROS_H
