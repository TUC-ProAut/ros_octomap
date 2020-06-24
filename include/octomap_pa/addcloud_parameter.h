/******************************************************************************
*                                                                             *
* addcloud_parameter.h                                                        *
* ====================                                                        *
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

#ifndef __ADDCLOUD_PARAMETER_H
#define __ADDCLOUD_PARAMETER_H

//**************************[cAddCloudParameter]*******************************
class cAddCloudParameter {
  public:
    cAddCloudParameter(void);
    cAddCloudParameter(const cAddCloudParameter &other);
    cAddCloudParameter& operator = (const cAddCloudParameter &other);

    // octomap parameter
    //! octomap parameter: probability for a "hit"
    double map_prob_hit_;
    //! octomap parameter: probability for a "miss"
    double map_prob_miss_;

    //! pointcloud insertion parameter: use pcl-transform instead of
    //! octomap-transform (speeds up)
    bool pcd_explicit_transform_;

    //! pointcloud insertion parameter: use voxel-filter (speeds up)
    bool pcd_voxel_active_;
    //! pointcloud insertion parameter: use pcl-filter instead of
    //! octomap-filter (slows down)
    bool pcd_voxel_explicit_;
    //! pointcloud insertion parameter: relative resolution of pcl-filter
    double pcd_voxel_explicit_relative_resolution_;
};

#endif // __ADDCLOUD_PARAMETER_H
