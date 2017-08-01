/******************************************************************************
*                                                                             *
* addcloud_parameter.cpp                                                      *
* ======================                                                      *
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
* Copyright (c) 2015-2017, Peter Weissig, Technische Universität Chemnitz     *
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

// local headers
#include "octomap_pa/addcloud_parameter.h"

//**************************[cAddCloudParameter]*******************************
cAddCloudParameter::cAddCloudParameter() {

    // octomap parameter
    map_prob_hit_  = 0.70;
    map_prob_miss_ = 0.40;

    // transform for pointcloud
    pcd_explicit_transform_ = true;

    // voxel filter for pointcloud
    pcd_voxel_active_                       = true;
    pcd_voxel_explicit_                     = false;
    pcd_voxel_explicit_relative_resolution_ =   0.5;
}

//**************************[cAddCloudParameter]*******************************
cAddCloudParameter::cAddCloudParameter(const cAddCloudParameter &other) {

    *this = other;
}

//**************************[operator =]***************************************
cAddCloudParameter& cAddCloudParameter::operator = (
    const cAddCloudParameter &other) {

    // octomap parameter
    map_prob_hit_  = other.map_prob_hit_ ;
    map_prob_miss_ = other.map_prob_miss_;

    // transform for pointcloud
    pcd_explicit_transform_ = other.pcd_explicit_transform_;

    // voxel filter and transform for pointcloud
    pcd_voxel_explicit_relative_resolution_ =
      other.pcd_voxel_explicit_relative_resolution_;
    pcd_voxel_active_   = other.pcd_voxel_active_  ;
    pcd_voxel_explicit_ = other.pcd_voxel_explicit_;

    return *this;
}
