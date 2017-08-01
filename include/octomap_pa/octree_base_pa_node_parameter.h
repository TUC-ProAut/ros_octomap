/******************************************************************************
*                                                                             *
* octree_base_pa_node_parameter.h                                             *
* ===============================                                             *
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

#ifndef __OCTREE_BASE_PA_NODE_PARAMETER_H
#define __OCTREE_BASE_PA_NODE_PARAMETER_H

// standard headers
#include <string>

//**************************[cOctreeBasePaNodeParameter]***********************
class cOctreeBasePaNodeParameter {
  public:
    cOctreeBasePaNodeParameter(void);

    //! name of the topic for subscribing a pointcloud ("~in_cloud") -
    //! new type (pointcloud2)
    std::string topic_in_cloud_;
    //! name of the topic for subscribing a pointcloud ("~in_cloud_old") -
    //! old type (pointcloud)
    std::string topic_in_cloud_old_;
    //! name of the topic for subscribing a laserscan ("~in_laser")
    std::string topic_in_laser_;

    //! name of the topic for publishing the octomap ("~out_octomap")
    std::string topic_out_octomap_;
    //! name of the topic for publishing the octomap ("~out_octomap_full")
    std::string topic_out_octomap_full_;

    //! name of the topic for publishing free voxels as pointcloud
    //! ("~out_cloud_free")
    std::string topic_out_cloud_free_;
    //! name of the topic for publishing occupied voxels as pointcloud
    //! ("~out_cloud_occupied")
    std::string topic_out_cloud_occupied_;
};

#endif // __OCTREE_BASE_PA_NODE_PARAMETER_H
