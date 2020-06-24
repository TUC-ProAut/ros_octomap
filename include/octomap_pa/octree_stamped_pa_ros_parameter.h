/******************************************************************************
*                                                                             *
* octree_stamped_pa_ros_parameter.h                                           *
* =================================                                           *
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

#ifndef __OCTREE_STAMPED_PA_ROS_PARAMETER_H
#define __OCTREE_STAMPED_PA_ROS_PARAMETER_H

// standard headers
#include <string>

//**************************[cOctreeStampedPaRosParameter]*********************
class cOctreeStampedPaRosParameter {
  public:
    cOctreeStampedPaRosParameter(void);
    cOctreeStampedPaRosParameter(const cOctreeStampedPaRosParameter &other);
    cOctreeStampedPaRosParameter& operator = (const
      cOctreeStampedPaRosParameter &other);

    //! duration how long the outdated nodes will be kept (60s)
    double degrading_time_;

    //! turns on automatic degrading (true)
    bool auto_degrading_;

    //! intervall for automatic degrading (2.0s)
    double auto_degrading_intervall_;
};


#endif // __OCTREE_STAMPED_PA_ROS_PARAMETER_H
