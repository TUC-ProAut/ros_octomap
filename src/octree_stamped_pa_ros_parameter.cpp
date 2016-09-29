/******************************************************************************
*                                                                             *
* octree_stamped_pa_ros_parameter.cpp                                         *
* ===================================                                         *
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

// local headers
#include "octree_stamped_pa_ros_parameter.h"

//**************************[cOctreeStampedPaRosParameter]*********************
cOctreeStampedPaRosParameter::cOctreeStampedPaRosParameter() {

    // degrading parameters
    degrading_time_           = 60.0;
    auto_degrading_           = true;
    auto_degrading_intervall_ = 2.0 ;
}

//**************************[cOctreeStampedPaRosParameter]*********************
cOctreeStampedPaRosParameter::cOctreeStampedPaRosParameter(
    const cOctreeStampedPaRosParameter &other) {

    *this = other;
}

//**************************[operator =]***************************************
cOctreeStampedPaRosParameter& cOctreeStampedPaRosParameter::operator = (
    const cOctreeStampedPaRosParameter &other) {

    // degrading parameters
    degrading_time_           = other.degrading_time_          ;
    auto_degrading_           = other.auto_degrading_          ;
    auto_degrading_intervall_ = other.auto_degrading_intervall_;

    return *this;
}
