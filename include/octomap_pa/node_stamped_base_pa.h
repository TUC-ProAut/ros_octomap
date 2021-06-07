/******************************************************************************
*                                                                             *
* node_stamped_base_pa.h                                                      *
* ======================                                                      *
*                                                                             *
* File is based on OcTreeStamped.h and OcTreeNode.h                           *
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

#ifndef NODE_STAMPED_BASE_PA_H
#define NODE_STAMPED_BASE_PA_H

// local headers
#include "octomap_pa/time_pa.h"

// standard headers
#include <stdint.h>
#include <iostream>
#include <cstddef>

//**************************[cNodeStampedBasePa]*******************************
template <typename NODE>
class cNodeStampedBasePa : public NODE {

  public:
    typedef NODE                     NodeTypeBase;
    typedef cNodeStampedBasePa<NODE> NodeTypeFull;

    cNodeStampedBasePa();
    cNodeStampedBasePa(const cNodeStampedBasePa<NODE>& other);
    cNodeStampedBasePa(const cTimePa &timestamp);
    virtual ~cNodeStampedBasePa();

    bool operator==(const cNodeStampedBasePa<NODE>& other) const;

    void copyData(const cNodeStampedBasePa<NODE>& from);

    // streaming (saving and loading)
    virtual std::istream& readData(std::istream &s);
    virtual std::ostream& writeData(std::ostream &s) const;

    // timestamp
    inline const cTimePa& getTimestamp() const;
    inline void setTimestamp(const cTimePa &timestamp);
    inline void updateTimestamp(const cTimePa &timestamp);

    // update occupancy and timesteps of inner nodes
    inline void updateTimestampChildren();
    inline void updateOccupancyChildren();

  protected:
    cTimePa timestamp;
};

#include "octomap_pa/node_stamped_base_pa.hxx"

#endif //#ifndef NODE_STAMPED_BASE_PA_H
