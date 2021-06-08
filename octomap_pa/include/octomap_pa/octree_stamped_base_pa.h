/******************************************************************************
*                                                                             *
* octree_stamped_base_pa.h                                                    *
* ========================                                                    *
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
* Copyright (c) 2015-2021 TU Chemnitz                                         *
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

#ifndef OCTREE_STAMPED_BASE_PA_H
#define OCTREE_STAMPED_BASE_PA_H

// local headers
#include "octomap_pa/node_stamped_base_pa.h"

// standard headers
#include <string>

//**************************[cOcTreeStampedBasePa]*****************************
template <template <typename> class OCTREE, typename NODE>
class cOcTreeStampedBasePa : public OCTREE< cNodeStampedBasePa<NODE> > {

  public:
    typedef cNodeStampedBasePa<NODE> NodeTypeFull;
    typedef NODE                     NodeTypeBase;
    typedef OCTREE< NodeTypeFull >   TreeTypeBase;

    /// Default constructor, sets resolution of leafs
    cOcTreeStampedBasePa(double resolution);

    // Default destructor
    virtual ~cOcTreeStampedBasePa(void);

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    cOcTreeStampedBasePa<OCTREE,NODE>* create() const;

    virtual void updateNodeLogOdds(NodeTypeFull* node, const float& update)
      const;

    void degradeOutdatedNodes(const cTimePa timediff);

    /// Used to set internal timestamp (the value will remain until next
    /// update). Therefore this function must be called before the insertion
    /// of the related measurement
    inline void setTimestamp(const cTimePa timestamp);
    inline const cTimePa& getTimestamp(void) const;

  protected:
    /// used to set new data (insertion of measurement) to actual time stamp
    cTimePa current_timestamp;

};


#include "octomap_pa/octree_stamped_base_pa.hxx"

#endif //#ifndef OCTREE_STAMPED_BASE_PA_H
