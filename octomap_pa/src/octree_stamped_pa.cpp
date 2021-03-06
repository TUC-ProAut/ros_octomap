/******************************************************************************
*                                                                             *
* octree_stamped_pa.cpp                                                       *
* =====================                                                       *
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

// local headers
#include "octomap_pa/octree_stamped_pa.h"


//**************************[cOcTreeStampedPa]*********************************
cOcTreeStampedPa::cOcTreeStampedPa(double resolution)
  : TreeTypeBase(resolution) {

    StaticMemberInit.ensureLinking();
}


//**************************[~cOcTreeStampedPa]********************************
cOcTreeStampedPa::~cOcTreeStampedPa() {

}

//**************************[create]*******************************************
cOcTreeStampedPa* cOcTreeStampedPa::create() const {

    return new cOcTreeStampedPa(resolution);
}


//**************************[getTreeType]**************************************
std::string cOcTreeStampedPa::getTreeType() const {

    return "OctreeStampedPa";
}


/**
 * Static member object which ensures that this OcTree's prototype
 * ends up in the classIDMapping only once
 */

//**************************[StaticMemberInitializer]**************************
cOcTreeStampedPa::StaticMemberInitializer::StaticMemberInitializer() {

    cOcTreeStampedPa* tree = new cOcTreeStampedPa(0.1);
    AbstractOcTree::registerTreeType(tree);
}


//**************************[ensureLinking]************************************
void cOcTreeStampedPa::StaticMemberInitializer::ensureLinking() {

}

//**************************[static member]************************************
cOcTreeStampedPa::StaticMemberInitializer cOcTreeStampedPa::StaticMemberInit;
