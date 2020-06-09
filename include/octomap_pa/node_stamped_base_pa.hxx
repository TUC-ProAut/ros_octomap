/******************************************************************************
*                                                                             *
* node_stamped_base_pa.hxx                                                    *
* ========================                                                    *
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
* Copyright (c) 2015-2020, Peter Weissig, Technische Universität Chemnitz     *
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

// additional libraries
#include <octomap/octomap.h>

//**************************[cNodeStampedBasePa]*******************************
template <typename NODE>
  cNodeStampedBasePa<NODE>::cNodeStampedBasePa() : NODE() {

}

//**************************[cNodeStampedBasePa]*******************************
template <typename NODE>
  cNodeStampedBasePa<NODE>::cNodeStampedBasePa(
  const cNodeStampedBasePa<NODE>& other) : NODE() {

    copyData(other);
    if (other.children != NULL){
        NODE::allocChildren();
        for (unsigned i = 0; i<8; ++i) {
            if (other.children[i] != NULL) {
                NODE::children[i] = new cNodeStampedBasePa<NODE>(*(
                  static_cast<cNodeStampedBasePa<NODE>*>(other.children[i])));
            }
        }
    }
}

//**************************[cNodeStampedBasePa]*******************************
template <typename NODE>
  cNodeStampedBasePa<NODE>::cNodeStampedBasePa(const cTimePa &timestamp) :
  NODE(), timestamp(timestamp) {

}

//**************************[~cNodeStampedBasePa]******************************
template <typename NODE>
  cNodeStampedBasePa<NODE>::~cNodeStampedBasePa() {

}

//**************************[operator ==]**************************************
template <typename NODE>
  bool cNodeStampedBasePa<NODE>::operator==(
  const cNodeStampedBasePa<NODE>& other) const {

    return ((static_cast<NODE>(other) == static_cast<NODE> (*this)) &&
      (other.timestamp == timestamp));
}

//**************************[copyData]*****************************************
template <typename NODE>
  void cNodeStampedBasePa<NODE>::copyData(
  const cNodeStampedBasePa<NODE>& from){

    NODE::copyData(from);
    timestamp = from.getTimestamp();
}


//**************************[writeData]****************************************
template <typename NODE>
  std::ostream& cNodeStampedBasePa<NODE>::writeData(std::ostream &s) const {

    // write node data
    s.write((const char*) &this->value, sizeof(this->value)); // occupancy
    s.write((const char*) &timestamp  , sizeof(timestamp  )); // timestamp

    return s;
}

//**************************[readData]*****************************************
template <typename NODE>
  std::istream& cNodeStampedBasePa<NODE>::readData (std::istream &s) {

    // read node data
    s.read((char*) &this->value, sizeof(this->value)); // occupancy
    s.read((char*) &timestamp  , sizeof(timestamp  )); // timestamp

    return s;
}

//**************************[getTimestamp]*************************************
template <typename NODE>
  inline const cTimePa& cNodeStampedBasePa<NODE>::getTimestamp() const {

    return timestamp;
}

//**************************[setTimestamp]*************************************
template <typename NODE>
  inline void cNodeStampedBasePa<NODE>::setTimestamp(
  const cTimePa &timestamp) {

    this->timestamp = timestamp;
}

//**************************[updateTimestamp]**********************************
template <typename NODE>
  inline void cNodeStampedBasePa<NODE>::updateTimestamp(
  const cTimePa &timestamp) {

    if (this->timestamp < timestamp) {setTimestamp(timestamp);}
}

//**************************[updateTimestampChildren]**************************
template <typename NODE>
  inline void cNodeStampedBasePa<NODE>::updateTimestampChildren() {

    if (NODE::children == NULL) { return;}
    for (unsigned int i=0; i<8; i++) {
        if (NODE::children[i] == NULL) { continue;}
        cNodeStampedBasePa *pnode = static_cast<cNodeStampedBasePa*>
          (NODE::children[i]);

        updateTimestamp(pnode->timestamp);
    }
}

//**************************[updateOccupancyChildren]**************************
template <typename NODE>
  inline void cNodeStampedBasePa<NODE>::updateOccupancyChildren() {

    //setLogOdds(getMaxChildLogOdds());  // conservative
    NODE::updateOccupancyChildren();
    updateTimestampChildren();
}
