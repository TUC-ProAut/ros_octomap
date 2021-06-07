/******************************************************************************
*                                                                             *
* octree_stamped_base_pa.hxx                                                  *
* ==========================                                                  *
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

template <template <typename> class OCTREE, typename NODE>
  cOcTreeStampedBasePa<OCTREE, NODE>::cOcTreeStampedBasePa(double resolution)
  : TreeTypeBase(resolution) {

}

template <template <typename> class OCTREE, typename NODE>
  cOcTreeStampedBasePa<OCTREE, NODE>::~cOcTreeStampedBasePa() {

}

template <template <typename> class OCTREE, typename NODE>
  cOcTreeStampedBasePa<OCTREE, NODE>* cOcTreeStampedBasePa<OCTREE, NODE>::
  create() const {

    return new cOcTreeStampedBasePa(TreeTypeBase::resolution);
}

template <template <typename> class OCTREE, typename NODE>
  void cOcTreeStampedBasePa<OCTREE, NODE>::updateNodeLogOdds(
  NodeTypeFull* node, const float& update) const {

    TreeTypeBase::updateNodeLogOdds(node, update);
    node->updateTimestamp(current_timestamp);
}

template <template <typename> class OCTREE, typename NODE>
  void cOcTreeStampedBasePa<OCTREE, NODE>::degradeOutdatedNodes(
  const cTimePa timediff) {

    cTimePa timethreshold = current_timestamp - timediff;

    for( typename TreeTypeBase::leaf_iterator it = TreeTypeBase::begin_leafs(),
      end=TreeTypeBase::end_leafs(); it!= end; ++it) {
        if ( it->getTimestamp() < timethreshold) {
            TreeTypeBase::deleteNode(it.getKey());
        }
    }
}

template <template <typename> class OCTREE, typename NODE>
  inline void cOcTreeStampedBasePa<OCTREE, NODE>::setTimestamp(
  const cTimePa timestamp) {

    this->current_timestamp = timestamp;
}

template <template <typename> class OCTREE, typename NODE>
  inline const cTimePa& cOcTreeStampedBasePa<OCTREE, NODE>::getTimestamp(
  void) const {

    return current_timestamp;
}
