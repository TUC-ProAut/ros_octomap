/******************************************************************************
*                                                                             *
* time_pa.cpp                                                                 *
* ===========                                                                 *
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
#include "time_pa.h"

// standard headers
#include <cmath>


//**************************[cTimePa]******************************************
cTimePa::cTimePa (const int32_t seconds, const int32_t nanoseconds):
  seconds(seconds),nanoseconds(nanoseconds){

}

cTimePa::cTimePa (const double seconds) {

    this->seconds = floor(seconds);
    this->nanoseconds = round((seconds - floor(seconds)) * 1e9);

    fix();
}

cTimePa::cTimePa (const cTimePa &other): seconds(other.seconds),
  nanoseconds(other.nanoseconds){

}

const cTimePa& cTimePa::operator = (const cTimePa &other) {

    seconds     = other.seconds;
    nanoseconds = other.nanoseconds;
    return *this;
}

bool cTimePa::operator == (const cTimePa &other) const {

    return (seconds == other.seconds) && (nanoseconds == other.nanoseconds);
}

bool cTimePa::operator < (const cTimePa &other) const {

    return (seconds < other.seconds) ||
      ((seconds == other.seconds) && (nanoseconds < other.nanoseconds));
}

bool cTimePa::operator > (const cTimePa &other) const {

    return (seconds > other.seconds) ||
      ((seconds == other.seconds) && (nanoseconds > other.nanoseconds));
}

cTimePa cTimePa::operator - (const cTimePa &other) {

    cTimePa result(seconds, nanoseconds);
    result.seconds    -= other.seconds    ;
    result.nanoseconds-= other.nanoseconds;

    result.fix();
    return result;
}

cTimePa cTimePa::operator + (const cTimePa &other) {

    cTimePa result(seconds, nanoseconds);
    result.seconds    += other.seconds    ;
    result.nanoseconds+= other.nanoseconds;

    result.fix();
    return result;
}

void cTimePa::fix(void) {

    seconds     += nanoseconds / 1000000000;
    nanoseconds %= 1000000000;

    if (nanoseconds < 0) {
        seconds--;
        nanoseconds += 1000000000;
    }
}
