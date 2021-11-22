//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//


#ifndef TRACE_MOBILITY_H
#define TRACE_MOBILITY_H

#include "veins/base/modules/BaseMobility.h"


class MIXIM_API TraceMobility : public BaseMobility
{
  protected:
    std::map<simtime_t, Coord> trace;
    double pgsY;
    double lonOffset;
    double latOffset;
    double antennaPositionOffset;
    double antennaHeight;
    cOutVector xVec;
    cOutVector yVec;
    cOutVector zVec;

  public:
    virtual void initialize(int);
    Coord lonLatToCart (double, double, double);
    Coord cartToLonLat (const Coord&);

  protected:
    virtual void makeMove();

    virtual void handleSelfMsg( cMessage* );

    virtual void fixIfHostGetsOutside();

    Coord calculateAntennaPosition(const Coord&, const Coord&) const;

};

#endif

