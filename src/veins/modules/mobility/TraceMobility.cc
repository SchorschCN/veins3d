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

#include "TraceMobility.h"
#include "veins/modules/utility/NBHeightMapper.h"

#include <omnetpp.h>

#include <fstream>

Define_Module(TraceMobility);

void TraceMobility::initialize(int stage)
{
	BaseMobility::initialize(stage);

	debugEV << "initializing TraceMobility stage " << stage << endl;

	if (stage == 0){
	    xVec.setName("x");
	    yVec.setName("y");
	    zVec.setName("z");

	    pgsY = world->getPgs()->y;

	    lonOffset = par("lonOffset").doubleValue();
	    latOffset = par("latOffset").doubleValue();

	    antennaPositionOffset = par("antennaPositionOffset").doubleValue();
	    antennaHeight = par("antennaHeight").doubleValue();

	    const NBHeightMapper& hm = NBHeightMapper::get();
	    std::vector<std::string> demFiles {par("demFile").stdstringValue()};
	    if (!hm.ready()) NBHeightMapper::loadHM(demFiles, par("isRasterType").boolValue());

	    std::ifstream traceFile(par("traceFile").stringValue());
	    std::string line;
	    while (getline(traceFile, line)) {
	        cStringTokenizer tokenizer(line.c_str(), ",");
	        double t = atof(tokenizer.nextToken());
	        double y = atof(tokenizer.nextToken());
            double x = atof(tokenizer.nextToken());
            double z = hm.getZ(Position(x, y));

            // necessary due to missing values in the DEM
//            if ((t > 1513.5 && t < 1515.6) || (t > 1518.9 && t < 1520.6)) {
            if (z < 350.0) {
                continue;
            }

            // add antenna height
            if (this->getFullPath() == "TraceScenario.nodes[0].mobility")
                z += 1.525;
            else
                z += 1.482;

	        trace[t] = lonLatToCart(x, y, z);
	    }
	}
}


void TraceMobility::fixIfHostGetsOutside()
{
	//This shouldn't happen. Be sure that the playground is large enough.
}

void TraceMobility::handleSelfMsg( cMessage* msg) {
    makeMove();
    updatePosition();

    scheduleAt(simTime() + updateInterval, msg);
}

void TraceMobility::makeMove() {
	debugEV << "start makeMove " << move.info() << endl;

	Coord lastPos = move.getCurrentPosition();

	simtime_t now = simTime();

	simtime_t prevTime, nextTime;
	Coord prevPos, nextPos;
	std::map<simtime_t, Coord>::iterator it;
	for (it = trace.begin(); it != trace.end(); it++ )
	{
	    if (it == std::prev(trace.end(), 1)) {
	        endSimulation();
	    }
	    if (it->first > now) {
	        nextTime = it->first;
	        nextPos = it->second;
	        prevTime = std::prev(it, 1)->first;
	        prevPos = std::prev(it, 1)->second;
	        break;
	    }
	}

	Coord realPos = prevPos + (nextPos - prevPos)*(now.dbl() - prevTime.dbl())/(nextTime.dbl() - prevTime.dbl());

	Coord antennaPos = calculateAntennaPosition(realPos, move.getOrientation());

	move.setStart(realPos, simTime());
	xVec.record(realPos.x);
	yVec.record(realPos.y);
	zVec.record(realPos.z);

//	Coord dir = realPos - lastPos;
	Coord dir = nextPos - prevPos;
	if (dir != Coord::ZERO) {
	    dir = dir/dir.length();
	    move.setDirectionByVector(dir);
	}

	fixIfHostGetsOutside();
}

Coord TraceMobility::lonLatToCart(double lon, double lat, double alt) {
    double cartX = (lon - lonOffset)*cos(fabs(latOffset/180*M_PI))*111320.0;
    double cartY = (lat - latOffset) * 111136.0;
    cartY = pgsY - cartY;

    return Coord(cartX, cartY, alt);
}

Coord TraceMobility::cartToLonLat(const Coord& cart) {
    double lon = cart.x/cos(fabs(latOffset/180*M_PI))/111320.0 + lonOffset;
    double lat = (pgsY - cart.y)/111136.0 + latOffset;

    return Coord(lon, lat, cart.z);
}

Coord TraceMobility::calculateAntennaPosition(const Coord& vehiclePos, const Coord& orient) const {
    Coord corPos;

    double angle = atan2(orient.y, orient.x);

    double elev_angle = 0;
    if (orient.z >= 0.001 || orient.z <= -0.001) {
        elev_angle = asin(orient.z/orient.length());
    }

    if (antennaPositionOffset >= 0.001 || antennaHeight >= 0.001) {
        double apparentOffset = antennaPositionOffset*cos(elev_angle) + antennaHeight*sin(elev_angle);
        //calculate antenna position of vehicle according to antenna offset and antenna height
        corPos = Coord(vehiclePos.x - apparentOffset*cos(angle), vehiclePos.y - apparentOffset*sin(angle),
                vehiclePos.z - antennaPositionOffset*sin(elev_angle) + antennaHeight*cos(elev_angle));
    } else {
        corPos = Coord(vehiclePos.x, vehiclePos.y, vehiclePos.z);
    }
    return corPos;
}
