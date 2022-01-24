//
// ObstacleControl - models obstacles that block radio transmissions
// Copyright (C) 2010 Christoph Sommer <christoph.sommer@informatik.uni-erlangen.de>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

#include <sstream>
#include <map>
#include <set>
#include <iostream>
#include "veins/modules/obstacle/VegetationObstacleControl.h"

using Veins::VegetationObstacleControl;

Define_Module(Veins::VegetationObstacleControl);

VegetationObstacleControl::~VegetationObstacleControl() {

}


/*TODO Debug this function*/
void VegetationObstacleControl::initialize(int stage) {
	if (stage == 1)	{
		debug = par("debug");

		obstacles.clear();
		cacheEntries.clear();
        this->fractionInObstacle = 0;

		annotations = AnnotationManagerAccess().getIfExists();
		if (annotations) annotationGroup = annotations->createGroup("obstacles");

		obstaclesXml = par("obstacles");
		addFromXml(obstaclesXml);

	}
}

void VegetationObstacleControl::finish() {
	for (Obstacles::iterator i = obstacles.begin(); i != obstacles.end(); ++i) {
		for (ObstacleGridRow::iterator j = i->begin(); j != i->end(); ++j) {
			while (j->begin() != j->end()) erase(*j->begin());
		}
	}
	obstacles.clear();
	this->fractionInObstacle = 0.0;
}

void VegetationObstacleControl::handleMessage(cMessage *msg) {
	if (msg->isSelfMessage()) {
		handleSelfMsg(msg);
		return;
	}
	error("VegetationObstacleControl doesn't handle messages from other modules");
}

void VegetationObstacleControl::handleSelfMsg(cMessage *msg) {
	error("VegetationObstacleControl doesn't handle self-messages");
}

void VegetationObstacleControl::addFromXml(cXMLElement* xml) {
	std::string rootTag = xml->getTagName();
	if (rootTag != "obstacles") {
		throw cRuntimeError("Obstacle definition root tag was \"%s\", but expected \"obstacles\"", rootTag.c_str());
	}

	cXMLElementList list = xml->getChildren();
	for (cXMLElementList::const_iterator i = list.begin(); i != list.end(); ++i)
	{
		cXMLElement* e = *i;

		std::string tag = e->getTagName();

		if (tag == "type")
		{
		    // <type id="landuse.forest" db-per-cut="0" db-per-meter="0.25" /> db-per-meter differences according to model
            // <type id="natural.wood" db-per-cut="0" db-per-meter="0.35" />
			ASSERT(e->getAttribute("id"));
			std::string id = e->getAttribute("id");
			ASSERT(e->getAttribute("db-per-cut"));
			std::string perCutParS = e->getAttribute("db-per-cut");
			double perCutPar = 0.0;
			ASSERT(e->getAttribute("db-per-meter"));
			std::string perMeterParS = e->getAttribute("db-per-meter");
			double perMeterPar = strtod(perMeterParS.c_str(), 0);

			perCut[id] = perCutPar;
			perMeter[id] = perMeterPar;

		}
		else if (tag == "poly")
		{
			// <poly id="building#0" type="building" color="#F00" shape="16,0 8,13.8564 -8,13.8564 -16,0 -8,-13.8564 8,-13.8564" />
			ASSERT(e->getAttribute("id"));
			std::string id = e->getAttribute("id");
			ASSERT(e->getAttribute("type"));
			std::string type = e->getAttribute("type");


			/*don't add polygons with types different from landuse.forest&natural wood*/
			if (type != "forest" && type != "wood") continue;

			ASSERT(e->getAttribute("color"));
			std::string color = e->getAttribute("color");
			ASSERT(e->getAttribute("shape"));
			std::string shape = e->getAttribute("shape");

			Obstacle obs(id, type, getAttenuationPerCut(type), getAttenuationPerMeter(type));
			std::vector<Coord> sh;
			cStringTokenizer st(shape.c_str());
			while (st.hasMoreTokens())
			{
				std::string xy = st.nextToken();
				std::vector<double> xya = cStringTokenizer(xy.c_str(), ",").asDoubleVector();
				ASSERT(xya.size() == 2);
//				sh.push_back(Coord(xya[0], xya[1]));
				sh.push_back(lonLatToCart(xya[0], xya[1], 0));
			}
			obs.setShape(sh);
			add(obs);
		}
		else
		{
			throw cRuntimeError("Found unknown tag in obstacle definition: \"%s\"", tag.c_str());
		}

	}

}


/* TODO WRITE COMMENT TO FOLLOWING VALUES, EARTH RADIUS SET AS MACRO OR CONST DOUBLE
 * QUESTION:
 * 1. EARTH RADIUS?
 * 2. 111136.0 OR 111320.0?
 *
 * */
Coord VegetationObstacleControl::lonLatToCart(double lon, double lat, double alt) {
    double cartX = (lon - 11.203823)*cos(fabs(49.625335/180*M_PI))*111320.0;
    double cartY = (lat - 49.625335) * 111136.0;
    cartY = 5000 - cartY;

    return Coord(cartX, cartY, alt);
}

void VegetationObstacleControl::addFromTypeAndShape(std::string id, std::string typeId, std::vector<Coord> shape) {
	if (!isTypeSupported(typeId)) {
		throw cRuntimeError("Unsupported obstacle type: \"%s\"", typeId.c_str());
	}
	Obstacle obs(id, typeId, getAttenuationPerCut(typeId), getAttenuationPerMeter(typeId));
	obs.setShape(shape);
	add(obs);
}

void VegetationObstacleControl::add(Obstacle obstacle) {
	Obstacle* o = new Obstacle(obstacle);

	size_t fromRow = std::max(0, int(o->getBboxP1().x / GRIDCELL_SIZE));
	size_t toRow = std::max(0, int(o->getBboxP2().x / GRIDCELL_SIZE));
	size_t fromCol = std::max(0, int(o->getBboxP1().y / GRIDCELL_SIZE));
	size_t toCol = std::max(0, int(o->getBboxP2().y / GRIDCELL_SIZE));
	for (size_t row = fromRow; row <= toRow; ++row) {
		for (size_t col = fromCol; col <= toCol; ++col) {
			if (obstacles.size() < col+1) obstacles.resize(col+1);
			if (obstacles[col].size() < row+1) obstacles[col].resize(row+1);
			(obstacles[col])[row].push_back(o);
		}
	}

	// visualize using AnnotationManager
	if (annotations) o->visualRepresentation = annotations->drawPolygon(o->getShape(), "green", annotationGroup);

	cacheEntries.clear();
}

void VegetationObstacleControl::erase(const Obstacle* obstacle) {
	for (Obstacles::iterator i = obstacles.begin(); i != obstacles.end(); ++i) {
		for (ObstacleGridRow::iterator j = i->begin(); j != i->end(); ++j) {
			for (ObstacleGridCell::iterator k = j->begin(); k != j->end(); ) {
				Obstacle* o = *k;
				if (o == obstacle) {
					k = j->erase(k);
				} else {
					++k;
				}
			}
		}
	}

	if (annotations && obstacle->visualRepresentation) annotations->erase(obstacle->visualRepresentation);
	delete obstacle;

	cacheEntries.clear();
}


//TODO: remove unused variables and functions
double VegetationObstacleControl::calcDepth(const Coord& senderPos, const Coord& receiverPos) const {
	Enter_Method_Silent();

	if ((perCut.size() == 0) || (perMeter.size() == 0)) {
		throw cRuntimeError("Unable to use Vegetation Attenuation: No obstacle types have been configured");
	}

	//TODO No obstacles have been added, not solved until 3.12
	if (obstacles.size() == 0) {
		throw cRuntimeError("Unable to use Vegetation Attenuation: No obstacles have been added");
	}

	// TODO: check if cache still useful in VegetationObstacleControl?

	// return cached result, if available

	CacheKey cacheKey(senderPos, receiverPos);
#if 0
	CacheEntries::const_iterator cacheEntryIter = cacheEntries.find(cacheKey);
	if (cacheEntryIter != cacheEntries.end()) return cacheEntryIter->second;
#endif
	// calculate bounding box of transmission
	Coord bboxP1 = Coord(std::min(senderPos.x, receiverPos.x), std::min(senderPos.y, receiverPos.y));
	Coord bboxP2 = Coord(std::max(senderPos.x, receiverPos.x), std::max(senderPos.y, receiverPos.y));

    /*TODO remove this debug info
	std::cout<<"sender and receiver MOVED TO:"<< std::endl;
	std::cout<<"                "<<bboxP2.x<<", "<<bboxP2.y<< std::endl;
	std::cout<<bboxP1.x<<", "<<bboxP1.y<<std::endl;
	*/

	size_t fromRow = std::max(0, int(bboxP1.x / GRIDCELL_SIZE));
	size_t toRow = std::max(0, int(bboxP2.x / GRIDCELL_SIZE));
	size_t fromCol = std::max(0, int(bboxP1.y / GRIDCELL_SIZE));
	size_t toCol = std::max(0, int(bboxP2.y / GRIDCELL_SIZE));

	std::set<Obstacle*> processedObstacles;
	double factor = 1;
	for (size_t col = fromCol; col <= toCol; ++col)
	{
        //TODO remove this debug info
//        std::cout<<"calcDepth runs, col= "<< col <<std::endl;
	    if (col >= obstacles.size()) break;
		for (size_t row = fromRow; row <= toRow; ++row)
		{
	        //TODO remove this debug info
//	        std::cout<<"calcDepth runs, row= "<< row <<std::endl;
			if (row >= obstacles[col].size()) break;
			const ObstacleGridCell& cell = (obstacles[col])[row];
			for (ObstacleGridCell::const_iterator k = cell.begin(); k != cell.end(); ++k)
			{

				Obstacle* o = *k;

				if (processedObstacles.find(o) != processedObstacles.end()) continue;
				processedObstacles.insert(o);
				/*
			    //TODO remove this debug info
			    std::cout<<"obstacle bbox is:"<< std::endl;
			    std::cout<<"                "<<o->getBboxP2().x<<", "<<o->getBboxP2().y<< std::endl;
			    std::cout<<o->getBboxP1().x<<", "<<o->getBboxP1().y<<std::endl;

			    std::cout<<"transmission bbox is:"<< std::endl;
			    std::cout<<"                "<<bboxP2.x<<", "<<bboxP2.y<< std::endl;
			    std::cout<<bboxP1.x<<", "<<bboxP1.y<<std::endl;
                */

				// bail if bounding boxes cannot overlap
				if (o->getBboxP2().x < bboxP1.x) continue;
				if (o->getBboxP1().x > bboxP2.x) continue;
				if (o->getBboxP2().y < bboxP1.y) continue;
				if (o->getBboxP1().y > bboxP2.y) continue;

		        //TODO remove this debug info
//		        std::cout<<"current obstacle may affect attenuation"<< std::endl;
				/*TODO remove this debug info
                std::cout<<"calculated obstacle bbox is:"<< std::endl;
                std::cout<<"                "<<o->getBboxP2().x<<", "<<o->getBboxP2().y<< std::endl;
                std::cout<<o->getBboxP1().x<<", "<<o->getBboxP1().y<<std::endl;
                */
				double factorOld = factor;

				factor *= o->calculateAttenuation(senderPos, receiverPos);

                //TODO remove this debug info
//                std::cout<<"fraction in obstacle is CALCULATED now "<< fractionInObstacle << std::endl;
				if(o->getFractionInObstacle()!=0)
				{
				    std::cout<<"vegetation obstructs signal"<<std::endl;
				    std::cout<<"current simulation time is: "<<simTime()<<std::endl;
				    this->fractionInObstacle += o->getFractionInObstacle();
				}
				/*
				else
				{
				    std::cout<<"fraction in obstacle: "<<o->getFractionInObstacle()<<std::endl;
				    std::cout<<"vegetation didnt obstruct signal"<<std::endl;
				    std::cout<<"current simulation time is: "<<simTime()<<std::endl;
				}
				*/
			    //TODO remove this debug info

//			    std::cout<<"sum of fraction in obstacle: "<< fractionInObstacle << ", fraction in current obstacle: "<< o->getFractionInObstacle() << std::endl;

				// draw a "hit!" bubble
//				if (annotations && (factor != factorOld)) annotations->drawBubble(o->getBboxP1(), "hit");

				// bail if attenuation is already extremely high
				if (factor < 1e-30) break;

			}
		}
	}

	// cache result
	if (cacheEntries.size() >= 1000) cacheEntries.clear();
	cacheEntries[cacheKey] = factor;
	double depth = senderPos.distance(receiverPos)*(this->fractionInObstacle);

	//TODO remove this debug info

//	std::cout<<"fraction in obstacle is "<< fractionInObstacle << ", total depth is "<< depth << std::endl;
	this->fractionInObstacle = 0;
	return depth;
}

/* AttenuationPerCut is not relevant in Vegetation Obstacles*/
double VegetationObstacleControl::getAttenuationPerCut(std::string type) {
	if (perCut.find(type) != perCut.end()) return perCut[type];
	else {
		error("Obstacle type %s unknown", type.c_str());
		return -1;
	}
}


double VegetationObstacleControl::getAttenuationPerMeter(std::string type) {
	if (perMeter.find(type) != perMeter.end()) return perMeter[type];
	else {
		error("Obstacle type %s unknown", type.c_str());
		return -1;
	}
}

bool VegetationObstacleControl::isTypeSupported(std::string type) {
	//the type of obstacle is supported if there are attenuation values for interior
	return perMeter.find(type) != perMeter.end();
}
