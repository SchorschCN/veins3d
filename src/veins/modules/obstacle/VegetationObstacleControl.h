//
// ObstacleControl - models obstacles that block radio transmissions
// Copyright (C) 2006 Christoph Sommer <christoph.sommer@informatik.uni-erlangen.de>
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

/*define class VegetationObstacleControl*/
#ifndef VEGETATION_OBSTACLE_OBSTACLECONTROL_H
#define VEGETATION_OBSTACLE_OBSTACLECONTROL_H

#include <list>

#include <omnetpp.h>
#include "veins/base/utils/Coord.h"
#include "veins/modules/obstacle/Obstacle.h"
#include "veins/modules/world/annotations/AnnotationManager.h"

/**
 * ObstacleControl models obstacles that block radio transmissions.
 *
 * Each Obstacle is a polygon.
 * Transmissions that cross one of the polygon's lines will have
 * their receive power set to zero.
 */
namespace Veins {
class VegetationObstacleControl : public cSimpleModule
{
	public:
		~VegetationObstacleControl();
		void initialize(int stage);
		int numInitStages() const { return 2; }
		void finish();
		void handleMessage(cMessage *msg);
		void handleSelfMsg(cMessage *msg);
        Coord lonLatToCart(double lon, double lat, double alt);
		void addFromXml(cXMLElement* xml);
		void addFromTypeAndShape(std::string id, std::string typeId, std::vector<Coord> shape);
		void add(Obstacle obstacle);
		void erase(const Obstacle* obstacle);
		bool isTypeSupported(std::string type);
		double getAttenuationPerCut(std::string type);
		double getAttenuationPerMeter(std::string type);


		/**
		 * calculate depth in obstacles, return signal strength
		 */
		double calcDepth(const Coord& senderPos, const Coord& receiverPos) const;

	protected:
		struct CacheKey {
			const Coord senderPos;
			const Coord receiverPos;

			CacheKey(const Coord& senderPos, const Coord& receiverPos) :
				senderPos(senderPos),
				receiverPos(receiverPos) {
			}

			bool operator<(const CacheKey& o) const {
				if (senderPos.x < o.senderPos.x) return true;
				if (senderPos.x > o.senderPos.x) return false;
				if (senderPos.y < o.senderPos.y) return true;
				if (senderPos.y > o.senderPos.y) return false;
				if (receiverPos.x < o.receiverPos.x) return true;
				if (receiverPos.x > o.receiverPos.x) return false;
				if (receiverPos.y < o.receiverPos.y) return true;
				if (receiverPos.y > o.receiverPos.y) return false;
				return false;
			}
		};

		enum { GRIDCELL_SIZE = 5000 };

		typedef std::list<Obstacle*> ObstacleGridCell;
		typedef std::vector<ObstacleGridCell> ObstacleGridRow;
		typedef std::vector<ObstacleGridRow> Obstacles;
		typedef std::map<CacheKey, double> CacheEntries;

		bool debug; /**< whether to emit debug messages */
		cXMLElement* obstaclesXml; /**< obstacles to add at startup */
        mutable double fractionInObstacle;

		Obstacles obstacles;
		AnnotationManager* annotations;
		AnnotationManager::Group* annotationGroup;
		std::map<std::string, double> perCut;
		std::map<std::string, double> perMeter;
		mutable CacheEntries cacheEntries;
};
}

namespace Veins {
    class VegetationObstacleControlAccess
    {
	    public:
		    VegetationObstacleControlAccess()
	        {
		    }

		    VegetationObstacleControl* getIfExists()
		    {
		        return dynamic_cast<VegetationObstacleControl*>(getSimulation()->getModuleByPath("vegetationObstacles"));
		    }
    };
}

#endif              /*end define class VegetationObstacleControl*/

