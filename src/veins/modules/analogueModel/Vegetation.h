#ifndef VEGETATION_H_
#define VEGETATION_H_

#include "veins/base/phyLayer/AnalogueModel.h"
#include "veins/base/messages/AirFrame_m.h"
#include "veins/base/utils/Move.h"
#include "veins/modules/obstacle/VegetationObstacleControl.h"
#include "veins/modules/utility/NBHeightMapper.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/base/modules/BaseWorldUtility.h"
#include "veins/base/phyLayer/Signal_.h"
#include "veins/base/phyLayer/Mapping.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"



using Veins::AirFrame;

using Veins::TraCIScenarioManager;
using Veins::TraCIMobility;
using Veins::TraCICommandInterface;

//using Veins::VegetationObstacleControl;
#include <cstdlib>

#if 0
class Vegetation : public AnalogueModel {
	protected:

//		double wavelength;		//wavelength
//		double frequency;
//		const Coord* playgroundSize;	//playground size
//		double dsmCellSize;
//		static double* dsmCache;
//		static size_t cacheRows;
//		static size_t cacheCols;
//		std::pair<double, double> isInLOS(const Coord& pos, const Coord& orient, const Coord& senderPos, const Coord& receiverPos);
//		virtual double calcDepth(const Coord& senderPos, const Coord& receiverPos) const = 0;
//		bool leaf;
//		const bool Torus;
	public:
//		Vegetation ();
//		Vegetation (double carrierFrequency, std::vector<std::string> dsmFiles, bool withLeaf)=delete;
		virtual double calcAttenuation (const Coord& senderPos, const Coord& receiverPos) const = 0;
		virtual void filterSignal (AirFrame * frame, const Coord& sendersPos, const Coord& receiverPos) const = 0;
};

#endif

#endif 		/*VEGETATION_H_*/

