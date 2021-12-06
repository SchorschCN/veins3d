#ifndef VEGETATION_WEISSBG_H_
#define VEGETATION_WEISSBG_H_

#include "veins/modules/analogueModel/Vegetation.h"
#include "veins/modules/obstacle/VegetationObstacleControl.h"
using Veins::AirFrame;
using Veins::VegetationObstacleControl;
using Veins::TraCIScenarioManager;


#include <cstdlib>

class VegetationWeissbg : public AnalogueModel {
	protected:
        VegetationObstacleControl& vegetationObstacleControl;
		double wavelength;		//wavelength
		double Frequency;
		const Coord* playgroundSize;	//playground size
		bool debug;
//		double calcDepth(const Coord& senderPos, const Coord& receiverPos) const;

	public:
		VegetationWeissbg(VegetationObstacleControl& vegetationObstacleControl, double carrierFrequency, const Coord& playgroundSize, bool debug);
		double calcAttenuation(const Coord& senderPos, const Coord& receiverPos) const;
		void filterSignal(AirFrame * frame, const Coord& sendersPos, const Coord& receiverPos);
};
#endif 		/*VEGETATION_WEISSBG_H_*/
