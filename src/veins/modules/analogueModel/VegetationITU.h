#ifndef VEGETATION_ITU_H_
#define VEGETATION_ITU_H_

#include "veins/modules/analogueModel/Vegetation.h"
#include "veins/modules/obstacle/VegetationObstacleControl.h"
using Veins::AirFrame;
using Veins::VegetationObstacleControl;
using Veins::TraCIScenarioManager;

#include <cstdlib>

class VegetationITU : public AnalogueModel {
    protected:
        VegetationObstacleControl& vegetationObstacleControl;
        double wavelength;
        double Frequency;
        const Coord* playgroundSize;
        bool debug;

    public:
        VegetationITU(VegetationObstacleControl& vegetationObsCtl, double carrierFrequency, const Coord& playgroundSize, bool debug);
        double calcAttenuation(const Coord& senderPos, const Coord& receiverPos) const;
        void filterSignal(AirFrame * frame, const Coord& sendersPos, const Coord& receiverPos);
};
#endif 		/*VEGETATION_ITU_H_*/
