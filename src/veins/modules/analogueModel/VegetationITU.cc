#include "veins/modules/analogueModel/VegetationITU.h"


VegetationITU::VegetationITU(VegetationObstacleControl& vegetationObsCtl, double carrierFrequency, const Coord& playgroundSize, bool debug) :
vegetationObstacleControl(vegetationObsCtl), Frequency(carrierFrequency), debug(debug)
{
    this->wavelength = BaseWorldUtility::speedOfLight() / carrierFrequency;
    BaseWorldUtility* world = FindModule<BaseWorldUtility*>::findGlobalModule();
    this->playgroundSize = world->getPgs();
//TODO:check
}

double VegetationITU::calcAttenuation(const Coord& senderPos, const Coord& receiverPos) const
{
    double depthInVegetation = this->vegetationObstacleControl.calcDepth(senderPos,receiverPos);
    double L = 1;
    double fInMhz = Frequency/1000000;
    if( depthInVegetation < 400 && fInMhz>=200 && fInMhz<=95000 )
    {
        L = 0.2*pow(fInMhz,0.3)*pow(depthInVegetation,0.6);
    }
    else
    {
        EV << "Attenuation by Vegetation using ITU model could not be calculated" << endl;
    }
    return FWMath::dBm2mW(-L);
}

void VegetationITU::filterSignal(AirFrame* frame, const Coord& sendersPos, const Coord& receiverPos)
{
    //TODO: MODIFY THIS CODE
    Signal& s = frame->getSignal();
    double factor = calcAttenuation(sendersPos, receiverPos);
    EV << "Attenuation by Vegetation is: " << factor << endl;
    bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
    const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();
    ConstantSimpleConstMapping* attMapping = new ConstantSimpleConstMapping(domain, factor);
    s.addAttenuation(attMapping);
}
