#include "veins/modules/analogueModel/VegetationFITUR.h"



VegetationFITUR::VegetationFITUR(VegetationObstacleControl& vegetationObsCtl, double carrierFrequency, const Coord& playgroundSize, bool leaf, bool debug) :
vegetationObstacleControl(vegetationObsCtl), Frequency(carrierFrequency), playgroundSize(playgroundSize),leaf(leaf), debug(debug)
{
    this->wavelength = BaseWorldUtility::speedOfLight() / carrierFrequency;
    BaseWorldUtility* world = FindModule<BaseWorldUtility*>::findGlobalModule();
//    this->playgroundSize = world->getPgs();
//TODO:check
}
#if 0
double VegetationCOST::calcDepth(const Coord& senderPos, const Coord& receiverPos) const
{
    double totalDistance = senderPos.distance(receiverPos);
    //modify this function call to solve coupling problem in obstacleControl
    obstacleControl.calculateAttenuation(senderPos, receiverPos);
    int fractionInObstacle = 0;
    return fractionInObstacle * totalDistance;
}
#endif

double VegetationFITUR::calcAttenuation(const Coord& senderPos, const Coord& receiverPos) const
{
    double depthInVegetation = this->vegetationObstacleControl.calcDepth(senderPos,receiverPos);
    double L = 1;
    double fInMhz = Frequency/1000000;
    if(fInMhz>=9600 && fInMhz<=57600)
    {
        if(leaf==false)
        {
            L = 0.37*pow(fInMhz,0.18)*pow(depthInVegetation,0.59);
        }
        else
        {
            L = 0.39*pow(fInMhz,0.39)*pow(depthInVegetation,0.25);
        }
    }
    else
    {
        EV << "Attenuation by Vegetation using weissberger's model could not be calculated" << endl;
    }
    return FWMath::dBm2mW(-L);
}

void VegetationFITUR::filterSignal(AirFrame* frame, const Coord& sendersPos, const Coord& receiverPos)
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
