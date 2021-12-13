#include "veins/modules/analogueModel/VegetationWeissbg.h"


VegetationWeissbg::VegetationWeissbg(VegetationObstacleControl& vegetationObsCtl, double carrierFrequency, const Coord& playgroundSize, bool debug) :
vegetationObstacleControl(vegetationObsCtl), Frequency(carrierFrequency), debug(debug)
{
	this->wavelength = BaseWorldUtility::speedOfLight() / carrierFrequency;
	BaseWorldUtility* world = FindModule<BaseWorldUtility*>::findGlobalModule();
	this->playgroundSize = world->getPgs();
//TODO:check
}

//TODO DELETE IT
#if 0
double VegetationWeissbg::calcDepth(const Coord& senderPos, const Coord& receiverPos) const
{
    double totalDistance = senderPos.distance(receiverPos);
    //modify this function call to solve coupling problem in obstacleControl
    obstacleControl.calculateAttenuation(senderPos, receiverPos);
    int fractionInObstacle = 0;
    return fractionInObstacle * totalDistance;
}
#endif

double VegetationWeissbg::calcAttenuation(const Coord& senderPos, const Coord& receiverPos) const
{
	double depthInVegetation = this->vegetationObstacleControl.calcDepth(senderPos,receiverPos);
	double L = 1;
	double fInGhz = Frequency/1000000000;
	if(depthInVegetation >= 0 && depthInVegetation <= 14)
	{
		L = 0.45*pow(fInGhz,0.284)*depthInVegetation;
        std::cout<<"weissbg model used, attenuation is "<<L<<std::endl;
	}
	else if(depthInVegetation > 14 && depthInVegetation <= 400)
	{
		L = 1.33*pow(fInGhz,0.284)*pow(depthInVegetation,0.588);
		std::cout<<"weissbg model used, attenuation is "<<L<<std::endl;
	}
	else
	{
		EV << "Attenuation by Vegetation using weissberger's model could not be calculated" << endl;
	}
	return FWMath::dBm2mW(-L);
}

void VegetationWeissbg::filterSignal(AirFrame* frame, const Coord& sendersPos, const Coord& receiverPos)
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

#if 0
void VegetationWeissbg::filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos)
{
	//TODO: MODIFY THIS CODE
	Signal& s = frame->getSignal();
	double factor = calcAttenuation(senderPos, receiverPos);
	EV << "Attenuation by Vegetation (using Weissberger Model) is: " << factor << endl;
	bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
	const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();
	ConstantSimpleConstMapping* attMapping = new ConstantSimpleConstMapping(domain, factor);
	s.addAttenuation(attMapping);
}


std::pair<double, double> VegetationWeissbg::isInLOS(const Coord& pos, const Coord& orient, const Coord& senderPos, const Coord& receiverPos)
{
	std::vector<Coord> shape;

}
#endif
