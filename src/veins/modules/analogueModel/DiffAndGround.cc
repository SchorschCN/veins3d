#include <veins/modules/analogueModel/DiffAndGround.h>
#include "veins/modules/utility/NBHeightMapper.h"
#include "veins/base/modules/BaseWorldUtility.h"

using Veins::AirFrame;

DiffAndGround::DiffAndGround(std::vector<std::string> demFiles, bool isRasterType, double carrierFrequency,
		double epsilonR, double groundSpacing, bool considerDEM, double diffSpacing, double demCellSize, bool considerVehicles) {
	this->carrierFrequency = carrierFrequency;

	const NBHeightMapper& hm = NBHeightMapper::get();
	if (!hm.ready()) NBHeightMapper::loadHM(demFiles, isRasterType);

	diffModel = new EnvironmentalDiffraction(carrierFrequency, considerDEM, demCellSize, diffSpacing, considerVehicles);
	nrayModel = new NRayGroundInterference(carrierFrequency, epsilonR, groundSpacing);
}

void DiffAndGround::filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos) {
	Signal& s = frame->getSignal();

	double factor = diffModel->calcAttenuation(frame, senderPos, receiverPos);
	if (!FWMath::close(factor, 1.0)) {
		// if diffraction model does not return 1.0, LOS is obstructed => use this attenuation plus simple path loss
		double dist = (receiverPos - senderPos).length();
		double freespace = BaseWorldUtility::speedOfLight() / (4 * M_PI * dist * carrierFrequency);
		freespace = freespace * freespace;
		factor = factor * freespace;
	}
	else {
		//otherwise apply n-ray model
		factor = nrayModel->calcAttenuation(senderPos, receiverPos);
	}

	EV << "Attenuation by NRayGroundInterference is: " << factor << endl;

	bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
	const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();
	ConstantSimpleConstMapping* attMapping = new ConstantSimpleConstMapping(domain, factor);
	s.addAttenuation(attMapping);
}

