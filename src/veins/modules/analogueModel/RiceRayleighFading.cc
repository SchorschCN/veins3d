#include <veins/modules/analogueModel/RiceRayleighFading.h>
#include "veins/base/messages/AirFrame_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"

using Veins::ChannelAccess;

RiceRayleighFading::RiceRayleighFading(double carrierFrequency, int numPaths, double kFactor, simtime_t_cref interval) :
        carrierFrequency(carrierFrequency), numPaths(numPaths), kFactor(kFactor), interval(interval)
{
    double theta = RNGCONTEXT uniform(-M_PI, M_PI);
    for (int i = 0; i < numPaths; ++i) {
        aoa.push_back((2 * M_PI * i - M_PI + theta) / (4 * numPaths));
        randPhaseReal.push_back(RNGCONTEXT uniform(-M_PI, M_PI));
        randPhaseImag.push_back(RNGCONTEXT uniform(-M_PI, M_PI));
    }
}

void RiceRayleighFading::filterSignal(AirFrame* frame, const Coord& senderPos, const Coord& receiverPos)
{
    Signal& signal = frame->getSignal();
    const double relSpeed = getRelSpeed(frame);

    signal.addAttenuation(
            new RiceRayleighFading::Mapping(*this, relSpeed, kFactor, Argument(signal.getReceptionStart()), interval,
                    Argument(signal.getReceptionEnd())));
}

void RiceRayleighFading::filterSignal(AirFrame* frame, const Coord& senderPos, const Coord& receiverPos,
        double curKFactor)
{
    Signal& signal = frame->getSignal();
    const double relSpeed = getRelSpeed(frame);

    signal.addAttenuation(
            new RiceRayleighFading::Mapping(*this, relSpeed, curKFactor, Argument(signal.getReceptionStart()), interval,
                    Argument(signal.getReceptionEnd())));
}

inline const double RiceRayleighFading::getRelSpeed(AirFrame* frame) const
{
    ChannelMobilityPtrType senderMob =
            dynamic_cast<ChannelAccess* const >(frame->getSenderModule())->getMobilityModule();
    ChannelMobilityPtrType receiverMob =
            dynamic_cast<ChannelAccess* const >(frame->getArrivalModule())->getMobilityModule();

    return (senderMob->getCurrentSpeed() - receiverMob->getCurrentSpeed()).length();
}

double RiceRayleighFading::Mapping::getValue(const Argument& pos) const
{
    double t = pos.getTime().dbl();
    double f_dopp = relSpeed * model.carrierFrequency / BaseWorldUtility::speedOfLight();

    double real = 0, imag = 0;

    for (int i = 0; i < model.numPaths; ++i) {
        double realInc = cos(2 * M_PI * f_dopp * t * cos(model.aoa[i]) + model.randPhaseReal[i]);
        double imagInc = cos(2 * M_PI * f_dopp * t * sin(model.aoa[i]) + model.randPhaseImag[i]);

        real += realInc;
        imag += imagInc;
    }

    real *= sqrt(2.0 / model.numPaths);
    imag *= sqrt(2.0 / model.numPaths);

    double att = ((real + sqrt(2 * kFactor)) * (real + sqrt(2 * kFactor)) + imag * imag) / (2 * (kFactor + 1));

    return att;
}
