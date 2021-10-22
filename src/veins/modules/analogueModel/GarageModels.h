#ifndef GARAGEMODELS_H_
#define GARAGEMODELS_H_

#include <veins/base/phyLayer/AnalogueModel.h>
#include "veins/base/messages/AirFrame_m.h"
#include "veins/modules/floor/FloorControl.h"
#include "veins/modules/analogueModel/EnvironmentalDiffraction.h"
#include "veins/modules/obstacle/ObstacleControl.h"
#include "veins/modules/analogueModel/RiceRayleighFading.h"

using Veins::ObstacleControl;

/**
 * @brief Model to determine the attenuation in parking garages.
 * This model actually combines multi-level floor attenuation, diffraction by vehicles, shadowing caused by
 * walls and small-scale fading depending on the individual setting and positions of sender and receiver.
 *
 * A detailed explanation of this model in the context of 3D VANET simulation and example simulation results
 * can be found in:
 * Alexander Brummer, Thomas Deinlein and Anatoli Djanatliev, "On the Simulation of Vehicular Networking
 * Scenarios in Multi-Story Parking Garages," 13th IEEE Vehicular Networking Conference (VNC 2021),
 * Ulm, Germany (Virtual Event), November 2021
 *
 *
 * An example config.xml for this AnalogueModel can be the following, containing the parameters for
 * Rice/Rayleigh fading:
 * @verbatim
     <AnalogueModel type="GarageModels">
         <!-- The signal frequency -->
         <parameter name="carrierFrequency" type="double" value="5.890e+9"/>
         <!-- How many paths should be considered for Rice/Rayleigh fading (recommended: 8) -->
         <parameter name="numPaths" type="long" value="8"/>
         <!-- Rician K factor in linear units (NOT dB), for Rayleigh fading use 0 -->
         <parameter name="kFactor" type="double" value="0"/>
         <!-- Interval at which Rice/Rayleigh fading samples are generated -->
         <parameter name="interval" type="double" value="0.0004"/>
     </AnalogueModel>
 @endverbatim
 *
 * @author Alexander Brummer
 *
 * @ingroup analogueModels
 */
class GarageModels : public AnalogueModel
{
public:
    /**
     * @brief Initializes the analogue model.
     *
     * @param carrierFrequency the carrier frequency
     * @param numPaths number of paths to be considered for Rice/Rayleigh fading
     * @param kFactor Rician K factor
     * @param interval interval at which fading samples are generated
     */
    GarageModels(double carrierFrequency, int numPaths, double kFactor, simtime_t_cref interval);

    /**
     * @brief Initializes the analogue model with the given models/control objects.
     *
     * @param carrierFrequency the carrier frequency
     * @param floorCtrl the FloorControl object
     * @param obstCtrl the ObstacleControl object
     * @param diffModel the EnvironmentalDiffraction model
     * @param riceRayleighModel the RiceRayleighFading model
     */
    GarageModels(double carrierFrequency, FloorControl* floorCtrl, ObstacleControl* obstCtrl,
            EnvironmentalDiffraction* diffModel, RiceRayleighFading* riceRayleighModel) :
            carrierFrequency(carrierFrequency), floorCtrl(floorCtrl), obstCtrl(obstCtrl), diffModel(diffModel), riceRayleighModel(
                    riceRayleighModel)
    {
    }

    /**
     * @brief Destructor.
     * Delete the models/control objects.
     */
    virtual ~GarageModels()
    {
        delete diffModel;
        delete riceRayleighModel;
    }

    /**
     * @brief Filters a specified AirFrame's Signal by adding the correct attenuation.
     *
     * @param frame the Airframe in question
     * @param senderPos the sender's position
     * @param receiverPos the receiver's position
     */
    virtual void filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos);

protected:
    /** @brief Carrier frequency. */
    double carrierFrequency;

    /** @brief FloorControl object */
    FloorControl* floorCtrl;

    /** @brief ObstacleControl object */
    ObstacleControl* obstCtrl;

    /** @brief EnvironmentalDiffraction model object */
    EnvironmentalDiffraction* diffModel;

    /** @brief RiceRayleighFading model object */
    RiceRayleighFading* riceRayleighModel;
};

#endif /* GARAGEMODELS_H_ */
