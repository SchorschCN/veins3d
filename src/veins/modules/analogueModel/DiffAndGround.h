#ifndef DIFFANDGROUND_H_
#define DIFFANDGROUND_H_

#include <veins/base/phyLayer/AnalogueModel.h>
#include "veins/base/messages/AirFrame_m.h"
#include "veins/modules/analogueModel/EnvironmentalDiffraction.h"
#include "veins/modules/analogueModel/NRayGroundInterference.h"

/**
 * @brief Combination of the EnvironmentalDiffraction model and the NRayGroundInterference model.
 * Which model is used depends on whether the LOS is obstructed or not.
 *
 *
 * An example config.xml for this AnalogueModel can be the following (includes the parameters of
 * both models):
 * @verbatim
    <AnalogueModel type="DiffAndGround">
    	<!-- File name of the DEM to query the terrain's elevation -->
		<parameter name="demFiles" type="string" value="MfrOfr25.tif"/>
		<!-- True if the DEM is in raster format (GeoTiff) -->
		<parameter name="isRasterType" type="bool" value="true"/>
		<!-- The signal frequency -->
		<parameter name="carrierFrequency" type="double" value="5.890e+9"/>
		<!-- Relative permittivity -->
		<parameter name="epsilonR" type="double" value="1.008"/>
		<!-- Spacing of queried height profile points for the n-ray ground interference model -->
		<parameter name="groundSpacing" type="double" value="7"/>
		 <!-- Whether terrain characteristics should be considered for the diffracton model -->
		<parameter name="considerDEM" type="bool" value="true"/>
		<!-- Spacing of queried height profile points for the diffraction model -->
		<parameter name="diffSpacing" type="double" value="1"/>
		<!-- Cell size for the DEM cache (0 means caching is disabled) -->
		<parameter name="demCellSize" type="double" value="0"/>
		<!-- States whether other vehicles should be considered -->
		<parameter name="considerVehicles" type="bool" value="false"/>
	</AnalogueModel>
   @endverbatim
 *
 * @author Alexander Brummer
 *
 * @ingroup analogueModels
 */
class DiffAndGround : public AnalogueModel
{
public:

	/**
	 * @brief Initializes the analogue model.
	 *
	 * @param demFiles the filename(s) of the DEM
	 * @param isRasterType whether the DEM is in raster format (GeoTiff)
	 * @param carrierFrequency the carrier frequency
	 * @param epsilonR relative permittivity
	 * @param groundSpacing spacing of queried height profile points for the n-ray ground interference model
	 * @param considerDEM states whether the terrain characteristics should be considered for the diffraction model
	 * @param diffSpacing spacing of queried height profile points for the diffraction model
	 * @param demCellSize cell size for the DEM cache
	 * @param considerVehicles states whether other vehicles should be considered
	 */
	DiffAndGround(std::vector<std::string> demFiles, bool isRasterType, double carrierFrequency, double epsilonR,
			double groundSpacing, bool considerDEM, double diffSpacing, double demCellSize, bool considerVehicles);

	/**
	 * @brief Destructor, deletes both model objects.
	 */
	virtual ~DiffAndGround()
	{
		delete diffModel;
		delete nrayModel;
	}
	;

	/**
	 * @brief Filters a specified AirFrame's Signal by adding the correct attenuation:
	 * if the LOS is obstructed, the diffraction model is applied, otherwise the n-ray model.
	 *
	 * @param frame the Airframe in question
	 * @param senderPos the sender's position
	 * @param receiverPos the receiver's position
	 */
	virtual void filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos);

protected:
	/** @brief Carrier frequency. */
	double carrierFrequency;

	/** @brief EnvironmentalDiffraction model object */
	EnvironmentalDiffraction* diffModel;

	/** @brief NRayGroundInterference model object */
	NRayGroundInterference* nrayModel;
};

#endif /* DIFFANDGROUND_H_ */
