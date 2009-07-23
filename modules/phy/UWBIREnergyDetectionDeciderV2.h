/* -*- mode:c++ -*- ********************************************************
 * file:        UWBIREnergyDetectionDeciderV2.h
 *
 * author:      Jerome Rousselot <jerome.rousselot@csem.ch>
 *
 * copyright:   (C) 2008-2009 Centre Suisse d'Electronique et Microtechnique (CSEM) SA
 * 				Systems Engineering
 *              Real-Time Software and Networking
 *              Jaquet-Droz 1, CH-2002 Neuchatel, Switzerland.
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 * description: this Decider models an energy-detection receiver with soft-decision
 ***************************************************************************/

#ifndef _UWBIRENERGYDETECTIONDECIDERV2_H
#define	_UWBIRENERGYDETECTIONDECIDERV2_H

#include <vector>
#include <map>
#include <math.h>

#include "Signal_.h"
#include "Mapping.h"
#include "AirFrame_m.h"
#include "Decider.h"
#include "UWBIRDeciderResult.h"
#include "AlohaMacLayer.h"
#include "IEEE802154A.h"
#include "UWBIRPacket.h"
#include "BaseUtility.h"
//#include "UWBIRRadio.h"
//#include "PhyUtils.h"

using namespace std;

#define PI 3.1415926

class UWBIRPhyLayer;


class UWBIREnergyDetectionDeciderV2: public Decider {
private:
	bool trace, stats;
	long nbRandomBits;
	long nbFailedSyncs, nbSuccessfulSyncs;
	double nbSymbols, allThresholds;
	double vsignal2, vnoise2, snirs, snirEvals, pulseSnrs;
	double packetSNIR, packetSignal, packetNoise, packetSamples;

protected:
	double syncThreshold;
	bool syncAlwaysSucceeds;
	UWBIRPacket packet;
	int catUWBIRPacket;
	BaseUtility* utility;

public:
	// Boltzmann constant multiplied by 500 MHz (signal bandwidth) in mJ.K-1 !
	const static double kB500M = 5 * 1.38E-12; // mW/K
	const static int temperature = 293; // 20 Celsius degrees
	const static double noiseVariance = 404.34E-12;
	const static double Ptx = 37.06E-6; // radiated power at origin (-41.3 dBm/MHz over 500 MHz in Watts)
	const static double resistor = 50; // 50 Ohms
	const static double lambda = 0.04;// center frequency wavelength
	UWBIREnergyDetectionDeciderV2(DeciderToPhyInterface* iface,
			UWBIRPhyLayer* _uwbiface,
			double _syncThreshold, bool _syncAlwaysSucceeds, bool _stats,
			bool _trace) :
		Decider(iface), trace(_trace),
				stats(_stats), nbRandomBits(0), nbFailedSyncs(0),
				nbSuccessfulSyncs(0), nbSymbols(0), syncThreshold(_syncThreshold),
				syncAlwaysSucceeds(_syncAlwaysSucceeds), uwbiface(_uwbiface), tracking(0),
				channelSensing(false), synced(false), vsignal2(0), vnoise2(0), snirEvals(0), pulseSnrs(0) {

		zerosEnergies.setName("ZerosEnergies");
		onesEnergies.setName("OnesEnergies");
		signalLengths.setName("signalLengths");
		receivedPulses.setName("receivedPulses");
		syncThresholds.setName("syncThresholds");
		timeHoppings.setName("timeHoppings");
		ebN0.setName("EbN0");
		pulseSINR.setName("sinr");

		utility = iface->getUtility();
		catUWBIRPacket = utility->getCategory(&packet);
	};

	virtual simtime_t processSignal(AirFrame* frame);

	long getNbRandomBits() {
		return nbRandomBits;
	};

	double getAvgThreshold() {
		if (nbSymbols > 0)
			return allThresholds / nbSymbols;
		else
			return 0;
	};

	long getNbFailedSyncs() {
		return nbFailedSyncs;
	};

	long getNbSuccessfulSyncs() {
		return nbSuccessfulSyncs;
	};

	double getNoiseValue() {
		 return normal(0, noiseVariance);
	}

protected:
	map<Signal*, int> currentSignals;
	cOutVector zerosEnergies, onesEnergies, thresholds, signalLengths,
			receivedPulses;
	cOutVector syncThresholds, timeHoppings;
	cOutVector ebN0;
	cOutVector pulseSINR;

	UWBIRPhyLayer* uwbiface;
	Signal* tracking;
	enum {
		FIRST, HEADER_OVER, SIGNAL_OVER
	};

	bool channelSensing;
	bool synced;

	vector<ConstMapping*> receivingPowers;
	ConstMapping* signalPower; // = signal->getReceivingPower();
	// store relative offsets between signals starts
	vector<simtime_t> offsets;
	vector<AirFrame*> airFrameVector;
	// Create an iterator for each potentially colliding airframe
	vector<AirFrame*>::iterator airFrameIter;

	typedef ConcatConstMapping<std::multiplies<double> > MultipliedMapping;

	void decodePacket(Signal* signal, vector<bool> * receivedBits);
	simtime_t handleNewSignal(Signal* s);
	simtime_t handleHeaderOver(map<Signal*, int>::iterator& it);
	virtual bool attemptSync(Signal* signal);
	simtime_t
			handleSignalOver(map<Signal*, int>::iterator& it, AirFrame* frame);
	// first value is energy from signal, other value is total window energy
	pair<double, double> integrateWindow(int symbol, simtime_t now,
			simtime_t burst, Signal* signal);

	simtime_t handleChannelSenseRequest(ChannelSenseRequest* request);

};

#endif	/* _UWBIRENERGYDETECTIONDECIDERV2_H */

