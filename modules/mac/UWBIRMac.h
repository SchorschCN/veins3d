/* -*- mode:c++ -*- ********************************************************
 * file:        UWBIRMac.h
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
 * description: All MAC designed for use with UWB-IR should derive from this
 * 				class. It provides the necessary functions to build UWBIR
 * 				packets and to receive them.
 ***************************************************************************/

#ifndef UWBIRMAC_H
#define UWBIRMAC_H

#include "BaseMacLayer.h"
#include "Decider.h"
#include "MacToPhyControlInfo.h"
#include "PhyToMacControlInfo.h"
#include "MacControlInfo.h"
#include "Signal_.h"
#include "MacToPhyInterface.h"
#include "UWBIRMacPkt_m.h"
#include "IEEE802154A.h"
#include "UWBIRRadio.h"
#include "UWBIRDeciderResult.h"
#include "Packet.h"
#include <vector>
#include <utility>

using namespace std;

class UWBIRMac : public BaseMacLayer {

public:

    virtual void initialize(int stage);

    virtual void finish();


protected:
    bool debug;
    bool stats;
    bool trace;
    bool rsDecoder;
    bool packetsAlwaysValid;
    double totalRxBits, errRxBits; // double and not long as we divide one by the other to get the BER
    MacToPhyInterface* phy;
    Packet packet;
    int catPacket;
    int macaddress, netaddress;

    cOutVector packetsBER;
    cOutVector dataLengths;
    cOutVector erroneousSymbols;
    cOutVector sentPulses;
    cOutVector receivedPulses;
    cOutVector meanPacketBER;
    cOutVector packetSuccessRate;
    cOutVector packetSuccessRateNoRS;
    cOutVector ber;
    cStdDev meanBER;

    long nbReceivedPacketsNoRS, nbReceivedPacketsRS;
    long nbSentPackets;
    long nbSymbolErrors;
    long nbHandledRxPackets;

    void prepareData(UWBIRMacPkt* packet);

    void handleLowerMsg(cPacket *msg);

    bool validatePacket(UWBIRMacPkt * mac);

    void initCounters();
public:
    int getmacaddress() { return macaddress; }
    int getnetaddress() { return netaddress; }
};

#endif // UWBIRMAC_H

