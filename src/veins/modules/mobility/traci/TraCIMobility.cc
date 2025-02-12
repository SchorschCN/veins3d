//
// Copyright (C) 2006-2012 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include <limits>
#include <iostream>
#include <sstream>

#include "veins/modules/mobility/traci/TraCIMobility.h"

using Veins::TraCIMobility;

Define_Module(Veins::TraCIMobility);

const simsignalwrap_t TraCIMobility::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

namespace {
const double MY_INFINITY = (std::numeric_limits<double>::has_infinity ? std::numeric_limits<double>::infinity() : std::numeric_limits<double>::max());
}

void TraCIMobility::Statistics::initialize()
{
    firstRoadNumber = MY_INFINITY;
    startTime = simTime();
    totalTime = 0;
    stopTime = 0;
    minSpeed = MY_INFINITY;
    maxSpeed = -MY_INFINITY;
    totalDistance = 0;
    totalCO2Emission = 0;
}

void TraCIMobility::Statistics::watch(cSimpleModule&)
{
    WATCH(totalTime);
    WATCH(minSpeed);
    WATCH(maxSpeed);
    WATCH(totalDistance);
}

void TraCIMobility::Statistics::recordScalars(cSimpleModule& module)
{
    if (firstRoadNumber != MY_INFINITY) module.recordScalar("firstRoadNumber", firstRoadNumber);
    module.recordScalar("startTime", startTime);
    module.recordScalar("totalTime", totalTime);
    module.recordScalar("stopTime", stopTime);
    if (minSpeed != MY_INFINITY) module.recordScalar("minSpeed", minSpeed);
    if (maxSpeed != -MY_INFINITY) module.recordScalar("maxSpeed", maxSpeed);
    module.recordScalar("totalDistance", totalDistance);
    module.recordScalar("totalCO2Emission", totalCO2Emission);
}

void TraCIMobility::initialize(int stage)
{
    if (stage == 0) {
        BaseMobility::initialize(stage);

        debug = par("debug");
        antennaPositionOffset = par("antennaPositionOffset");
        antennaHeight = par("antennaHeight");
        accidentCount = par("accidentCount");

        currentPosXVec.setName("posx");
        currentPosYVec.setName("posy");
        currentPosZVec.setName("posz");
        currentSpeedVec.setName("speed");
        currentAccelerationVec.setName("acceleration");
        currentCO2EmissionVec.setName("co2emission");

        statistics.initialize();
        statistics.watch(*this);

        ASSERT(isPreInitialized);
        isPreInitialized = false;

        Coord nextPos = calculateAntennaPosition(roadPosition);
        //nextPos.z = move.getCurrentPosition().z;

        move.setStart(nextPos);
        move.setDirectionByVector(Coord(cos(elev_angle) * cos(angle), -cos(elev_angle) * sin(angle), sin(elev_angle)));
        move.setSpeed(speed);

        WATCH(road_id);
        WATCH(speed);
        WATCH(angle);
        WATCH(elev_angle);

        isParking = false;

        startAccidentMsg = 0;
        stopAccidentMsg = 0;
        manager = 0;
        last_speed = -1;

        if (accidentCount > 0) {
            simtime_t accidentStart = par("accidentStart");
            startAccidentMsg = new cMessage("scheduledAccident");
            stopAccidentMsg = new cMessage("scheduledAccidentResolved");
            scheduleAt(simTime() + accidentStart, startAccidentMsg);
        }
    }
    else if (stage == 1) {
        // don't call BaseMobility::initialize(stage) -- our parent will take care to call changePosition later
    }
    else {
        BaseMobility::initialize(stage);
    }
}

void TraCIMobility::finish()
{
    statistics.stopTime = simTime();

    statistics.recordScalars(*this);

    cancelAndDelete(startAccidentMsg);
    cancelAndDelete(stopAccidentMsg);

    isPreInitialized = false;
}

void TraCIMobility::handleSelfMsg(cMessage* msg)
{
    if (msg == startAccidentMsg) {
        getVehicleCommandInterface()->setSpeed(0);
        simtime_t accidentDuration = par("accidentDuration");
        scheduleAt(simTime() + accidentDuration, stopAccidentMsg);
        accidentCount--;
    }
    else if (msg == stopAccidentMsg) {
        getVehicleCommandInterface()->setSpeed(-1);
        if (accidentCount > 0) {
            simtime_t accidentInterval = par("accidentInterval");
            scheduleAt(simTime() + accidentInterval, startAccidentMsg);
        }
    }
}

void TraCIMobility::preInitialize(std::string external_id, const Coord& position, std::string road_id, double speed,
        double angle, double elev_angle)
{
    this->external_id = external_id;
    this->lastUpdate = 0;
    this->roadPosition = position;
    this->road_id = road_id;
    this->speed = speed;
    this->angle = angle;
    this->elev_angle = elev_angle;
    this->antennaPositionOffset = par("antennaPositionOffset");

    Coord nextPos = calculateAntennaPosition(roadPosition);
    //nextPos.z = move.getCurrentPosition().z;

    move.setStart(nextPos);
    move.setDirectionByVector(Coord(cos(elev_angle) * cos(angle), -cos(elev_angle) * sin(angle), sin(elev_angle)));
    move.setSpeed(speed);

    isPreInitialized = true;
}

void TraCIMobility::nextPosition(const Coord& position, std::string road_id, double speed, double angle,
        double elev_angle, TraCIScenarioManager::VehicleSignal signals)
{
    if (debug)
        EV << "nextPosition " << position.x << " " << position.y << " " << position.z << " " << road_id << " " << speed
                << " " << angle << " " << elev_angle << std::endl;
    isPreInitialized = false;
    this->roadPosition = position;
    this->road_id = road_id;
    this->speed = speed;
    this->angle = angle;
    this->elev_angle = elev_angle;
    this->signals = signals;

    changePosition();
}

void TraCIMobility::changePosition()
{
    // ensure we're not called twice in one time step
    ASSERT(lastUpdate != simTime());

    // keep statistics (for current step)
    currentPosXVec.record(move.getStartPos().x);
    currentPosYVec.record(move.getStartPos().y);
    currentPosZVec.record(move.getStartPos().z);

    Coord nextPos = calculateAntennaPosition(roadPosition);
    //nextPos.z = move.getCurrentPosition().z;

    // keep statistics (relative to last step)
    if (statistics.startTime != simTime()) {
        simtime_t updateInterval = simTime() - this->lastUpdate;

        double distance = move.getStartPos().distance(nextPos);
        statistics.totalDistance += distance;
        statistics.totalTime += updateInterval;
        if (speed != -1) {
            statistics.minSpeed = std::min(statistics.minSpeed, speed);
            statistics.maxSpeed = std::max(statistics.maxSpeed, speed);
            currentSpeedVec.record(speed);
            if (last_speed != -1) {
                double acceleration = (speed - last_speed) / updateInterval;
                double co2emission = calculateCO2emission(speed, acceleration);
                currentAccelerationVec.record(acceleration);
                currentCO2EmissionVec.record(co2emission);
                statistics.totalCO2Emission += co2emission * updateInterval.dbl();
            }
            last_speed = speed;
        }
        else {
            last_speed = -1;
            speed = -1;
        }
    }
    this->lastUpdate = simTime();

    move.setStart(Coord(nextPos.x, nextPos.y, nextPos.z));
    move.setDirectionByVector(Coord(cos(elev_angle) * cos(angle), -cos(elev_angle) * sin(angle), sin(elev_angle)));
    move.setSpeed(speed);
    fixIfHostGetsOutside();
    updatePosition();
}

void TraCIMobility::changeParkingState(bool newState)
{
    Enter_Method_Silent();
    isParking = newState;
    emit(parkingStateChangedSignal, this);
}

void TraCIMobility::fixIfHostGetsOutside()
{
    Coord pos = move.getStartPos();
    Coord dummy = Coord::ZERO;
    double dum;

    bool outsideX = (pos.x < 0) || (pos.x >= playgroundSizeX());
    bool outsideY = (pos.y < 0) || (pos.y >= playgroundSizeY());
    bool outsideZ = (!world->use2D()) && ((pos.z < 0) || (pos.z >= playgroundSizeZ()));
    if (outsideX || outsideY || outsideZ) {
        throw cRuntimeError("Tried moving host to (%f, %f) which is outside the playground", pos.x, pos.y);
    }

    handleIfOutside(RAISEERROR, pos, dummy, dummy, dum);
}

double TraCIMobility::calculateCO2emission(double v, double a) const
{
    // Calculate CO2 emission parameters according to:
    // Cappiello, A. and Chabini, I. and Nam, E.K. and Lue, A. and Abou Zeid, M., "A statistical model of vehicle emissions and fuel consumption," IEEE 5th International Conference on Intelligent Transportation Systems (IEEE ITSC), pp. 801-809, 2002

    double A = 1000 * 0.1326; // W/m/s
    double B = 1000 * 2.7384e-03; // W/(m/s)^2
    double C = 1000 * 1.0843e-03; // W/(m/s)^3
    double M = 1325.0; // kg

    // power in W
    double P_tract = A * v + B * v * v + C * v * v * v + M * a * v; // for sloped roads: +M*g*sin_theta*v

    /*
       // "Category 7 vehicle" (e.g. a '92 Suzuki Swift)
       double alpha = 1.01;
       double beta = 0.0162;
       double delta = 1.90e-06;
       double zeta = 0.252;
       double alpha1 = 0.985;
     */

    // "Category 9 vehicle" (e.g. a '94 Dodge Spirit)
    double alpha = 1.11;
    double beta = 0.0134;
    double delta = 1.98e-06;
    double zeta = 0.241;
    double alpha1 = 0.973;

    if (P_tract <= 0) return alpha1;
    return alpha + beta * v * 3.6 + delta * v * v * v * (3.6 * 3.6 * 3.6) + zeta * a * v;
}

Coord TraCIMobility::calculateAntennaPosition(const Coord& vehiclePos) const
{
    Coord corPos;
    if (antennaPositionOffset >= 0.001 || antennaHeight >= 0.001) {
        double apparentOffset = antennaPositionOffset * cos(elev_angle) + antennaHeight * sin(elev_angle);
        //calculate antenna position of vehicle according to antenna offset and antenna height
        corPos = Coord(vehiclePos.x - apparentOffset * cos(angle), vehiclePos.y + apparentOffset * sin(angle),
                vehiclePos.z - antennaPositionOffset * sin(elev_angle) + antennaHeight * cos(elev_angle));
    }
    else {
        corPos = Coord(vehiclePos.x, vehiclePos.y, vehiclePos.z);
    }
    return corPos;
}
