#include "veins/modules/analogueModel/DiffractionEP.h"
#include "veins/base/utils/FindModule.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/TraceMobility.h"

#include <iostream>

#define WIDTH 1.8 // assumed vehicle width
#define LENGTH 4.5 // assumed vehicle length
#define HEIGHT 1.5 // assumed vehicle height
#define R_E 8495000 // assumed effective Earth radius

using Veins::TraCIScenarioManager;
using Veins::TraCIMobility;
using Veins::TraCICommandInterface;


const Coord* DiffractionEP::pgs;
double* DiffractionEP::demCache;
size_t DiffractionEP::cacheRows;
size_t DiffractionEP::cacheCols;


DiffractionEP::DiffractionEP(double carrierFrequency, bool considerDEM, std::vector<std::string> demFiles,
        bool isRasterType, double demCellSize, double spacing, bool considerVehicles){
    this->wavelength = BaseWorldUtility::speedOfLight()/carrierFrequency;
    this->spacing = spacing;
    this->demCellSize = demCellSize;
    this->considerVehicles = considerVehicles;
    this->considerDEM = considerDEM;



    if (!considerDEM) return;

    // the DEM cache is shared by all DiffractionEP objects and needs to be initialized once;
    // if demCellSize equals 0, DEM caching is turned off
    if (demCellSize != 0.0 && demCache == NULL) {
        BaseWorldUtility* world = FindModule<BaseWorldUtility*>::findGlobalModule();
        DiffractionEP::pgs = world->getPgs();
        // determine the size of the DEM cache based on the playground size and the grid granularity
        DiffractionEP::cacheCols = (size_t)ceil(pgs->x/demCellSize);
        DiffractionEP::cacheRows = (size_t)ceil(pgs->y/demCellSize);
        int n = DiffractionEP::cacheCols*DiffractionEP::cacheRows;
        // use 1D array for storage and initialize with NaN values
        demCache = new double[n];
        std::fill_n(demCache, n, std::numeric_limits<double>::quiet_NaN());
    }

    const NBHeightMapper& hm = NBHeightMapper::get();
    if (!hm.ready()) NBHeightMapper::loadHM(demFiles, isRasterType);
}


/* 14.11.2021
 * add (deleted) Alternative constructor implementation
 * without this, undefined symbol error will come
 * */
DiffractionEP::DiffractionEP(double carrierFrequency, bool considerDEM, double demCellSize, double spacing, bool considerVehicles){
    this->wavelength = BaseWorldUtility::speedOfLight()/carrierFrequency;
    this->spacing = spacing;
    this->demCellSize = demCellSize;
    this->considerVehicles = considerVehicles;
    this->considerDEM = considerDEM;

    if (!considerDEM) return;

    // the DEM cache is shared by all DiffractionEP objects and needs to be initialized once;
    // if demCellSize equals 0, DEM caching is turned off
    if (demCellSize != 0.0 && demCache == NULL) {
        BaseWorldUtility* world = FindModule<BaseWorldUtility*>::findGlobalModule();
        DiffractionEP::pgs = world->getPgs();
        // determine the size of the DEM cache based on the playground size and the grid granularity
        DiffractionEP::cacheCols = (size_t)ceil(pgs->x/demCellSize);
        DiffractionEP::cacheRows = (size_t)ceil(pgs->y/demCellSize);
        int n = DiffractionEP::cacheCols*DiffractionEP::cacheRows;
        // use 1D array for storage and initialize with NaN values
        demCache = new double[n];
        std::fill_n(demCache, n, std::numeric_limits<double>::quiet_NaN());
    }

    const NBHeightMapper& hm = NBHeightMapper::get();
    if (!hm.ready()) throw cRuntimeError("No height map for environmental diffraction model");
}


void DiffractionEP::filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos) {
    Signal& s = frame->getSignal();

    double factor = calcAttenuation(senderPos, receiverPos);
    EV << "Attenuation by environmental diffraction is: " << factor << endl;

    bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
    const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();
    ConstantSimpleConstMapping* attMapping = new ConstantSimpleConstMapping(domain, factor);
    s.addAttenuation(attMapping);
}

double DiffractionEP::calcAttenuation(const Coord& senderPos, const Coord& receiverPos) {
    std::map<double, double> edgeMap;

    Coord los = receiverPos - senderPos;
    double dLos = los.length();
    los = los/dLos;
    edgeMap[0.0] = senderPos.z;
    edgeMap[dLos] = receiverPos.z;

    /*TraCIScenarioManager* traciManager = FindModule<TraCIScenarioManager*>::findGlobalModule();
    if (traciManager == NULL) {
        throw cRuntimeError("Could not find TraCIScenarioManager module");
    }

    if (considerVehicles) {
        // check all equipped cars first

        std::map<std::string, HostPos*>* hostsGrid = traciManager->getHostsGrid();

        Coord bboxP1 = Coord(std::min(senderPos.x, receiverPos.x), std::min(senderPos.y, receiverPos.y));
        Coord bboxP2 = Coord(std::max(senderPos.x, receiverPos.x), std::max(senderPos.y, receiverPos.y));

        double css = traciManager->carCellSize;
        size_t fromRow = 0, toRow = 0, fromCol = 0, toCol = 0;
        if (css != 0.0) {
            fromRow = std::max(0, int(bboxP1.y / css));
            toRow = std::max(0, int(bboxP2.y / css));
            fromCol = std::max(0, int(bboxP1.x / css));
            toCol = std::max(0, int(bboxP2.x / css));
        }

        for (size_t row = fromRow; row <= toRow; ++row) {
            if (row >= traciManager->carGridRows) break;
            for (size_t col = fromCol; col <= toCol; ++col) {
                if (col >= traciManager->carGridCols) break;
                const std::map<std::string, HostPos*> cell = hostsGrid[row*traciManager->carGridCols + col];
                for (auto const& host : cell) {
                    const Coord& pos = std::get<0>(*(host.second));
                    const Coord& orient = std::get<1>(*(host.second));
                    if (senderPos == pos || receiverPos == pos) continue;
                    // check if this car is in LOS; if yes, add it to knife-edge map
                    double d, h;
                    std::tie(d, h) = isInLOS(pos, orient, senderPos, receiverPos);
                    if (d < 0) continue;

                    std::map<double, double>::iterator it = edgeMap.find(d);
                    if (it != edgeMap.end()) {
                        if (it->second < h) it->second = h;
                    } else edgeMap[d] = h;
                }
            }
        }


    }*/

    if (considerDEM) {
        // open DEM, read equidistant points along LOS (dependent on spacing) and add them to knife-edge map
        //TraCICommandInterface* traciCI = traciManager->getCommandInterface();
        TraceMobility* tm = FindModule<TraceMobility*>::findGlobalModule();
        const NBHeightMapper& hm = NBHeightMapper::get();
        if (!hm.ready()) throw cRuntimeError("No height map for environmental diffraction model");
        for (double d = spacing; d < dLos; d += spacing) {
            if (edgeMap.find(d) != edgeMap.end()) continue; //if we already have a car at this distance, continue
            const Coord p = senderPos + los*d;
            // demCellSize of 0 means no DEM caching
            if (demCellSize == 0.0) {
            // get the height value by querying the DEM
                double lon, lat;
                //std::tie(lon, lat) = traciCI->getLonLat(p);
                Coord lonLat = tm->cartToLonLat(p);
                //edgeMap[d] = hm.getZ(Position(lon, lat));
                edgeMap[d] = hm.getZ(Position(lonLat.x, lonLat.y));
            } else {
                // determine the position in the DEM cache for coordinate p
                size_t x = (size_t)(p.x/demCellSize);
                size_t y = (size_t)(p.y/demCellSize);
                // if this height value has not been determined yet, compute it now
                if (std::isnan(DiffractionEP::demCache[y*DiffractionEP::cacheCols + x])) {
                    // determine the coordinates of the center of the required grid cell
                    double cellCenterX, cellCenterY;
                    if (x < DiffractionEP::cacheCols - 1) cellCenterX = (0.5 + x)*demCellSize;
                    else cellCenterX = (DiffractionEP::pgs->x + x*demCellSize)/2;
                    if (y < DiffractionEP::cacheRows - 1) cellCenterY = (0.5 + y)*demCellSize;
                    else cellCenterY = (DiffractionEP::pgs->y + y*demCellSize)/2;
                    // get the height value by querying the DEM
                    double lon, lat;
                    //std::tie(lon, lat) = traciCI->getLonLat(Coord(cellCenterX, cellCenterY));
                    Coord lonLat = tm->cartToLonLat(Coord(cellCenterX, cellCenterY));
                    //demCache[y*DiffractionEP::cacheCols + x] = hm.getZ(Position(lon, lat));
                    demCache[y*DiffractionEP::cacheCols + x] = hm.getZ(Position(lonLat.x, lonLat.y));
                }
                // return the height value as stored in the DEM cache
                edgeMap[d] = demCache[y*DiffractionEP::cacheCols + x];
            }
        }
    }
    // now apply multiple knife-edge model
    // based on ITU-R Recommendation P.526-11: Propagation by diffraction (10/2009)
    if (edgeMap.size() == 2)
    {
        std::cout<<"no knife edges"<<std::endl;
        return 1.0;
    }
    /*
    if (edgeMap.size() >= 6)
    {
        std::cout<<"too many knife edges, model not applicable"<<std::endl;
        return 1.0;
    }
    */
    std::map<double,double>::iterator it = edgeMap.begin();
    double d1,d2,h;
    double sum,before,after;
    double mid_diff_to_lower, higher_diff_to_lower;
    double v=0,L=0;
    unsigned int count=edgeMap.size()-2;

    for(;it!=edgeMap.end();++it)
    {
            if(it->first==dLos)
            {
//                std::cout<<"this is receiver at:"<<it->first<<" "<<it->second<<std::endl;
                continue;
            }
//            std::cout<<"this is edge :"<<it->first<<" "<<it->second<<std::endl;
//            auto nx = std::next(it, 2);
//            std::prev(it, 4)
            double former_first=it->first;
            double former_second=it->second;
            ++it;
            double mid_first=it->first;
            double mid_second=it->second;
            ++it;
            double latter_first=it->first;
            double latter_second=it->second;
            --it;
            --it;
            d1=mid_first-former_first;
            d2=latter_first-mid_first;
            /*former edge is lower*/
            if(latter_second>former_second)
            {
                higher_diff_to_lower=latter_second-former_second;
                sum=d1+d2;      //  d1/sum=mid_diff_to_lower/higher_diff_to_lower;
                mid_diff_to_lower=d1*higher_diff_to_lower/sum;
                h=mid_second-former_second-mid_diff_to_lower;
                v=getDiffParameter(d1*spacing/1000,d2*spacing/1000,h);
                L+=calcDiffLoss(v);
            }
            /*latter edge is lower*/
            else if(latter_second<former_second)
            {
                higher_diff_to_lower=former_second-latter_second;
                sum=d1+d2;      //  d1/sum=mid_diff_to_lower/higher_diff_to_lower;
                mid_diff_to_lower=d2*higher_diff_to_lower/sum;
                h=it->second-latter_second-mid_diff_to_lower;
                v=getDiffParameter(d1*spacing,d2*spacing,h);
                L+=calcDiffLoss(v);
            }
            else
            {
                    h=mid_second-latter_second;
                    v=getDiffParameter(d1*spacing,d2*spacing,h);
                    L+=calcDiffLoss(v);
            }
//        if(it)
            std::cout<<"loss calculated  :"<<L<<std::endl;
    }

    return FWMath::dBm2mW(-L);
/*
    double d_p, v_p;
    std::tie(d_p, v_p) = getHighestV(edgeMap, 0.0, dLos);
    if (v_p <= -0.78) return 1.0;

    double d_t, v_t;
    if (++(edgeMap.begin()) == edgeMap.find(d_p)) {
        v_t = -1.0;
    } else {
        std::tie(d_t, v_t) = getHighestV(edgeMap, 0.0, d_p);
    }

    double d_r, v_r;
    if (++(edgeMap.find(d_p)) == edgeMap.find(dLos)) {
        v_r = -1.0;
    } else {
        std::tie(d_r, v_r) = getHighestV(edgeMap, d_p, dLos);
    }

    double T = 1.0 - exp(-getJFuncValue(v_p)/6.0);
    double C = 10.0 + 0.04*dLos/1000.0;
    double L = getJFuncValue(v_p) + T*(getJFuncValue(v_t) + getJFuncValue(v_r) + C);


*/
//    return FWMath::dBm2mW(-L);
}

std::pair<double, double> DiffractionEP::isInLOS(const Coord& pos, const Coord& orient, const Coord& senderPos, const Coord& receiverPos) {
    std::vector<Coord> shape;
    Coord orient2D(orient.x, orient.y);
    orient2D = orient2D/orient2D.length();
    Coord perpend(-(orient2D.y), orient2D.x);

    shape.push_back(pos + perpend*WIDTH/2); //front left corner
    shape.push_back(pos - perpend*WIDTH/2); //front right corner
    double elev_angle = asin(orient.z/orient.length());
    double apparentLength = LENGTH*cos(elev_angle);
    shape.push_back(pos - perpend*WIDTH/2 - orient2D*apparentLength); //rear right corner
    shape.push_back(pos + perpend*WIDTH/2 - orient2D*apparentLength); //rear left corner

    bool inLOS = false;
    std::vector<Coord>::const_iterator i = shape.begin();
    std::vector<Coord>::const_iterator j = (shape.rbegin()+1).base();
    for (; i != shape.end(); j = i++) {
        Coord c1 = *i;
        Coord c2 = *j;
        if (segmentsIntersect(senderPos, receiverPos, c1, c2)) {
            inLOS = true;
            break;
        }
    }

    if (inLOS) {
        // determine center and height of the vehicle
        Coord center = pos - orient/orient.length()*LENGTH/2;
        Coord center0 = center;
        center0.z = 0;
        Coord senderPos0 = senderPos;
        senderPos0.z = 0;
        double d = (center0 - senderPos0).length();
        double h = HEIGHT/cos(elev_angle);
        if (h > LENGTH/2) h = LENGTH/2;
        h += center.z;
        return std::make_pair(d, h);
    } else return std::make_pair(-1.0, -1.0);
}

bool DiffractionEP::segmentsIntersect(Coord p1From, Coord p1To, Coord p2From, Coord p2To) {
    Coord p1Vec = p1To - p1From;
    Coord p2Vec = p2To - p2From;
    Coord p1p2 = p1From - p2From;

    double D = (p1Vec.x * p2Vec.y - p1Vec.y * p2Vec.x);

    double p1Frac = (p2Vec.x * p1p2.y - p2Vec.y * p1p2.x) / D;
    if (p1Frac < 0 || p1Frac > 1) return false;

    double p2Frac = (p1Vec.x * p1p2.y - p1Vec.y * p1p2.x) / D;
    if (p2Frac < 0 || p2Frac > 1) return false;

    return true;
}

std::pair<double, double> DiffractionEP::getHighestV(const std::map<double, double>& edgeMap, double a, double b) {
    std::pair<double, double> result = std::make_pair(0.0, -1.0);
    for (std::map<double, double>::const_iterator it = ++(edgeMap.find(a)); it != edgeMap.find(b); ++it) {
        double d_an = it->first - a;
        double d_nb = b - it->first;
        double d_ab = b - a;
        double h = it->second + (d_an*d_nb/2/R_E) - ((edgeMap.at(a)*d_nb + edgeMap.at(b)*d_an)/d_ab);
        double v_n = h*sqrt(2*d_ab/wavelength/d_an/d_nb);
        if (v_n > result.second) {
            result.first = it->first;
            result.second = v_n;
        }
    }
    return result;
}

double DiffractionEP::getJFuncValue(double v) {
    if (v <= -0.78) return 0;
    else return 6.9 + 20*log10(sqrt((v - 0.1)*(v - 0.1) + 1) + v - 0.1);
}

/*
 * calculate fresnel-kirchhoff diffraction parameter
 * */
double DiffractionEP::getDiffParameter(double d1, double d2, double h)
{
    return h*sqrt(2*(d1+d2)/this->wavelength*d1*d2);
}

/*
 * calculate diffraction loss
 * diffraction parameter as input
 * */
double DiffractionEP::calcDiffLoss(double parameter)
{
    if(parameter>-0.8&&parameter<0)
        return -20*log10(0.5-0.62*parameter);
    else if ( parameter>= 0 && parameter<1)
        return -20*log10(pow(0.5,-0.95*parameter));
    else if ( parameter>= 1 && parameter<2.4)
        return -20*log10(0.4-sqrt((0.1184-pow((0.38-0.1*parameter),2))));
    else if ( parameter > 2.4)
        return -20*log10(0.225/parameter);
    else
        return 0;
}
