/****************************************************************************/
/// @file    NBHeightMapper.cpp
/// @author  Jakob Erdmann
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @date    Sept 2011
/// @version $Id: NBHeightMapper.cpp 22608 2017-01-17 06:28:54Z behrisch $
///
// Set z-values for all network positions based on data from a height map
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2011-2017 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
//#ifdef _MSC_VER
//#include <windows_config.h>
//#else
//#include <config.h>
//#endif

#include <string>
//#include <utils/common/MsgHandler.h>
//#include <utils/common/ToString.h>
//#include "StringUtils.h"
//#include <utils/options/OptionsCont.h>
#include "GeomHelper.h"
#include "NBHeightMapper.h"
//#include "GeoConvHelper.h"
//#include <utils/common/RGBColor.h>

//#ifdef HAVE_GDAL
#include <ogrsf_frmts.h>
#include <ogr_api.h>
#include <gdal_priv.h>
//#endif
#include "veins/base/utils/MiXiMDefs.h"

//#ifdef CHECK_MEMORY_LEAKS
//#include <foreign/nvwa/debug_new.h>
//#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// static members
// ===========================================================================
NBHeightMapper NBHeightMapper::Singleton;

// ===========================================================================
// method definitions
// ===========================================================================


NBHeightMapper::NBHeightMapper():
    myRTree(&Triangle::addSelf), myRaster(0) {
}


NBHeightMapper::~NBHeightMapper() {
    clearData();
}


const NBHeightMapper&
NBHeightMapper::get() {
    return Singleton;
}


bool
NBHeightMapper::ready() const {
    return myRaster != 0 || myTriangles.size() > 0;
}


double
NBHeightMapper::getZ(const Position& geo) const {
    if (!ready()) {
        EV_WARN << "Cannot supply height since no height data was loaded" << endl;
        return 0;
    }

    if (myRaster != 0) {
        double result = -1e6;
        if (myBoundary.around(geo)) {
            const int xSize = int((myBoundary.xmax() - myBoundary.xmin()) / mySizeOfPixel.x() + .5);
            const double normX = (geo.x() - myBoundary.xmin()) / mySizeOfPixel.x();
            const double normY = (geo.y() - myBoundary.ymax()) / mySizeOfPixel.y();
            PositionVector corners;
            corners.push_back(Position(floor(normX) + 0.5, floor(normY) + 0.5, myRaster[(int)normY * xSize + (int)normX]));
            if (normX - floor(normX) > 0.5) {
                corners.push_back(Position(floor(normX) + 1.5, floor(normY) + 0.5, myRaster[(int)normY * xSize + (int)normX + 1]));
            } else {
                corners.push_back(Position(floor(normX) - 0.5, floor(normY) + 0.5, myRaster[(int)normY * xSize + (int)normX - 1]));
            }
            if (normY - floor(normY) > 0.5) {
                corners.push_back(Position(floor(normX) + 0.5, floor(normY) + 1.5, myRaster[((int)normY + 1) * xSize + (int)normX]));
            } else {
                corners.push_back(Position(floor(normX) + 0.5, floor(normY) - 0.5, myRaster[((int)normY - 1) * xSize + (int)normX]));
            }
            result = Triangle(corners).getZ(Position(normX, normY));
        }
        if (result > -1e5 && result < 1e5) {
            return result;
        }
    }
    // coordinates in degrees hence a small search window
    float minB[2];
    float maxB[2];
    minB[0] = (float)geo.x() - 0.00001f;
    minB[1] = (float)geo.y() - 0.00001f;
    maxB[0] = (float)geo.x() + 0.00001f;
    maxB[1] = (float)geo.y() + 0.00001f;
    QueryResult queryResult;
    int hits = myRTree.Search(minB, maxB, queryResult);
    Triangles result = queryResult.triangles;
    assert(hits == (int)result.size());
    //UNUSED_PARAMETER(hits); // only used for assertion

    for (Triangles::iterator it = result.begin(); it != result.end(); it++) {
        const Triangle* triangle = *it;
        if (triangle->contains(geo)) {
            return triangle->getZ(geo);
        }
    }
    EV_WARN << "Could not get height data for coordinate " << geo << endl;
    return 0;
}


void
NBHeightMapper::addTriangle(PositionVector corners) {
    Triangle* triangle = new Triangle(corners);
    myTriangles.push_back(triangle);
    Boundary b = corners.getBoxBoundary();
    const float cmin[2] = {(float) b.xmin(), (float) b.ymin()};
    const float cmax[2] = {(float) b.xmax(), (float) b.ymax()};
    myRTree.Insert(cmin, cmax, triangle);
}


void
NBHeightMapper::loadHM(std::vector<std::string> files, bool isRasterType) {
    if (isRasterType) {
        // parse file(s)
        for (std::vector<std::string>::const_iterator file = files.begin(); file != files.end(); ++file) {
            EV << "Parsing from raster file '" << *file << "'" << endl;
            int numFeatures = Singleton.loadTiff(*file);
            EV << " done (parsed " << numFeatures << " features, Boundary: " << Singleton.getBoundary() << ")." << endl;
        }
    } else {
        // parse file(s)
        for (std::vector<std::string>::const_iterator file = files.begin(); file != files.end(); ++file) {
            EV << "Parsing from shape-file '" << *file << "'" << endl;
            int numFeatures = Singleton.loadShapeFile(*file);
            EV << " done (parsed " << numFeatures << " features, Boundary: " << Singleton.getBoundary() << ")." << endl;
        }
    }
}


int
NBHeightMapper::loadShapeFile(const std::string& file) {
#if GDAL_VERSION_MAJOR < 2
    OGRRegisterAll();
    OGRDataSource* ds = OGRSFDriverRegistrar::Open(file.c_str(), FALSE);
#else
    GDALAllRegister();
    GDALDataset* ds = (GDALDataset*)GDALOpenEx(file.c_str(), GDAL_OF_VECTOR | GA_ReadOnly, NULL, NULL, NULL);
#endif
    if (ds == NULL) {
        throw ProcessError("Could not open shape file '" + file + "'.");
    }

    // begin file parsing
    OGRLayer* layer = ds->GetLayer(0);
    layer->ResetReading();

    // triangle coordinates are stored in WGS84 and later matched with network coordinates in WGS84
    // build coordinate transformation
    OGRSpatialReference* sr_src = layer->GetSpatialRef();
    OGRSpatialReference sr_dest;
    sr_dest.SetWellKnownGeogCS("WGS84");
    OGRCoordinateTransformation* toWGS84 = OGRCreateCoordinateTransformation(sr_src, &sr_dest);
    if (toWGS84 == 0) {
        EV_WARN << "Could not create geocoordinates converter; check whether proj.4 is installed." << endl;
    }

    int numFeatures = 0;
    OGRFeature* feature;
    layer->ResetReading();
    while ((feature = layer->GetNextFeature()) != NULL) {
        OGRGeometry* geom = feature->GetGeometryRef();
        assert(geom != 0);

        // @todo gracefull handling of shapefiles with unexpected contents or any error handling for that matter
        assert(std::string(geom->getGeometryName()) == std::string("POLYGON"));
        // try transform to wgs84
        geom->transform(toWGS84);
        OGRLinearRing* cgeom = ((OGRPolygon*) geom)->getExteriorRing();
        // assume TIN with with 4 points and point0 == point3
        assert(cgeom->getNumPoints() == 4);
        PositionVector corners;
        for (int j = 0; j < 3; j++) {
            Position pos((double) cgeom->getX(j), (double) cgeom->getY(j), (double) cgeom->getZ(j));
            corners.push_back(pos);
            myBoundary.add(pos);
        }
        addTriangle(corners);
        numFeatures++;

        /*
        OGRwkbGeometryType gtype = geom->getGeometryType();
        switch (gtype) {
            case wkbPolygon: {
                break;
            }
            case wkbPoint: {
                WRITE_WARNING("got wkbPoint");
                break;
            }
            case wkbLineString: {
                WRITE_WARNING("got wkbLineString");
                break;
            }
            case wkbMultiPoint: {
                WRITE_WARNING("got wkbMultiPoint");
                break;
            }
            case wkbMultiLineString: {
                WRITE_WARNING("got wkbMultiLineString");
                break;
            }
            case wkbMultiPolygon: {
                WRITE_WARNING("got wkbMultiPolygon");
                break;
            }
            default:
                WRITE_WARNING("Unsupported shape type occured");
            break;
        }
        */
        OGRFeature::DestroyFeature(feature);
    }
#if GDAL_VERSION_MAJOR < 2
    OGRDataSource::DestroyDataSource(ds);
#else
    GDALClose(ds);
#endif
    OCTDestroyCoordinateTransformation(toWGS84);
    OGRCleanupAll();
    return numFeatures;
}


int
NBHeightMapper::loadTiff(const std::string& file) {
    GDALAllRegister();
    GDALDataset* poDataset = (GDALDataset*)GDALOpen(file.c_str(), GA_ReadOnly);
    if (poDataset == 0) {
        EV_ERROR << "Cannot load GeoTIFF file." << endl;
        return 0;
    }
    const int xSize = poDataset->GetRasterXSize();
    const int ySize = poDataset->GetRasterYSize();
    double adfGeoTransform[6];
    if (poDataset->GetGeoTransform(adfGeoTransform) == CE_None) {
        Position topLeft(adfGeoTransform[0], adfGeoTransform[3]);
        mySizeOfPixel.set(adfGeoTransform[1], adfGeoTransform[5]);
        const double horizontalSize = xSize * mySizeOfPixel.x();
        const double verticalSize = ySize * mySizeOfPixel.y();
        myBoundary.add(topLeft);
        myBoundary.add(topLeft.x() + horizontalSize, topLeft.y() + verticalSize);
    } else {
        EV_ERROR << "Could not parse geo information from " << file << "." << endl;
        return 0;
    }


    const int picSize = xSize * ySize;
    myRaster = (float*)CPLMalloc(sizeof(float) * picSize);
    for (int i = 1; i <= poDataset->GetRasterCount(); i++) {
        GDALRasterBand* poBand = poDataset->GetRasterBand(i);
        /*if (poBand->GetColorInterpretation() != GCI_GrayIndex) {
            EV_ERROR << ("Unknown color band in " + file + ".") << endl;
            clearData();
            break;
        }
        if (poBand->GetRasterDataType() != GDT_Int16) {
            EV_ERROR << ("Unknown data type in " + file + ".") << endl;
            clearData();
            break;
        }*/
        assert(xSize == poBand->GetXSize() && ySize == poBand->GetYSize());
        if (poBand->RasterIO(GF_Read, 0, 0, xSize, ySize, myRaster, xSize, ySize, GDT_Float32, 0, 0) == CE_Failure) {
            EV_ERROR << "Failure in reading " << file << "." << endl;
            clearData();
            break;
        }
    }
    GDALClose(poDataset);
    return picSize;
}


void
NBHeightMapper::clearData() {
    for (Triangles::iterator it = myTriangles.begin(); it != myTriangles.end(); it++) {
        delete *it;
    }
    myTriangles.clear();
    if (myRaster != 0) {
        CPLFree(myRaster);
        myRaster = 0;
    }
    myBoundary.reset();
}


// ===========================================================================
// Triangle member methods
// ===========================================================================
NBHeightMapper::Triangle::Triangle(const PositionVector& corners):
    myCorners(corners) {
    assert(myCorners.size() == 3);
    // @todo assert non-colinearity
}


void
NBHeightMapper::Triangle::addSelf(const QueryResult& queryResult) const {
    queryResult.triangles.push_back(this);
}


bool
NBHeightMapper::Triangle::contains(const Position& pos) const {
    return myCorners.around(pos);
}


double
NBHeightMapper::Triangle::getZ(const Position& geo) const {
    // en.wikipedia.org/wiki/Line-plane_intersection
    Position p0 = myCorners.front();
    Position line(0, 0, 1);
    p0.sub(geo); // p0 - l0
    Position normal = normalVector();
    return p0.dotProduct(normal) / line.dotProduct(normal);
}


Position
NBHeightMapper::Triangle::normalVector() const {
    // @todo maybe cache result to avoid multiple computations?
    Position side1 = myCorners[1] - myCorners[0];
    Position side2 = myCorners[2] - myCorners[0];
    return side1.crossProduct(side2);
}


/****************************************************************************/

