/****************************************************************************

 Copyright (C) 2017-2018 Jan Hackenberg, free software developer
 All rights reserved.

 Contact : https://github.com/SimpleForest

 Developers : Jan Hackenberg

 This file is part of SimpleForest plugin Version 1 for Computree.

 SimpleForest plugin is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 SimpleForest plugin is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with SimpleForest plugin.  If not, see <http://www.gnu.org/licenses/>.

 PluginSimpleForest is an extended version of the SimpleTree platform.

*****************************************************************************/
#ifndef SF_GROWTH_DIRECTION_HPP
#define SF_GROWTH_DIRECTION_HPP
#include <iostream>

#include "sf_growthDirection.h"

template <typename PointType, typename FeatureType>
SF_GrowthDirection<PointType, FeatureType>::SF_GrowthDirection(typename pcl::PointCloud<PointType>::Ptr cloudIn,
                                                               typename pcl::PointCloud<FeatureType> ::Ptr featuresOut):
    SF_AbstractFeature<PointType, FeatureType>(cloudIn, featuresOut) {

}

template <typename PointType, typename FeatureType>
void SF_GrowthDirection<PointType, FeatureType>::setParameters(float rangeNormal,
                                                               float rangeGd) {
    _rangeNormal = rangeNormal;
    _rangeGd     = rangeGd;
}

template<typename PointType, typename FeatureType>
std::vector<SF_PCAValues> SF_GrowthDirection<PointType,FeatureType>::computeNormalPca() {
    SF_PCA<PointType> sfpca(SF_AbstractCloud<PointType>::_cloudIn);
    sfpca.setParameters(_rangeNormal, false, true);
    sfpca.computeFeatures();
    return sfpca.getPcaValues();
}

template<typename PointType, typename FeatureType>
void SF_GrowthDirection<PointType,FeatureType>::addNormals(std::vector<SF_PCAValues> &values) {
    for(size_t i = 0; i < values.size(); i++) {
        SF_PCAValues val = values[i];
        SF_AbstractCloud<PointType>::_cloudIn->points[i].normal_x = val.getVector3()[0];
        SF_AbstractCloud<PointType>::_cloudIn->points[i].normal_y = val.getVector3()[1];
        SF_AbstractCloud<PointType>::_cloudIn->points[i].normal_z = val.getVector3()[2];
    }
}

template<typename PointType, typename FeatureType>
void SF_GrowthDirection<PointType,FeatureType>::addGrowthDirection(std::vector<SF_PCAValues> &values) {
    for(size_t i = 0; i < values.size(); i++) {
        FeatureType pointGd;
        pointGd.x = SF_AbstractCloud<PointType>::_cloudIn->points[i].x;
        pointGd.y = SF_AbstractCloud<PointType>::_cloudIn->points[i].y;
        pointGd.z = SF_AbstractCloud<PointType>::_cloudIn->points[i].z;
        SF_PCAValues val = values[i];
        if (val.getLambda1() > MAX_LAMBDA3) {
            pointGd.normal_x = val.getVector1()[0];
            pointGd.normal_y = val.getVector1()[1];
            pointGd.normal_z = val.getVector1()[2];
        } else {
            PointType p = SF_AbstractCloud<PointType>::_cloudIn->points[i];
            typename pcl::PointCloud<PointType>::Ptr neighborhood (new pcl::PointCloud<PointType>);
            SF_AbstractCloud<PointType>::extractNeighborsByRange(_kdTree,p,neighborhood, _rangeGd);
            for(size_t j = 0; j < neighborhood->points.size(); j++) {
                neighborhood->points[j].x = neighborhood->points[j].normal_x;
                neighborhood->points[j].y = neighborhood->points[j].normal_y;
                neighborhood->points[j].z = neighborhood->points[j].normal_z;
            }
            Eigen::Vector4f origin (0,0,0,0);
            SF_PCAValues valGd = SF_PCA<PointType>::computeFeaturesFromNeighbors(neighborhood, origin);
            pointGd.normal_x = valGd.getVector3()[0];
            pointGd.normal_y = valGd.getVector3()[1];
            pointGd.normal_z = valGd.getVector3()[2];
        }
        SF_AbstractFeature<PointType, FeatureType>::_featuresOut->points[i] = pointGd;
    }
}

template <typename PointType, typename FeatureType>
void SF_GrowthDirection<PointType, FeatureType>::computeFeatures() {
    if(SF_GrowthDirection::_featuresOut->points.size() != SF_GrowthDirection::_cloudIn->points.size()) {
        SF_GrowthDirection::_featuresOut->resize(SF_GrowthDirection::_cloudIn->points.size());
    }
    _kdTree.reset(new pcl::KdTreeFLANN<PointType> ());
    _kdTree->setInputCloud(SF_GrowthDirection::_cloudIn);
    std::vector<SF_PCAValues> pcaValues = computeNormalPca();
    addNormals(pcaValues);
    addGrowthDirection(pcaValues);
}

#endif // SF_GROWTH_DIRECTION_HPP
