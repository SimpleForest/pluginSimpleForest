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
#ifndef SF_GROWTH_DIRECTION_H
#define SF_GROWTH_DIRECTION_H

#include "pcl/cloud/feature/sf_abstractFeature.h"
#include "../pca/sf_pca.h"

template <typename PointType, typename FeatureType>
class SF_GrowthDirection: public  SF_AbstractFeature<PointType, FeatureType> {
    float _rangeNormal = 0.1f;
    float _rangeGd = 0.3f;
    typename pcl::KdTree<PointType>::Ptr _kdTree;
    static constexpr float MAX_LAMBDA3 = 0.9;
    void addNormals(std::vector<SF_PCAValues>& values);
    void addGrowthDirection(std::vector<SF_PCAValues>& values);
    virtual void createIndices(){; }
    virtual void createIndex(PointType point,
                         float sqrdDistance) {}
    virtual void reset(){_featuresOut.reset(new pcl::PointCloud<FeatureType>);}


public:
    SF_GrowthDirection(typename pcl::PointCloud<PointType>::Ptr cloudIn,
                       typename pcl::PointCloud<FeatureType>::Ptr featuresOut);
    void computeFeatures();
    std::vector<SF_PCAValues> computeNormalPca();
    void setParameters(float rangeNormal, float rangeGd);
};

#include "sf_growthDirection.hpp"

#endif // SF_GROWTH_DIRECTION_H
