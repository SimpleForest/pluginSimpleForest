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
#ifndef SF_PCA_H
#define SF_PCA_H

#include <pcl/features/normal_3d.h>

#include "pcl/cloud/feature/sf_abstractFeature.h"
#include "sf_pcavalues.h"

template <typename PointType>
class SF_PCA: public  SF_AbstractCloud <PointType> {
private:
    float _range;
    bool _useRange;
    int _k;
    bool _centerZero;
    std::vector<SF_PCAValues> pcaValues;
    void extractNeighbors(typename pcl::KdTree<PointType>::Ptr kdTree,
                           PointType p,
                           typename pcl::PointCloud<PointType>::Ptr neighborhood);
public:
    SF_PCA(typename pcl::PointCloud<PointType>::Ptr cloud_in);
    virtual void computeFeatures();
    virtual void createIndices(){}
    virtual void createIndex(PointType point,
                             float sqrd_distance){}
    virtual void reset(){pcaValues.clear();}
    static SF_PCAValues computeFeaturesFromNeighbors(typename pcl::PointCloud<PointType>::Ptr neighborhood,
                                                   const Eigen::Vector4f& xyz_centroid);
    void computeFeaturesForPoint(const PointType& p,
                                 typename pcl::KdTree<PointType>::Ptr kdTree,
                                 int index);
    void setParameters(float range,
                       bool centerZero,
                       bool useRange);
    void setParameters(int k,
                       bool centerZero);
    std::vector<SF_PCAValues> getPcaValues() const;
};

#include "sf_pca.hpp"

#endif // SF_PCA_H

