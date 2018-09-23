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
#ifndef SF_GROUND_FILTER_H
#define SF_GROUND_FILTER_H

#include <pcl/cloud/filter/binary/sf_abstractBinaryFilter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

#include "pcl/sf_math.h"
#include "pcl/cloud/feature/pca/sf_pca.h"

template <typename PointType>
class SF_GroundFilter: public Sf_AbstractBinaryFilter<PointType> {
    SF_ParamGroundFilter<PointType> _params;
    void transferNormalAndFilter(const SF_ParamGroundFilter<PointType> &params,
                                 typename pcl::PointCloud<PointType>::Ptr cloudIn,
                                 typename pcl::PointCloud<PointType>::Ptr cloudWithGrowthDirection);
public:
    SF_GroundFilter();
    virtual void compute();
    void setParams(SF_ParamGroundFilter<PointType> &params);
};

#include "pcl/cloud/filter/binary/ground/sf_groundFilter.hpp"

#endif // SF_GROUND_FILTER_H
