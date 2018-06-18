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

#include <pcl/cloud/filter/binary/sf_binary_filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

#include "pcl/sf_math.h"
#include "pcl/cloud/feature/pca/sf_pca.h"

template <typename PointType>
class SF_Ground_Filter: public Sf_Binary_Filter<PointType> {
    SF_Param_Ground_Filter<PointType> _params;
    void transfer_normal_and_filter(const SF_Param_Ground_Filter<PointType> &params, typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud,
                                                                      typename pcl::PointCloud<PointType>::Ptr cloud_with_growth_direction);
public:
    SF_Ground_Filter();
    virtual void compute();
    void set_params(SF_Param_Ground_Filter<PointType> &params);
};

#include "pcl/cloud/filter/binary/ground/sf_ground_filter.hpp"
#endif // SF_GROUND_FILTER_H
