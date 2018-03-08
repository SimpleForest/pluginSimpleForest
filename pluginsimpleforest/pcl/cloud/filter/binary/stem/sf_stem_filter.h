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
#ifndef SF_STEM_FILTER_H
#define SF_STEM_FILTER_H

#include <pcl/cloud/filter/binary/sf_binary_filter.h>

template <typename PointType>
class SF_Stem_Filter: public Sf_Binary_Filter<PointType> {

    typename pcl::PointCloud<PointType>::Ptr down_scale(const SF_Param_Stem_Filter<PointType> &params);

    void compute_normals(const SF_Param_Stem_Filter<PointType> &params, typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud);

    typename pcl::PointCloud<PointType>::Ptr compute_growth_direction(const SF_Param_Stem_Filter<PointType> &params, typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud);

    void extract_neighbornormals(typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud, typename pcl::PointCloud<PointType>::Ptr neighborhood,   std::vector<int> pointIdxRadiusSearch);

    void calc_growth_direction(typename pcl::PointCloud<PointType>::Ptr cloud_with_growth_direction, typename pcl::PointCloud<PointType>::Ptr neighborhood, PointType point);

    void calc_stem(const SF_Param_Stem_Filter<PointType> &params, typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud);

    void transfer_stem(const SF_Param_Stem_Filter<PointType> &params, typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud, typename pcl::PointCloud<PointType>::Ptr cloud_with_growth_direction);

public:

    SF_Stem_Filter();

    virtual void compute(const SF_Param_Stem_Filter<PointType> &params);



};
#include "pcl/cloud/filter/binary/stem/sf_stem_filter.hpp"
#endif // SF_STEM_FILTER_H
