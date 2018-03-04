/****************************************************************************

 Copyright (C) 2017-2017 Jan Hackenberg, free software developer
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
#ifndef SF_BINARY_FILTER_HPP
#define SF_BINARY_FILTER_HPP
#include"sf_binary_filter.h"

template <typename PointType>
Sf_Binary_Filter<PointType>::Sf_Binary_Filter() {
    reset();
}

template<typename PointType>
void Sf_Binary_Filter<PointType>::reset() {
    SF_Abstract_Filter<PointType>::reset();
    _cloud_out_filtered_noise.reset(new pcl::PointCloud<PointType>);
}

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr Sf_Binary_Filter<PointType>::get_cloud_out_filtered_noise() const {
    return _cloud_out_filtered_noise;
}

template<typename PointType>
double Sf_Binary_Filter<PointType>::get_percentage() {
    double in = SF_Abstract_Cloud<PointType>::_cloud_in->points.size();
    double out = SF_Abstract_Filter<PointType>::_cloud_out_filtered->points.size()*100.0;
    if(in==0) {
        return 0;
    }
    return (out/in);
}

template<typename PointType>
void Sf_Binary_Filter<PointType>::create_index(PointType point,
                                               float sqrd_distance) {
    if(Sf_Binary_Filter<PointType>::equals_by_sqrt_distance(sqrd_distance)) {
        SF_Abstract_Cloud<PointType>::_indices.push_back(0);
    } else {
        SF_Abstract_Cloud<PointType>::_indices.push_back(1);
        _cloud_out_filtered_noise->points.push_back(point);
    }
}

#endif // SF_BINARY_FILTER_HPP
