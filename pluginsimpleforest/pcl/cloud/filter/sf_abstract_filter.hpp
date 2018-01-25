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
#ifndef SF_ABSTRACT_FILTER_HPP
#define SF_ABSTRACT_FILTER_HPP

#include "sf_abstract_filter.h"



template <typename PointType>
SF_Abstract_Filter<PointType>::SF_Abstract_Filter(typename  pcl::PointCloud<PointType>::Ptr cloud_in):
    SF_Abstract_Cloud<PointType>(cloud_in) {

}

template <typename PointType>
void SF_Abstract_Filter<PointType>::reset() {
    _percentage_remaining = 0;
    SF_Abstract_Filter<PointType>::_cloud_out_filtered.reset(new pcl::PointCloud<PointType> ());
    SF_Abstract_Filter<PointType>::_indices.clear();
}

template<typename PointType>
void SF_Abstract_Filter<PointType>::write_empty() {
    size_t size = SF_Abstract_Filter<PointType>::_cloud_in->size();
    for(size_t i = 0; i < size; i++) {
        SF_Abstract_Cloud<PointType>::_indices.push_back(1);
    }
}

template<typename PointType>
void SF_Abstract_Filter<PointType>::iterate() {
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud (_cloud_out_filtered);
    SF_Abstract_Cloud<PointType>::iterate_over_cloud(kdtree);
}

template <typename PointType>
void SF_Abstract_Filter<PointType>::create_indices() {
    if(_cloud_out_filtered->points.size()>0) {
        iterate();
    } else {
        write_empty();
    }
}

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr SF_Abstract_Filter<PointType>::get_cloud_out_filtered() const {
    return _cloud_out_filtered;
}




#endif // SF_ABSTRACT_FILTER_HPP
