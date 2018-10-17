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

#include "sf_abstractFilter.h"

template <typename PointType>
SF_AbstractFilterDeprecated<PointType>::SF_AbstractFilterDeprecated() {

}

template <typename PointType>
void SF_AbstractFilterDeprecated<PointType>::reset() {
    _percentageRemaining = 0;
    SF_AbstractFilterDeprecated<PointType>::_cloudIn.reset(new pcl::PointCloud<PointType> ());
    SF_AbstractFilterDeprecated<PointType>::_cloudOutFiltered.reset(new pcl::PointCloud<PointType> ());
    SF_AbstractFilterDeprecated<PointType>::_indices.clear();
}

template<typename PointType>
void SF_AbstractFilterDeprecated<PointType>::writeEmpty() {
    size_t size = SF_AbstractFilterDeprecated<PointType>::_cloudIn->size();
    for(size_t i = 0; i < size; i++) {
        SF_AbstractCloud<PointType>::_indices.push_back(1);
    }
}

template<typename PointType>
void SF_AbstractFilterDeprecated<PointType>::iterate() {
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud (_cloudOutFiltered);
    SF_AbstractCloud<PointType>::iterateOverCloud(kdtree);
}

template <typename PointType>
void SF_AbstractFilterDeprecated<PointType>::createIndices() {
    if(_cloudOutFiltered->points.size()>0) {
        iterate();
    } else {
        writeEmpty();
    }
}

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr SF_AbstractFilterDeprecated<PointType>::getCloudOutFiltered() const {
    return _cloudOutFiltered;
}

#endif // SF_ABSTRACT_FILTER_HPP
