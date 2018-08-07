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

#ifndef SF_MULTIPLE_FILTER_HPP
#define SF_MULTIPLE_FILTER_HPP

template<typename PointType>
void Sf_Multiple_Filter<PointType>::reset() {
    _downScaledCloud.reset(new pcl::PointCloud<PointType>);
    _clouds.clear();
}

template<typename PointType>
void Sf_Multiple_Filter<PointType>::create_indices() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloudWithIndex(new pcl::PointCloud<pcl::PointXYZI>);
    for(size_t i = 0; i < _clouds.size(); i++) {
        pcl::PointCloud<PointType>::Ptr cloud = _clouds[i];
        for(size_t j = 0; j < cloud->points.size(); j++) {
            PointType p = cloud->points[j];
            pcl::PointXYZI p2(p.x,p.y,p.z,i);
            mergedCloudWithIndex->points.push_back(p2);
        }
    }
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (mergedCloudWithIndex);
    for(size_t i = 0; i < SF_Abstract_Cloud<PointType>::_cloud_in->points.size(); i++) {
        PointType p = SF_Abstract_Cloud<PointType>::_cloud_in->points[j];
        pcl::PointXYZI p2(p.x,p.y,p.z,0);
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        kdtree.nearestKSearch (p2, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        int index = mergedCloudWithIndex->points[pointIdxNKNSearch[0] ].intensity;
        SF_Abstract_Cloud<PointType>::_indices.push_back(index);
    }
}

#include "sf_multiple_filter.h"

#endif // SF_MULTIPLE_FILTER_HPP
