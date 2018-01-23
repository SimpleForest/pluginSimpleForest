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
#ifndef SF_NORMAL_HPP
#define SF_NORMAL_HPP
#include "sf_normal.h"
#include <pcl/features/normal_3d.h>
template <typename PointType, typename FeatureType>
SF_Normal<PointType, FeatureType>::SF_Normal(typename pcl::PointCloud<PointType>::Ptr cloud_in):
    _cloud_in(cloud_in) {

}
template <typename PointType, typename FeatureType>
void compute_feature() {
    pcl::NormalEstimation<PointType, FeatureType> ne;
    ne.setInputCloud (_cloud_in);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<FeatureType>::Ptr cloud_normals (new pcl::PointCloud<FeatureType>);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);;
}

#endif // SF_NORMAL_HPP
