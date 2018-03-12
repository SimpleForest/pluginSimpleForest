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
SF_Normal<PointType, FeatureType>::SF_Normal(typename pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud::Ptr features_out):
    _cloud_in(cloud_in), _features_out(features_out) {

}

template <typename PointType, typename FeatureType>
void SF_Normal<PointType, FeatureType>::set_parameters(float range, bool use_range){
    _range = range;
    _use_range = use_range;
}

template <typename PointType, typename FeatureType>
void SF_Normal<PointType, FeatureType>::set_parameters(int k, bool use_range) {
    _k = k;
    _use_range = use_range;
}

template <typename PointType, typename FeatureType>
void SF_Normal<PointType, FeatureType>::compute_features_range() {
    pcl::NormalEstimation<PointType, FeatureType> ne;
    ne.setInputCloud (_cloud_in);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (_range);
    ne.compute (*SF_Normal<PointType, FeatureType>::_features);;

}

template <typename PointType, typename FeatureType>
void SF_Normal<PointType, FeatureType>::compute_features_knn() {
    pcl::NormalEstimation<PointType, FeatureType> ne;
    ne.setInputCloud (_cloud_in);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    ne.setSearchMethod (tree);
    ne.setKSearch(k);
    ne.compute (*SF_Normal<PointType, FeatureType>::_features);;
}

template <typename PointType, typename FeatureType>
void SF_Normal<PointType, FeatureType>::compute_features() {
    if(_use_range) {
        compute_features_range();
    } else {
        compute_features_knn();
    }
}

#endif // SF_NORMAL_HPP
