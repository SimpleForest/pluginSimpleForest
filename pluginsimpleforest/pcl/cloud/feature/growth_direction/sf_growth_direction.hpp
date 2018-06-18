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
#ifndef SF_GROWTH_DIRECTION_HPP
#define SF_GROWTH_DIRECTION_HPP
#include <iostream>

#include "sf_growth_direction.h"

template <typename PointType, typename FeatureType>
SF_Growth_Direction<PointType, FeatureType>::SF_Growth_Direction(typename pcl::PointCloud<PointType>::Ptr cloud_in, typename pcl::PointCloud<FeatureType> ::Ptr features_out):
    SF_Abstract_Feature<PointType, FeatureType>(cloud_in, features_out) {

}

template <typename PointType, typename FeatureType>
void SF_Growth_Direction<PointType, FeatureType>::set_parameters(float range_normal, float range_gd) {
    _range_normal = range_normal;
    _range_gd     = range_gd;
}

template<typename PointType, typename FeatureType>
std::vector<PCA_Values> SF_Growth_Direction<PointType,FeatureType>::compute_normal_pca() {
    SF_PCA<PointType> sfpca(_cloud_in);
    sfpca.set_parameters(_range_normal, false, true);
    sfpca.compute_features();
    return sfpca.get_pca_values();
}

template<typename PointType, typename FeatureType>
void SF_Growth_Direction<PointType,FeatureType>::add_normals(std::vector<PCA_Values> &values) {
    for(size_t i = 0; i < values.size(); i++) {
        PCA_Values val = values[i];
        _cloud_in->points[i].normal_x = val.getVector3()[0];
        _cloud_in->points[i].normal_y = val.getVector3()[1];
        _cloud_in->points[i].normal_z = val.getVector3()[2];
    }
}

template<typename PointType, typename FeatureType>
void SF_Growth_Direction<PointType,FeatureType>::add_growth_direction(std::vector<PCA_Values> &values) {
    for(size_t i = 0; i < values.size(); i++) {
        typename FeatureType point_gd;
        point_gd.x = _cloud_in->points[i].x;
        point_gd.y = _cloud_in->points[i].y;
        point_gd.z = _cloud_in->points[i].z;
        PCA_Values val = values[i];
        if (val.getLambda1() > MAX_LAMBDA3) {
            point_gd.normal_x = val.getVector1()[0];
            point_gd.normal_y = val.getVector1()[1];
            point_gd.normal_z = val.getVector1()[2];
        } else {
            typename PointType p = _cloud_in->points[i];
            typename pcl::PointCloud<PointType>::Ptr neighborhood (new pcl::PointCloud<PointType>);
            extract_neighbors_by_range(_kd_tree,p,neighborhood, _range_gd);
            for(size_t j = 0; j < neighborhood->points.size(); j++) {
                neighborhood->points[j].x = neighborhood->points[j].normal_x;
                neighborhood->points[j].y = neighborhood->points[j].normal_y;
                neighborhood->points[j].z = neighborhood->points[j].normal_z;
            }
            Eigen::Vector4f origin (0,0,0,0);
            PCA_Values val_gd = SF_PCA<PointType>::compute_features_from_neighbors(neighborhood, origin);
            point_gd.normal_x = val_gd.getVector3()[0];
            point_gd.normal_y = val_gd.getVector3()[1];
            point_gd.normal_z = val_gd.getVector3()[2];
        }
        _features_out->points[i] = point_gd;
    }
}

template <typename PointType, typename FeatureType>
void SF_Growth_Direction<PointType, FeatureType>::compute_features() {
    if(SF_Growth_Direction::_features_out->points.size() != SF_Growth_Direction::_cloud_in->points.size()) {
        SF_Growth_Direction::_features_out->resize(SF_Growth_Direction::_cloud_in->points.size());
    }
    _kd_tree.reset(new pcl::KdTreeFLANN<PointType> ());
    _kd_tree->setInputCloud(SF_Growth_Direction::_cloud_in);
    std::vector<PCA_Values> pca_values = compute_normal_pca();
    add_normals(pca_values);
    add_growth_direction(pca_values);
}

#endif // SF_GROWTH_DIRECTION_HPP
