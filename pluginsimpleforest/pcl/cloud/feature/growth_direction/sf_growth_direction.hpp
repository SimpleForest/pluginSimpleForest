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
#include "sf_growth_direction.h"

template <typename PointType, typename FeatureType>
SF_Growth_Direction<PointType, FeatureType>::SF_Growth_Direction(typename pcl::PointCloud<PointType>::Ptr cloud_in, typename pcl::PointCloud<FeatureType> ::Ptr features_out):
    SF_Growth_Direction::_cloud_in(cloud_in), SF_Growth_Direction::_features_out(features_out) {

}

template <typename PointType, typename FeatureType>
void SF_Growth_Direction<PointType, FeatureType>::set_parameters(float range_normal, float range_gd) {
    _range_normal = range_normal;
    _range_gd     = range_gd;
}

template <typename PointType, typename FeatureType>
void SF_Growth_Direction<PointType, FeatureType>::compute_features_range() {
    pcl::NormalEstimation<PointType, FeatureType> ne;
    ne.setInputCloud (SF_Growth_Direction::_cloud_in);
    typename pcl::search::KdTree<PointType>::Ptr tree (new typename pcl::search::KdTree<PointType> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (_range_normal);
    ne.compute (*SF_Growth_Direction<PointType, FeatureType>::_features);
}

template <typename PointType, typename FeatureType>
void compute_pca_from_point(PointType & p, typename pcl::search::KdTree<PointType>::Ptr kd_tree) {
}

template<typename PointType, typename FeatureType>
std::vector<PCA_Values> SF_Growth_Direction::compute_normal_pca() {
    SF_PCA<PointType> sfpca(_cloud_in);
    sfpca.set_parameters(_range_normal, false, true);
    sfpca.compute_features();
    return sfpca.get_pca_values();
}

template<typename PointType, typename FeatureType>
void SF_Growth_Direction::add_normals(std::vector<PCA_Values> &values) {
    for(size_t i = 0; i < values.size(); i++) {
        PCA_Values val = values[i];
        _cloud_in[i].normal_x = val.getVector3()[0];
        _cloud_in[i].normal_y = val.getVector3()[1];
        _cloud_in[i].normal_z = val.getVector3()[2];
    }
}

template<typename PointType, typename FeatureType>
void SF_Growth_Direction::add_growth_direction(std::vector<PCA_Values> &values) {
    for(size_t i = 0; i < pca_values.size(); i++) {
        typename FeatureType point_gd;
        point_gd.x = _cloud_in[i].x;
        point_gd.y = _cloud_in[i].y;
        point_gd.z = _cloud_in[i].z;
        PCA_Values val = values[i];
        if (val.getLambda1() > MAX_LAMBDA3) {
            point_gd.normal_x = val.getVector1()[0];
            point_gd.normal_y = val.getVector1()[1];
            point_gd.normal_z = val.getVector1()[2];
        } else {
            typename PointType p = _cloud_in[i];
            typename pcl::PointCloud<PointType>::Ptr neighborhood;
            extract_neighbors_by_range(kdtree,p,neighborhood, _range_gd);
            for(size_t j = 0; j < neighborhood->points.size(); j++) {
                neighborhood[j].x = neighborhood[j].normal_x;
                neighborhood[j].y = neighborhood[j].normal_y;
                neighborhood[j].z = neighborhood[j].normal_z;
            }
            PCA_Values val_gd = compute_features_from_neighbors(neighborhood, Eigen::Vector4f(0,0,0));
            point_gd.normal_x = val_gd.getVector3()[0];
            point_gd.normal_y = val_gd.getVector3()[1];
            point_gd.normal_z = val_gd.getVector3()[2];
        }
        _features_out->points[i] = point_gd;
    }
}

template <typename PointType, typename FeatureType>
void SF_Growth_Direction<PointType, FeatureType>::compute_features() {
    if(SF_Growth_Direction::_features_out.points.size() != SF_Growth_Direction::_cloud_in.points.size()) {
        SF_Growth_Direction::_features_out.resize(SF_Growth_Direction::_cloud_in.points.size());
    }
    typename pcl::KdTree<PointType>::Ptr kd_tree (new pcl::KdTree<PointType> ());
    kd_tree->setInputCloud(SF_Growth_Direction::_cloud_in);

    std::vector<PCA_Values> pca_values = compute_normal_pca();
    add_normals(pca_values);
    add_growth_direction(pca_values);
}

#endif // SF_GROWTH_DIRECTION_HPP
