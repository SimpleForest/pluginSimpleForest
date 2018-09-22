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
#ifndef SF_PCA_HPP
#define SF_PCA_HPP

#include "sf_pca.h"
template <typename PointType>
SF_PCA<PointType>::SF_PCA(typename pcl::PointCloud<PointType>::Ptr cloud_in):
    SF_AbstractCloud<PointType>(cloud_in) {
    pca_values.resize(cloud_in->points.size());
}

template <typename PointType>
void SF_PCA<PointType>::set_parameters(float range, bool center_zero, bool use_range){
    _range = range;
    _center_zero = center_zero;
    _k =5;
    _use_range = use_range;
}

template <typename PointType>
void SF_PCA<PointType>::set_parameters(int k, bool center_zero) {
    _k = k;
    _center_zero = center_zero;
    _k = 0.03;
    _use_range = false;
}

template <typename PointType>
void SF_PCA<PointType>::extract_neighbors(typename pcl::KdTree<PointType>::Ptr kd_tree, PointType p, typename pcl::PointCloud<PointType>::Ptr neighborhood) {
    if(_use_range) {
        extractNeighborsByRange(kd_tree,p,neighborhood, _range);
    } else {
        extractNeighborsByKnn(kd_tree,p,neighborhood, _k);
    }
}

template <typename PointType>
PCA_Values SF_PCA<PointType>::compute_features_from_neighbors(typename pcl::PointCloud<PointType>::Ptr neighborhood, const Eigen::Vector4f& xyz_centroid) {
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrix (*neighborhood, xyz_centroid, covariance_matrix);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covariance_matrix);
    PCA_Values ppca;
    ppca.lambdas = eig.eigenvalues();
    ppca.vectors = eig.eigenvectors();
    return ppca;
}

template <typename PointType>
void SF_PCA<PointType>::compute_features_for_point(const PointType& p, typename pcl::KdTree<PointType>::Ptr kd_tree, int index) {
    Eigen::Vector4f xyz_centroid;
    typename pcl::PointCloud<PointType>::Ptr neighborhood(new typename pcl::PointCloud<PointType>);
    extract_neighbors(kd_tree,p,neighborhood);
    pcl::compute3DCentroid (*neighborhood, xyz_centroid);
    Eigen::Vector4f origin(0,0,0,1) ;
    PCA_Values ppca = compute_features_from_neighbors(neighborhood, (_center_zero ?  origin : xyz_centroid) );
    pca_values[index] = ppca;
}


template <typename PointType>
void SF_PCA<PointType>::compute_features() {
    if(pca_values.size() != SF_PCA<PointType>::_cloudIn->points.size()) {
        pca_values.resize(SF_PCA<PointType>::_cloudIn->points.size());
    }

    typename pcl::KdTreeFLANN<PointType>::Ptr kd_tree (new typename pcl::KdTreeFLANN<PointType> ());
    kd_tree->setInputCloud(SF_PCA<PointType>::_cloudIn);
    for(size_t i = 0; i < SF_PCA<PointType>::_cloudIn->points.size(); i++) {
        PointType p = SF_PCA<PointType>::_cloudIn->points[i];
        compute_features_for_point(p, kd_tree, i);
    }
}

template <typename PointType>
std::vector<PCA_Values> SF_PCA<PointType>::get_pca_values() const {
    return pca_values;
}



#endif // SF_PCA_HPP
