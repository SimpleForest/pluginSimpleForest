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

//#include "sf_pca.h"
//#include <pcl/features/normal_3d.h>
//template <typename PointType, typename FeatureType>
//SF_PCA<PointType, FeatureType>::SF_PCA(typename pcl::PointCloud<PointType>::Ptr cloud_in, PointCloudSF_PCA::Ptr features_out):
//    SF_Abstract_Feature<PointType, SF_Point_PCA>(cloud_in, features_out) {

//}

//template <typename PointType, typename FeatureType>
//void SF_PCA<PointType>::set_parameters(float range, bool use_range){
//    _range = range;
//    _k =5;
//    _use_range = use_range;
//}

//template <typename PointType, typename FeatureType>
//void SF_PCA<PointType>::set_parameters(int k) {
//    _k = k;
//    _range = 0.03;
//    _use_range = false;
//}

//template <typename PointType>
//void SF_PCA<PointType>::extract_neighbors_by_range(pcl::search::KdTree<PointType>::Ptr kdtree, PointType p, pcl::PointCloud<PointType>::Ptr neighborhood) {
//    std::vector<int> pointIdxRadiusSearch;
//    std::vector<float> pointRadiusSquaredDistance;
//    if (kdtree->radiusSearch( p, _range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
//        extract_neighbors_by_index(pointIdxRadiusSearch, neighborhood);
//    }
//}

//template <typename PointType>
//void SF_PCA<PointType>::extract_neighbors_by_index(std::vector<int> pointIndex, pcl::PointCloud<PointType>::Ptr neighborhood) {
//    for(size_t j = 0; j < pointIndex.size(); j++) {
//        size_t index = pointIndex[j];
//        PointType neighbor = _cloud_in->points[index];
//        neighborhood->points.push_back(neighbor);
//    }
//}


//template <typename PointType>
//void SF_PCA<PointType>::extract_neighbors_by_knn(pcl::search::KdTree<PointType>::Ptr kdtree, PointType p, pcl::PointCloud<PointType>::Ptr neighborhood) {
//    std::vector<int> pointIdxKNNSearch;
//    std::vector<float> pointKNNSquaredDistance;
//    if (kdtree->nearestKSearch( p, _k, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 ) {
//        extract_neighbors_by_index(pointIdxKNNSearch, neighborhood);
//    }
//}

//template <typename PointType>
//void SF_PCA<PointType>::extract_neighbors(pcl::search::KdTree<PointType>::Ptr kd_tree, PointType p, pcl::PointCloud<PointType>::Ptr neighborhood) {
//    if(_use_range) {
//        extract_neighbors_by_range(kd_tree,p,neighborhood);
//    } else {
//        extract_neighbors_by_knn(kd_tree,p,neighborhood);
//    }
//}

//template <typename PointType>
//SF_Point_PCA SF_PCA<PointType>::compute_features_from_neighbors(typename pcl::PointCloud<PointType>::Ptr neighborhood,const PointType& p, const Eigen::Vector4f& xyz_centroid) {
//    Eigen::Matrix3f covariance_matrix;
//    pcl::computeCovarianceMatrix (*neighborhood, xyz_centroid, covariance_matrix);
//    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covariance_matrix);
//    SF_Point_PCA ppca(p.x,p.y,p.z);
//    ppca.lambdas = eig.eigenvalues();
//    ppca.vectors = eig.eigenvectors();
//    return ppca;
//}

//template <typename PointType>
//SF_Point_PCA SF_PCA<PointType>::compute_features_from_point(const PointType& p, pcl::search::KdTree<PointType>::Ptr kd_tree) {
//    Eigen::Vector4f xyz_centroid;
//    pcl::PointCloud<PointType>::Ptr neighborhood(new pcl::PointCloud<PointType>);
//    extract_neighbors(kd_tree,p,neighborhood);
//    pcl::compute3DCentroid (*neighborhood, xyz_centroid);
//    SF_Point_PCA ppca = compute_features_from_neighbors(neighborhood,p,xyz_centroid);
//    _features_out.points[i] = ppca;
//}


//template <typename PointType>
//void SF_PCA<PointType>::compute_features() {
//    if(_features_out.points.size() != _cloud_in.points.size()) {
//        _features_out.resize(_cloud_in.points.size());
//    }

//    pcl::search::KdTree<PointType>::Ptr kd_tree (new pcl::search::KdTree<PointType> ());
//    kd_tree->setInputCloud(_cloud_in);
//    for(size_t i = 0; i < _cloud_in->points.size(); i++) {
//        PointType p = _cloud_in->points[i];
//        compute_features_from_point(p, kd_tree);
//    }
//}



#endif // SF_PCA_HPP
