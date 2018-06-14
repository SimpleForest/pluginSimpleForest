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
#ifndef SF_ABSTRACT_CLOUD_HPP
#define SF_ABSTRACT_CLOUD_HPP

#include <pcl/cloud/sf_abstract_cloud.h>
#include <pcl/filters/voxel_grid.h>

template <typename PointType>
SF_Abstract_Cloud<PointType>::SF_Abstract_Cloud() {

}

template <typename PointType>
SF_Abstract_Cloud<PointType>::SF_Abstract_Cloud(const typename pcl::PointCloud<PointType>::Ptr cloud_in) {
    _cloud_in= cloud_in;

}

template <typename PointType>
bool SF_Abstract_Cloud<PointType>::equals_by_sqrt_distance(float sqrt_distance) {
    if(sqrt_distance <_MIN_SQUARED_DISTANCE)
        return true;
    return false;
}

template<typename PointType>
void SF_Abstract_Cloud<PointType>::search_kd_tree(size_t index,
                                                 pcl::KdTreeFLANN<PointType> & kdtree) {
    PointType point = SF_Abstract_Cloud<PointType>::_cloud_in->at(index);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    kdtree.nearestKSearch (point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    create_index(point, pointNKNSquaredDistance[0]);
}

template<typename PointType>
void SF_Abstract_Cloud<PointType>::iterate_over_cloud(pcl::KdTreeFLANN<PointType> &kdtree) {
    size_t size = SF_Abstract_Cloud<PointType>::_cloud_in->size();
    for(size_t i = 0; i < size; i++) {
        search_kd_tree(i,kdtree);
    }
}

template <typename PointType>
std::vector<int> SF_Abstract_Cloud<PointType>::get_indices() const {
   return _indices;
}

template <typename PointType>
void SF_Abstract_Cloud<PointType>::set_cloud_in(const typename pcl::PointCloud<PointType>::Ptr &cloud_in) {
    _cloud_in = cloud_in;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr SF_Abstract_Cloud<PointType>::down_scale(float voxel_size) {
    typename pcl::PointCloud<PointType>::Ptr downscaled_cloud(new typename pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud (_cloud_in);
    sor.setLeafSize (voxel_size, voxel_size, voxel_size);
    sor.filter (*downscaled_cloud);
    downscaled_cloud->width = downscaled_cloud->points.size();
    downscaled_cloud->height = 1;
    typename pcl::PointCloud<PointType>::Ptr downscaled_cloud2(new typename pcl::PointCloud<PointType>);
    downscaled_cloud->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*downscaled_cloud, *downscaled_cloud2, indices);
    downscaled_cloud2->width = downscaled_cloud2->points.size();
    downscaled_cloud2->height = 1;
    downscaled_cloud2->is_dense = true;
    return downscaled_cloud2;
}


template <typename PointType>
void SF_Abstract_Cloud<PointType>::extract_neighborhood_by_index_list(std::vector<int> pointIndex, typename pcl::PointCloud<PointType>::Ptr neighborhood) {
    for(size_t j = 0; j < pointIndex.size(); j++) {
        size_t index = pointIndex[j];
        PointType neighbor = SF_Abstract_Cloud<PointType>::_cloud_in->points[index];
        neighborhood->points.push_back(neighbor);
    }
    neighborhood->width = pointIndex.size();
    neighborhood->height = 1;
    neighborhood->is_dense = true;
}

template <typename PointType>
void SF_Abstract_Cloud<PointType>::extract_neighbors_by_range(typename pcl::KdTree<PointType>::Ptr kdtree, PointType p, typename pcl::PointCloud<PointType>::Ptr neighborhood, float range) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (kdtree->radiusSearch( p, range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
        extract_neighborhood_by_index_list(pointIdxRadiusSearch, neighborhood);
    }
}


template <typename PointType>
void SF_Abstract_Cloud<PointType>::extract_neighbors_by_knn(typename pcl::KdTree<PointType>::Ptr kdtree, PointType p, typename pcl::PointCloud<PointType>::Ptr neighborhood, int k) {
    std::vector<int> pointIdxKNNSearch;
    std::vector<float> pointKNNSquaredDistance;
    if (kdtree->nearestKSearch( p, k, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 ) {
        extract_neighborhood_by_index_list(pointIdxKNNSearch, neighborhood);
    }
}




#endif // SF_ABSTRACT_CLOUD_HPP
