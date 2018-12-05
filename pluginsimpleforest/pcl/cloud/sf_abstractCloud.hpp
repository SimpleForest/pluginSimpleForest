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

#include <pcl/cloud/sf_abstractCloud.h>
#include <pcl/filters/voxel_grid.h>

template <typename PointType>
SF_AbstractCloud<PointType>::SF_AbstractCloud() {

}

template <typename PointType>
SF_AbstractCloud<PointType>::SF_AbstractCloud(const typename pcl::PointCloud<PointType>::Ptr cloudIn):
_cloudIn(cloudIn) {

}

template <typename PointType>
bool SF_AbstractCloud<PointType>::equalsBySqrtDistance(float sqrtDistance) {
    if(sqrtDistance <_MIN_SQUARED_DISTANCE)
        return true;
    return false;
}

template<typename PointType>
void SF_AbstractCloud<PointType>::searchKdTree(size_t index,
                                                 pcl::KdTreeFLANN<PointType> & kdtree) {
    PointType point = SF_AbstractCloud<PointType>::_cloudIn->at(index);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    kdtree.nearestKSearch (point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    createIndex(point, pointNKNSquaredDistance[0]);
}

template<typename PointType>
void SF_AbstractCloud<PointType>::iterateOverCloud(pcl::KdTreeFLANN<PointType> &kdtree) {
    size_t size = SF_AbstractCloud<PointType>::_cloudIn->size();
    for(size_t i = 0; i < size; i++) {
        searchKdTree(i,kdtree);
    }
}

template <typename PointType>
std::vector<int> SF_AbstractCloud<PointType>::getIndices() const {
   return _indices;
}

template <typename PointType>
void SF_AbstractCloud<PointType>::setCloudIn(const typename pcl::PointCloud<PointType>::Ptr &cloud_in) {
    _cloudIn = cloud_in;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr SF_AbstractCloud<PointType>::downScale(float voxelSize) {
    typename pcl::PointCloud<PointType>::Ptr downscaled_cloud(new typename pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> sor;
    std::cout<<"foo2s _cloudIn " << _cloudIn->points.size() <<std::endl;
    sor.setInputCloud (_cloudIn);
    sor.setLeafSize (voxelSize, voxelSize, voxelSize);
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
void SF_AbstractCloud<PointType>::extractNeighborhoodByIndexList(std::vector<int> pointIndex,
                                                                 typename pcl::PointCloud<PointType>::Ptr neighborhood) {
    for(size_t j = 0; j < pointIndex.size(); j++) {
        size_t index = pointIndex[j];
        PointType neighbor = SF_AbstractCloud<PointType>::_cloudIn->points[index];
        neighborhood->points.push_back(neighbor);
    }
}

template <typename PointType>
void SF_AbstractCloud<PointType>::extractNeighborsByRange(typename pcl::KdTree<PointType>::Ptr kdtree,
                                                          PointType &p,
                                                          typename pcl::PointCloud<PointType>::Ptr neighborhood,
                                                          float range) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (kdtree->radiusSearch( p, range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
        extractNeighborhoodByIndexList(pointIdxRadiusSearch, neighborhood);
    }
}


template <typename PointType>
void SF_AbstractCloud<PointType>::extractNeighborsByKnn(typename pcl::KdTree<PointType>::Ptr kdtree,
                                                        PointType &p,
                                                        typename pcl::PointCloud<PointType>::Ptr neighborhood,
                                                        int k) {
    std::vector<int> pointIdxKNNSearch;
    std::vector<float> pointKNNSquaredDistance;
    if (kdtree->nearestKSearch( p, k, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 ) {
        extractNeighborhoodByIndexList(pointIdxKNNSearch, neighborhood);
    }
}




#endif // SF_ABSTRACT_CLOUD_HPP
