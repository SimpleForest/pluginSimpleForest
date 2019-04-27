/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_voxelgriddownscale.hpp is part of SimpleForest - a plugin for the
    Computree platform.

    SimpleForest is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SimpleForest is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with
    SimpleForest. If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#ifndef SF_VOXELGRIDDOWNSCALE_HPP
#define SF_VOXELGRIDDOWNSCALE_HPP

#include "sf_voxelgriddownscale.h"

template<typename PointType>
SF_VoxelGridDownscale<PointType>::SF_VoxelGridDownscale()
{}

template<typename PointType>
void
SF_VoxelGridDownscale<PointType>::compute()
{
  SF_ParameterSetVoxelization<PointType> paramVoxel;
  paramVoxel.m_cloud = m_param.m_cloud;
  paramVoxel.m_voxelSize = m_param.m_voxelSize;
  SF_VoxelClustering<PointType> vc;
  vc.setParam(paramVoxel);
  vc.compute();
  std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>> clusters = vc.clusterOut();
  auto centroids = computeCentroids(clusters);
  downScale(clusters, centroids);
}

template<typename PointType>
void
SF_VoxelGridDownscale<PointType>::setParams(SF_ParameterSetVoxelgridDownscaling<PointType>& params)
{
  m_param = params;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr
SF_VoxelGridDownscale<PointType>::computeCentroids(
  const std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>>& clusters)
{
  typename pcl::PointCloud<PointType>::Ptr centroids(new pcl::PointCloud<PointType>());
  for (const auto& cluster : clusters) {
    auto cloud = cluster.first;
    pcl::CentroidPoint<PointType> centroid;
    for (const PointType& point : cloud->points) {
      centroid.add(point);
    }
    PointType center;
    centroid.get(center);
    centroids->points.push_back(std::move(center));
  }
  return centroids;
}

template<typename PointType>
void
SF_VoxelGridDownscale<PointType>::downScale(
  const std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>>& clusters,
  typename pcl::PointCloud<PointType>::Ptr centroids)
{
  typename pcl::PointCloud<PointType>::Ptr cloudOne(new pcl::PointCloud<PointType>);
  typename pcl::PointCloud<PointType>::Ptr cloudTwo(new pcl::PointCloud<PointType>);
  std::vector<size_t> CTIndicesOne;
  std::vector<size_t> CTIndicesTwo;
  size_t clusterIndex = 0;
  for (const auto& cluster : clusters) {
    auto cloud = cluster.first;
    const auto& CTindices = cluster.second;
    float closestDistance = std::numeric_limits<float>::max();
    PointType centroid = centroids->points[clusterIndex++];
    PointType closestPoint;
    size_t closestIndex;
    size_t cloudIndex = 0;
    for (const PointType& point : cloud->points) {
      float distance = pcl::squaredEuclideanDistance(point, centroid);
      if (distance < closestDistance) {
        if (closestDistance != std::numeric_limits<float>::max()) {
          cloudTwo->points.push_back(closestPoint);
          CTIndicesTwo.push_back(closestIndex);
        }
        closestDistance = distance;
        closestPoint = point;
        closestIndex = CTindices[cloudIndex++];
      } else {
        cloudTwo->points.push_back(point);
        CTIndicesTwo.push_back(CTindices[cloudIndex++]);
      }
    }
    cloudOne->points.push_back(std::move(closestPoint));
    CTIndicesOne.push_back(std::move(closestIndex));
  }
  std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>> pairOne(std::move(cloudOne), std::move(CTIndicesOne));
  std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>> pairTwo(std::move(cloudTwo), std::move(CTIndicesTwo));
  SF_AbstractBinaryFilter<PointType>::m_clusterOut.first = std::move(pairOne);
  SF_AbstractBinaryFilter<PointType>::m_clusterOut.second = std::move(pairTwo);
}

#endif // SF_VOXELGRIDDOWNSCALE_HPP
