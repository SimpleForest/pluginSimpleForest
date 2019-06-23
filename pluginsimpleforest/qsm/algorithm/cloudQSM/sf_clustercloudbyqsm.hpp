/****************************************************************************

 Copyright (C) 2017-2019 Dr. Jan Hackenberg, free software developer
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

#include "qsm/algorithm/cloudQSM/sf_clustercloudbyqsm.h"

#include "pcl/cloud/segmentation/dijkstra/sf_dijkstra.h"
#include "steps/visualization/sf_colorfactory.h"
#include <cmath>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

template<typename PointType>
SF_ClusterCloudByQSM<PointType>::SF_ClusterCloudByQSM()
{
  m_cloudUnfitted.reset(new typename pcl::PointCloud<PointType>());
  m_clustersCloud.reset(new typename pcl::PointCloud<PointType>());
}

template<typename PointType>
void
SF_ClusterCloudByQSM<PointType>::setParams(const SF_ParamSegmentTreeFromQSM<PointType>& params)
{
  m_params = params;
  m_numClstrs = params._numClstrs;
  m_cloud = params._cloudIn;
  m_qsm = params._qsm;
  m_paramsDistance = params._distanceParams;
}

template<typename PointType>
void
SF_ClusterCloudByQSM<PointType>::initializeKdTree()
{
  _kdtreeQSM.reset(new typename pcl::KdTreeFLANN<PointType>());
  _kdtreeQSM->setInputCloud(m_clustersCloud);
}

template<typename PointType>
SF_ParamSegmentTreeFromQSM<PointType>
SF_ClusterCloudByQSM<PointType>::params() const
{
  return m_params;
}

template<typename PointType>
std::vector<double>
SF_ClusterCloudByQSM<PointType>::filteredDistances(const std::vector<double>& distances, const std::vector<int>& parentIndices)
{
  std::vector<double> distancesFiltered = distances;
  std::vector<bool> isLeave(parentIndices.size(), true);
  for (const auto index : parentIndices) {
    if (index > -1) {
      isLeave[index] = false;
    }
  }
  std::vector<int> leaveIndices;
  for (int i = 0; i < isLeave.size(); i++) {
    if (isLeave[i]) {
      leaveIndices.push_back(i);
    }
  }
  for (int leave : leaveIndices) {
    int currentIndex = leave;
    double minDistance = distancesFiltered[leave];
    while (currentIndex != -1) {
      if (distances[currentIndex] > minDistance) {
        distancesFiltered[currentIndex] = minDistance;
      }
      minDistance = distancesFiltered[currentIndex];
      currentIndex = parentIndices[currentIndex];
    }
  }
  return distancesFiltered;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr
SF_ClusterCloudByQSM<PointType>::seedCloud()
{
  PointType min;
  float minZ = std::numeric_limits<float>::max();
  for (PointType point : m_cloud->points) {
    if (minZ > point.z) {
      minZ = point.z;
      min = point;
    }
  }
  typename pcl::PointCloud<PointType>::Ptr seed(new typename pcl::PointCloud<PointType>);
  seed->push_back(min);
  return seed;
}

template<typename PointType>
void
SF_ClusterCloudByQSM<PointType>::compute()
{
  CT_StandardCloudStdVectorT<int>* clusterID = new CT_StandardCloudStdVectorT<int>(m_cloud->points.size());

  auto cloudCpy = m_cloud;
  try {
    typename pcl::PointCloud<PointType>::Ptr cloudDownscaled(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(m_cloud);
    {
      sor.setLeafSize(0.02, 0.02, 0.02);
    }
    sor.filter(*cloudDownscaled);
    m_cloud = cloudDownscaled;

    Sf_CloudToModelDistance<PointType> cmd(m_qsm, m_cloud, m_paramsDistance);
    std::vector<double> growthVolumina = cmd.distances();

    try {
      typename pcl::PointCloud<PointType>::Ptr seed = seedCloud();
      SF_Dijkstra dijkstra(m_cloud, seed, 0.03);
      std::vector<int> parentIndices = dijkstra.getParentIndices();
      growthVolumina = filteredDistances(growthVolumina, parentIndices);
    } catch (...) {
      std::cout << "DIJKSTRA FAILED" << std::endl;
      std::cout << "growthVolumina size " << growthVolumina.size() << std::endl;
    }

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloudDownscaled);
    std::vector<double> growthVoluminaTransfered;
    for (auto searchPoint : cloudCpy->points) {
      int K = 1;
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);
      if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        growthVoluminaTransfered.push_back(growthVolumina[pointIdxNKNSearch[0]]);
      } else {
        growthVoluminaTransfered.push_back(0.001);
      }
    }
    m_cloud = cloudCpy;
    growthVolumina = growthVoluminaTransfered;
    std::vector<double> growthVoluminaSorted = growthVolumina;
    std::sort(growthVoluminaSorted.begin(), growthVoluminaSorted.end());
    std::vector<double> upperGrowthVolumina;
    size_t sizeCluster = m_cloud->points.size() / m_numClstrs;
    size_t upperBorder = 0;
    upperGrowthVolumina.push_back(growthVoluminaSorted[upperBorder]);
    for (size_t i = 0; i < m_numClstrs - 1; i++) {
      upperBorder += sizeCluster;
      upperBorder = std::min(upperBorder, std::max(static_cast<size_t>(1), growthVoluminaSorted.size()) - 1);
      upperGrowthVolumina.push_back(growthVoluminaSorted[upperBorder]);
    }
    for (size_t i = 0; i < m_cloud->points.size(); i++) {
      double growthVolume = growthVolumina[i];
      {
        for (size_t j = m_numClstrs - 1; j >= 0; j--) {
          if (growthVolume >= upperGrowthVolumina[j]) {
            switch (j) {
              case 0:
                (*clusterID)[i] = m_numClstrs - 1 - 0;
                break;
              case 1:
                (*clusterID)[i] = m_numClstrs - 1 - 1;
                break;
              case 2:
                (*clusterID)[i] = m_numClstrs - 1 - 2;
                break;
              case 3:
                (*clusterID)[i] = m_numClstrs - 1 - 3;
                break;
              case 4:
                (*clusterID)[i] = m_numClstrs - 1 - 4;
                break;
              case 5:
                (*clusterID)[i] = m_numClstrs - 1 - 5;
                break;
              default:
                break;
            }
            break;
          }
        }
      }
    }
  } catch (...) {
    m_cloud = cloudCpy;
    for (size_t i = 0; i < m_cloud->points.size(); i++) {
      (*clusterID)[i] = 0;
    }
  }
  m_params._clusterIDs = clusterID;
}
