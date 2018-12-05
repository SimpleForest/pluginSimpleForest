/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_clustertransfer.hpp is part of SimpleForest - a plugin for the
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

#ifndef SF_CLUSTERTRANSFER_HPP
#define SF_CLUSTERTRANSFER_HPP

#include "sf_clustertransfer.h"

template <typename PointType> void SF_ClusterTransfer::compute() {
  initialize();
  pcl::PointCloud<PointType>::Ptr cloud = m_cloudIn.first;
  std::vector<size_t> indicesCT = m_cloudIn.second;
  size_t indexCTVec = 0;
  std::for_each(cloud->points.begin(), cloud->points.end(),
                [this, &indicesCT](PointType point) {
                  size_t indexCT = indicesCT[indexCTVec++];
                  std::vector<int> knnSearchIndex(1);
                  std::vector<float> knnSquaredDistance(1);
                  if (m_kdtree->nearestKSearch(point, 1, knnSearchIndex,
                                               knnSquaredDistance)) {
                    PointType closestPoint = m_cloudInMerged[knnSearchIndex[0]];
                    point.x = closestPoint.x;
                    point.y = closestPoint.y;
                    point.z = closestPoint.z;
                    size_t indexCluster = m_ClusterIndices[knnSearchIndex[0]];
                    m_clusterOut[indexCluster].first.push_back(point);
                    m_clusterOut[indexCluster].second.push_back(indexCT);
                  }
                });
}

template <typename PointType> void SF_ClusterTransfer::initialize() {
  m_cloudInMerged.reset(new pcl::PointCloud<PointType>());
  size_t clusterIndex = 0;
  for (const auto &pair : m_clusterIn) {
    pcl::PointCloud<PointType>::Ptr cluster = pair.first;
    std::for_each(cluster->points.begin(), cluster->points.end(),
                  [this](const PointType &point) {
                    m_cloudInMerged->push_back(PointType(point));
                  });
    std::vector<size_t> indicesCTCluster = pair.second;
    std::for_each(indicesCTCluster.begin(), indicesCTCluster.end(),
                  [this, clusterIndex](size_t index) {
                    m_ClusterIndices.push_back(clusterIndex);
                  });
    pcl::PointCloud<PointType>::Ptr clusterOut(
        new pcl::PointCloud<PointType>());
    std::vector<size_t> indicesCTOut;
    m_clusterOut.push_back(
        std::pair<pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>(
            clusterOut, indicesCTOut));
    clusterIndex++;
  }
  m_kdtree.reset(new pcl::KdTreeFLANN<PointType>());
  m_kdtree->setInputCloud(m_cloudInMerged);
}

#endif // SF_CLUSTERTRANSFER_HPP
