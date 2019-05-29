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

#include "steps/visualization/sf_colorfactory.h"
#include <cmath>

template<typename PointType>
std::vector<typename pcl::PointCloud<PointType>::Ptr>
SF_ClusterCloudByQSM<PointType>::clusters() const
{
  return m_clusters;
}

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
void
SF_ClusterCloudByQSM<PointType>::compute()
{
  Sf_CloudToModelDistance<PointType> cmd(m_qsm, m_cloud, m_paramsDistance);
  std::vector<float> growthVolumina = cmd.distances();
  std::vector<float> growthVoluminaSorted = cmd.distances();
  std::sort(growthVoluminaSorted.begin(), growthVoluminaSorted.end());
  std::vector<float> upperGrowthVolumina;
  size_t sizeCluster = m_cloud->points.size() / m_numClstrs;
  size_t upperBorder = 0;
  upperGrowthVolumina.push_back(growthVoluminaSorted[upperBorder]);
  for (size_t i = 0; i < m_numClstrs - 1; i++) {
    upperBorder += sizeCluster;
    upperBorder = std::min(upperBorder, std::max(static_cast<size_t>(1), growthVoluminaSorted.size()) - 1);
    upperGrowthVolumina.push_back(growthVoluminaSorted[upperBorder]);
  }
  float logMin = std::log(-growthVoluminaSorted[growthVoluminaSorted.size() - 1]);
  float logMax = std::log(-growthVoluminaSorted[0]);
  CT_ColorCloudStdVector* colorsLogGrowthVolume = new CT_ColorCloudStdVector(m_cloud->points.size());
  CT_ColorCloudStdVector* colorsCluster = new CT_ColorCloudStdVector(m_cloud->points.size());
  CT_StandardCloudStdVectorT<int>* clusterID = new CT_StandardCloudStdVectorT<int>(m_cloud->points.size());
  m_clusters.clear();
  for (size_t i = 0; i < m_numClstrs; i++) {
    m_clusters.push_back(typename pcl::PointCloud<PointType>::Ptr(new typename pcl::PointCloud<PointType>));
  }
  for (size_t i = 0; i < m_cloud->points.size(); i++) {
    float growthVolume = growthVolumina[i];
    float logGrowthVolume = std::log(-growthVolume);
    {
      CT_Color& col = colorsLogGrowthVolume->colorAt(i);
      float perc = (logGrowthVolume - logMin) / (logMax - logMin);
      col.r() = (std::abs(255 * perc));
      col.g() = (std::abs(255 - 255 * perc));
      col.b() = (0);
    }
    {
      CT_Color& col = colorsCluster->colorAt(i);
      for (size_t j = m_numClstrs - 1; j >= 0; j--) {
        if (growthVolume >= upperGrowthVolumina[j]) {
          switch (j) {
            case 0:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::GREEN);
              (*clusterID)[i] = m_numClstrs - 1 - 0;
              break;
            case 1:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::YELLOW);
              (*clusterID)[i] = m_numClstrs - 1 - 1;
              break;
            case 2:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::BLUE);
              (*clusterID)[i] = m_numClstrs - 1 - 2;
              break;
            case 3:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::CYAN);
              (*clusterID)[i] = m_numClstrs - 1 - 3;
              break;
            case 4:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::VIOLET);
              (*clusterID)[i] = m_numClstrs - 1 - 4;
              break;
            case 5:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::RED);
              (*clusterID)[i] = m_numClstrs - 1 - 5;
              break;
            default:
              break;
          }
          m_clusters[j]->points.push_back(m_cloud->points[i]);
          break;
        }
      }
    }
  }
  m_params._colorsClusters = colorsCluster;
  m_params._colorsGrowthVolume = colorsLogGrowthVolume;
  m_params._clusterIDs = clusterID;
}
