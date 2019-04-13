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

#include "sf_clustercloudbyqsm.h"

#include "steps/visualization/sf_colorfactory.h"
#include <cmath>

std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>
SF_ClusterCloudByQSM::clusters() const
{
  return m_clusters;
}

SF_ClusterCloudByQSM::SF_ClusterCloudByQSM()
{
  m_cloudUnfitted.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
  m_clustersCloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
}

void
SF_ClusterCloudByQSM::setParams(const SF_ParamSegmentTreeFromQSM<pcl::PointXYZINormal>& params)
{
  m_params = params;
  m_numClstrs = params._numClstrs;
  m_cloud = params._cloudIn;
  m_qsm = params._qsm;
  m_paramsDistance = params._distanceParams;
}

void
SF_ClusterCloudByQSM::initializeKdTree()
{
  _kdtreeQSM.reset(new pcl::KdTreeFLANN<pcl::PointXYZINormal>());
  _kdtreeQSM->setInputCloud(m_clustersCloud);
}

SF_ParamSegmentTreeFromQSM<pcl::PointXYZINormal>
SF_ClusterCloudByQSM::params() const
{
  return m_params;
}

void
SF_ClusterCloudByQSM::compute()
{
  Sf_CloudToModelDistance cmd(m_qsm, m_cloud, m_paramsDistance);
  std::vector<float> growthVolumina = cmd.distances();
  std::vector<float> growthVoluminaSorted = cmd.distances();
  std::sort(growthVoluminaSorted.begin(), growthVoluminaSorted.end());
  std::vector<float> upperGrowthVolumina;
  int sizeCluster = m_cloud->points.size() / m_numClstrs;
  int upperBorder = 0;
  for (size_t i = 0; i < m_numClstrs - 1; i++) {
    upperBorder += sizeCluster;
    upperGrowthVolumina.push_back(growthVoluminaSorted[upperBorder]);
  }
  float logMax = std::log(growthVoluminaSorted[growthVoluminaSorted.size() - 1]);
  float logMin = std::log(growthVoluminaSorted[0]);
  CT_ColorCloudStdVector* colorsLogGrowthVolume = new CT_ColorCloudStdVector(m_cloud->points.size());
  CT_ColorCloudStdVector* colorsCluster = new CT_ColorCloudStdVector(m_cloud->points.size());
  std::vector<int> ids;
  m_clusters.clear();
  for (size_t i = 0; i < m_numClstrs; i++) {
    m_clusters.push_back(pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>));
  }
  for (size_t i = 0; i < m_cloud->points.size(); i++) {
    float growthVolume = growthVolumina[i];
    float logGrowthVolume = std::log(growthVolume);
    {
      CT_Color& col = colorsLogGrowthVolume->colorAt(i);
      float perc = (logGrowthVolume - logMin) / (logMax - logMin);
      col.r() = (std::abs(255 * perc));
      col.g() = (std::abs(255 - 255 * perc));
      col.b() = (0);
    }
    {
      CT_Color& col = colorsCluster->colorAt(i);
      for (size_t j = 0; j < m_numClstrs; j++) {
        if (j == m_numClstrs - 1) {
          ids.push_back(j);
          col = SF_ColorFactory::getColor(SF_ColorFactory::Color::RED);
          m_clusters[j]->points.push_back(m_cloud->points[i]);
        }
        if (growthVolume < upperGrowthVolumina[j]) {
          ids.push_back(j);
          switch (j % 5) {
            case 0:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::GREEN);
              break;
            case 1:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::YELLOW);
              break;
            case 2:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::BLUE);
              break;
            case 3:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::CYAN);
              break;
            case 4:
              col = SF_ColorFactory::getColor(SF_ColorFactory::Color::VIOLET);
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
  m_params._outputIndices = ids;
}
