/****************************************************************************

 Copyright (C) 2017-2019, Dr. Jan Hackenberg, free software developer
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

#ifndef SF_CONVERTERCTIDTOPCLCLOUD_HPP
#define SF_CONVERTERCTIDTOPCLCLOUD_HPP

#include "converters/CT_To_PCL/sf_converterCTIDToPCLCloud.h"

template<typename PointType>
Sf_ConverterCTIDToPCLCloud<PointType>::Sf_ConverterCTIDToPCLCloud()
{}

template<typename PointType>
std::vector<typename pcl::PointCloud<PointType>::Ptr>
Sf_ConverterCTIDToPCLCloud<PointType>::clusters() const
{
  return m_clusters;
}

template<typename PointType>
void
Sf_ConverterCTIDToPCLCloud<PointType>::compute()
{
  retrieveNumberClusters();
  initializeClusters();
  writeIdsAndClusters();
  revertClusters();
}

template<typename PointType>
void
Sf_ConverterCTIDToPCLCloud<PointType>::setCloudAndID(typename pcl::PointCloud<PointType>::Ptr cloud,
                                                     CT_PointsAttributesScalarTemplated<int>* IDs)
{
  m_cloud = cloud;
  m_IDs = IDs;
}

template<typename PointType>
void
Sf_ConverterCTIDToPCLCloud<PointType>::retrieveNumberClusters()
{
  int numberClusters = 0;
  for (size_t i = 0; i < m_cloud->points.size(); i++) {
    int ID = m_IDs->valueAt(i);
    if (numberClusters < ID + 1) {
      numberClusters = ID + 1;
    }
  }
  m_numClusters = numberClusters;
}

template<typename PointType>
void
Sf_ConverterCTIDToPCLCloud<PointType>::initializeClusters()
{
  for (size_t i = 0; i < m_numClusters; i++) {
    SF_CloudNormal::Ptr cloud(new typename pcl::PointCloud<PointType>());
    m_clusters.push_back(cloud);
  }
}

template<typename PointType>
void
Sf_ConverterCTIDToPCLCloud<PointType>::writeIdsAndClusters()
{
  for (size_t i = 0; i < m_cloud->points.size(); i++) {
    int ID = m_IDs->valueAt(i);
    m_cloud->points[i].intensity = m_numClusters - 1 - ID;
    m_clusters[ID]->push_back(m_cloud->points[i]);
  }
}

template<typename PointType>
void
Sf_ConverterCTIDToPCLCloud<PointType>::revertClusters()
{
  std::reverse(m_clusters.begin(), m_clusters.end());
}

#endif // SF_CONVERTERCTIDTOPCLCLOUD_HPP
