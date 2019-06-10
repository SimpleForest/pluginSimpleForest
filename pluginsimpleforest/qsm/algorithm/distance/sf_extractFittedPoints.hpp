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

#ifndef SF_EXTRACTFITTEDPOINTS_HPP
#define SF_EXTRACTFITTEDPOINTS_HPP

#include "qsm/algorithm/distance/sf_cloudToModelDistance.h"
#include "qsm/algorithm/distance/sf_extractFittedPoints.h"

#include "pcl/sf_math.h"
#include <algorithm>

template<typename PointType>
SF_ExtractFittedPoints<PointType>::SF_ExtractFittedPoints(std::shared_ptr<SF_ModelQSM> tree,
                                                          typename pcl::PointCloud<PointType>::Ptr cloud,
                                                          SF_CloudToModelDistanceParameters& params)
  : m_tree(tree), m_cloud(cloud), m_params(params)
{}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr
SF_ExtractFittedPoints<PointType>::cloudFitted() const
{
  return m_cloudFitted;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr
SF_ExtractFittedPoints<PointType>::cloudUnfitted() const
{
  return m_cloudUnfitted;
}

template<typename PointType>
void
SF_ExtractFittedPoints<PointType>::compute()
{
  m_cloudFitted.reset(new pcl::PointCloud<PointType>());
  m_cloudUnfitted.reset(new pcl::PointCloud<PointType>());
  SF_CloudToModelDistanceParameters distanceParams = m_params;
  float inlierDistance = distanceParams._inlierDistance;
  distanceParams._method = SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDER;
  Sf_CloudToModelDistance<pcl::PointXYZINormal> cmd(m_tree, m_cloud, distanceParams);
  std::vector<double> distances = cmd.distances();
  size_t index = 0;
  for (double distance : distances) {
    if (distance < inlierDistance) {
      m_cloudFitted->points.push_back(m_cloud->points[index]);
    } else {
      m_cloudUnfitted->points.push_back(m_cloud->points[index]);
    }
    ++index;
  }
}

#endif // SF_EXTRACTFITTEDPOINTS_HPP
