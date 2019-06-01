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

#ifndef SF_EXTRACTFITTEDPOINTS_H
#define SF_EXTRACTFITTEDPOINTS_H

#include "qsm/model/sf_modelQSM.h"
#include "sf_cloudToModelDistanceParameters.h"
#include <pcl/kdtree/kdtree_flann.h>

template<typename PointType>
class SF_ExtractFittedPoints
{
  SF_CloudToModelDistanceParameters m_params;
  std::shared_ptr<SF_ModelQSM> m_tree;
  typename pcl::PointCloud<PointType>::Ptr m_cloud;
  typename pcl::PointCloud<PointType>::Ptr m_cloudFitted;
  typename pcl::PointCloud<PointType>::Ptr m_cloudUnFitted;

public:
  SF_ExtractFittedPoints(std::shared_ptr<SF_ModelQSM> tree,
                         typename pcl::PointCloud<PointType>::Ptr cloud,
                         SF_CloudToModelDistanceParameters& params);
  void compute();
  typename pcl::PointCloud<PointType>::Ptr cloudFitted() const;
  pcl::PointCloud<PointType>::Ptr cloudUnFitted() const;
};

#include "qsm/algorithm/distance/sf_extractFittedPoints.hpp"

#endif // SF_EXTRACTFITTEDPOINTS_H
