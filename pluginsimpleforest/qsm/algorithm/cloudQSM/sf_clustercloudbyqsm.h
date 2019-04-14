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

#ifndef SF_CLUSTERCLOUDBYQSM_H
#define SF_CLUSTERCLOUDBYQSM_H

#include "../distance/sf_cloudToModelDistance.h"
#include "../distance/sf_cloudToModelDistanceParameters.h"
#include "steps/param/sf_paramAllSteps.h"

template<typename PointType>
class SF_ClusterCloudByQSM
{
  SF_CloudToModelDistanceParameters m_paramsDistance;
  std::shared_ptr<SF_ModelQSM> m_qsm;
  typename pcl::PointCloud<PointType>::Ptr m_cloud;
  typename pcl::PointCloud<PointType>::Ptr m_clustersCloud;
  size_t m_numClstrs;
  std::vector<typename pcl::PointCloud<PointType>::Ptr> m_clusters;
  typename pcl::PointCloud<PointType>::Ptr m_cloudUnfitted;
  void initializeKdTree();
  typename pcl::KdTreeFLANN<PointType>::Ptr _kdtreeQSM;
  SF_ParamSegmentTreeFromQSM<PointType> m_params;

public:
  SF_ClusterCloudByQSM();
  void compute();
  std::vector<typename pcl::PointCloud<PointType>::Ptr> clusters() const;
  void setParams(const SF_ParamSegmentTreeFromQSM<PointType>& params);
  SF_ParamSegmentTreeFromQSM<PointType> params() const;
};

#include "qsm/algorithm/cloudQSM/sf_clustercloudbyqsm.hpp"

#endif // SF_CLUSTERCLOUDBYQSM_H
