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

class SF_ClusterCloudByQSM
{
  SF_CloudToModelDistanceParameters m_params;
  std::shared_ptr<SF_ModelQSM> m_qsm;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr m_cloud;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr m_clustersCloud;
  size_t m_numClstrs;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> m_clusters;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr m_cloudUnfitted;
  void initializeKdTree();
  pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr _kdtreeQSM;

public:
  SF_ClusterCloudByQSM(SF_CloudToModelDistanceParameters params,
                       std::shared_ptr<SF_ModelQSM> qsm,
                       pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                       int numCltrs);
  void compute();
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clusters() const;
};

#endif // SF_CLUSTERCLOUDBYQSM_H
