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

#ifndef SF_SPHERE_FOLLOWING_H
#define SF_SPHERE_FOLLOWING_H

#include "qsm/algorithm/detection/sf_idetection.h"
#include "qsm/algorithm/sf_QSMCylinder.h"
#include "sf_spherefollowingParameters.h"
#include "steps/param/sf_paramAllSteps.h"

#include <boost/heap/fibonacci_heap.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree.h>

#include <memory>

struct Circle
{
public:
  pcl::ModelCoefficients m_circleCoeff;
  float m_distance;
  int m_clusterIndex;
  Eigen::Vector3f m_firstSplit;
  Circle() {}
  Circle(const Circle& other)
  {
    m_distance = other.m_distance;
    m_circleCoeff = other.m_circleCoeff;
    m_clusterIndex = other.m_clusterIndex;
    m_firstSplit = other.m_firstSplit;
  }
  Circle(pcl::ModelCoefficients circleCoeff, float distance, int clusterIndex, Eigen::Vector3f& firstSplit)
    : m_circleCoeff(circleCoeff), m_distance(distance), m_clusterIndex(clusterIndex), m_firstSplit(firstSplit)
  {}
};

class SF_SphereFollowing : public SF_IDetection
{
public:
  SF_SphereFollowing();
  std::shared_ptr<SF_ModelQSM> getQSM() override;
  void compute() override;
  float error() override;
  void setParams(const SF_ParamSpherefollowingBasic<pcl::PointXYZINormal>& params);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud() const;
  void setCloud(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud);

private:
  float getDistance(const pcl::ModelCoefficients& circle, const Circle& lastCircle);
  Eigen::Vector3f getCentroid(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);
  typename pcl::octree::OctreePointCloudSearch<pcl::PointXYZINormal>::Ptr m_octree;
  std::vector<SF_QSMDetectionCylinder> m_cylinders;
  SF_ParamSpherefollowingBasic<pcl::PointXYZINormal> m_params;
  std::shared_ptr<SF_ModelQSM> m_qsm;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr m_cloud;
  std::map<float, Circle> m_map;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr lowestSlice(float& minZ);
  pcl::PointIndices::Ptr surfaceIndices(Circle& lastCircle);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr extractCloud(pcl::PointIndices::Ptr indices);
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clusterEuclidean(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                                                                           size_t minIndex);
  void processClusters(std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>& clusters, const Circle& lastCircle);
  void initialize();
  void initializeOctree();
  void initializeHeap();
  void pushbackQueue(pcl::ModelCoefficients circleCoeff, float distance, int clusterID, Eigen::Vector3f firstSplit);
  void artificialTree();
  void buildTree();
  bool failedComputation = false;
};

#endif // SF_SPHERE_FOLLOWING_H
