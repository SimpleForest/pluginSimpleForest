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
#include "sf_spherefollowingParameters.h"

#include <boost/heap/fibonacci_heap.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree.h>

#include <memory>

struct heapData;

struct Circle {
public:
  pcl::ModelCoefficients _circleCoeff;
  float _distance;
  Circle() {}
  Circle(const Circle &other) {
    _distance = other._distance;
    _circleCoeff = other._circleCoeff;
  }
  Circle(pcl::ModelCoefficients circleCoeff, float distance)
      : _circleCoeff(circleCoeff), _distance(distance) {}
};

using Heap = boost::heap::fibonacci_heap<heapData>;

struct heapData {
//  Circle _circle;
//  Heap::handle_type handle;
//  heapData(Circle point) : Circle(point), handle() {}
//  bool operator<(heapData const &second) const {
//    return _circle._distance > second._circle._distance;
//  }
};

class SF_Dijkstra {
private:
//  float _maxDistance;

//  Heap _priorityQueue;
//  std::vector<Heap::handle_type> _handle;
//  std::vector<Point> _points;
//  typename pcl::PointCloud<pcl::PointXYZI>::Ptr _cloudIn;
//  const typename pcl::PointCloud<pcl::PointXYZI>::Ptr _cloudInSeeds;
//  typename pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr _kdtree;
//  float _range;
//  std::vector<float> _distances;

//  void initialize();
//  void initializeHeap();
//  void initializeKDTree();
//  void transferIntensity();
//  int getIndex(const pcl::PointXYZI &point);
//  float getDistance(const pcl::PointXYZI &p1, const pcl::PointXYZI &p2);
//  std::vector<int> getNeighbors(const pcl::PointXYZI &point);
//  void compute();

//public:
//  SF_Dijkstra(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
//              const typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInSeeds,
//              float range);
//  std::vector<float> getDistances() const;
//  float getMaxDistance() const;
};



class SF_SphereFollowing : public SF_IDetection {
  std::shared_ptr<SF_ModelQSM> _qsm;
  SF_SphereFollowingParameters _params;
  std::vector<SF_SphereFollowingOptimizationParameters> _optimParams;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr _cloud;


public:
  SF_SphereFollowing(
      SF_SphereFollowingParameters params,
      std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clusters);
  const virtual std::shared_ptr<SF_ModelQSM> getQSM() override;
  virtual void compute() override {}
  virtual void error() override {}

private:
  void initialize();
  void initializeCloud();
  typename pcl::octree::OctreePointCloudSearch<pcl::PointXYZINormal>::Ptr _octree;
  Eigen::Vector3f _min;
  Eigen::Vector3f _max;
};

#endif // SF_SPHERE_FOLLOWING_H
