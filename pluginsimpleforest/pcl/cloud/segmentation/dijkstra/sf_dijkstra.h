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
#ifndef SF_DIJKSTRA_H
#define SF_DIJKSTRA_H

#include <boost/heap/fibonacci_heap.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/sf_point.h>
#include <steps/param/sf_paramAllSteps.h>

struct heapData;

struct Point
{
public:
  pcl::PointXYZI _point;
  float _distance;
  bool _visited;
  Point() {}
  Point(const Point& p2)
  {
    _distance = p2._distance;
    _visited = p2._visited;
    _point = p2._point;
  }
  Point(pcl::PointXYZI point, float distance) : _point(point), _distance(distance) {}
};

using Heap = boost::heap::fibonacci_heap<heapData>;

struct heapData
{
  Point _point;
  Heap::handle_type handle;
  heapData(Point point) : _point(point), handle() {}
  bool operator<(heapData const& second) const { return _point._distance > second._point._distance; }
};

class SF_Dijkstra
{
private:
  float _maxDistance;
  bool m_useFixDistance = false;

  Heap _priorityQueue;
  std::vector<Heap::handle_type> _handle;
  std::vector<Point> _points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _cloudIn;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _cloudInSeeds;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr _kdtree;
  float _range;
  std::vector<float> _distances;
  std::vector<int> _parentIndices;

  void initialize();
  void initializeHeap();
  void initializeKDTree();
  void transferIntensity();
  int getIndex(const pcl::PointXYZI& point);
  float getDistance(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2);
  std::vector<int> getNeighbors(const pcl::PointXYZI& point);
  void compute();

public:
  SF_Dijkstra(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudIn,
              pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudInSeeds,
              float range,
              bool useFixedDistance = false);
  SF_Dijkstra(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
              pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInSeeds,
              float range,
              bool useFixedDistance = false);
  std::vector<float> getDistances() const;
  float getMaxDistance() const;
  std::vector<int> getParentIndices() const;
};

#endif // SF_DIJKSTRA_H
