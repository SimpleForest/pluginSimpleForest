#ifndef SF_DIJKSTRA_HPP
#define SF_DIJKSTRA_HPP

#include "sf_dijkstra.h"

#include <pcl/search/kdtree.h>

SF_Dijkstra::SF_Dijkstra(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInSeeds, float range)
  : _cloudIn(cloudIn), _cloudInSeeds(cloudInSeeds), _range(range)
{
  initialize();
  compute();
}

SF_Dijkstra::SF_Dijkstra(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudIn,
                         pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudInSeeds,
                         float range)
{
  _range = range;
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCpy(new pcl::PointCloud<pcl::PointXYZI>);
    for (pcl::PointXYZINormal point : cloudInSeeds->points) {
      pcl::PointXYZI pCpy;
      pCpy.x = point.x;
      pCpy.y = point.y;
      pCpy.z = point.z;
      pCpy.intensity = point.intensity;
      cloudInCpy->push_back(pCpy);
    }
    _cloudInSeeds = cloudInCpy;
  }
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCpy(new pcl::PointCloud<pcl::PointXYZI>);
    for (pcl::PointXYZINormal point : cloudIn->points) {
      pcl::PointXYZI pCpy;
      pCpy.x = point.x;
      pCpy.y = point.y;
      pCpy.z = point.z;
      pCpy.intensity = point.intensity;
      cloudInCpy->push_back(pCpy);
    }
    _cloudIn = cloudInCpy;
  }
  initialize();
  compute();
}

std::vector<float>
SF_Dijkstra::getDistances() const
{
  return _distances;
}

void
SF_Dijkstra::transferIntensity()
{
  pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
  kdtree->setInputCloud(_cloudInSeeds);
  size_t size = _cloudIn->points.size();
  float sqrdRange = _range * _range * 0.1f;
  for (size_t i = 0; i < size; i++) {
    pcl::PointXYZI point = _cloudIn->points[i];
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    if (kdtree->nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      if (pointNKNSquaredDistance[0] < sqrdRange) {
        int index = _cloudInSeeds->points[pointIdxNKNSearch[0]].intensity;
        _cloudIn->points[i].intensity = index;
        _distances.push_back(0);
      } else {
        _cloudIn->points[i].intensity = -1;
        _distances.push_back(1000);
      }
    } else {
      _cloudIn->points[i].intensity = -1;
      _distances.push_back(1000);
    }
  }
}

void
SF_Dijkstra::initializeHeap()
{
  _priorityQueue.clear();
  size_t size = _cloudIn->points.size();
  _points.resize(size);
  _handle.clear();
  for (size_t i = 0; i < size; i++) {
    Point point;
    point._point = _cloudIn->points[i];
    point._distance = _distances[i];
    point._visited = false;
    _points[i] = point;
    Heap::handle_type h = _priorityQueue.push(point);
    _handle.push_back(h);
    (*h).handle = h;
  }
}

void
SF_Dijkstra::initializeKDTree()
{
  _kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  _kdtree->setInputCloud(_cloudIn);
}

void
SF_Dijkstra::initialize()
{
  _parentIndices.resize(_cloudIn->points.size());
  std::fill(_parentIndices.begin(), _parentIndices.end(), -1);
  transferIntensity();
  initializeHeap();
  initializeKDTree();
}

int
SF_Dijkstra::getIndex(const pcl::PointXYZI& point)
{
  std::vector<int> indices(1);
  std::vector<float> sqrtDistances(1);
  _kdtree->nearestKSearch(point, 1, indices, sqrtDistances);
  return indices[0];
}

std::vector<int>
SF_Dijkstra::getNeighbors(const pcl::PointXYZI& point)
{
  std::vector<int> indices;
  std::vector<float> sqrtDistances;
  std::vector<int> result;
  _kdtree->radiusSearch(point, _range, indices, sqrtDistances);
  for (int i = 0; i < indices.size(); i++) {
    int index = indices[i];
    if (!_points[index]._visited) {
      result.push_back(index);
    }
  }
  return result;
}

std::vector<int>
SF_Dijkstra::getParentIndices() const
{
  return _parentIndices;
}

float
SF_Dijkstra::getDistance(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2)
{
  float dx, dy, dz;
  dx = p1.x - p2.x;
  dy = p1.y - p2.y;
  dz = p1.z - p2.z;
  return (std::sqrt(dx * dx + dy * dy + dz * dz));
}

void
SF_Dijkstra::compute()
{
  bool finished = false;
  _maxDistance = 0;
  while (!finished && !_priorityQueue.empty()) {
    Point pointStruct = _priorityQueue.top()._point;
    pcl::PointXYZI point = pointStruct._point;
    int index = getIndex(point);
    if (pointStruct._distance == 1000) {
      finished = true;
    } else {
      _priorityQueue.pop();
      _points[index]._visited = true;
      _distances[index] = pointStruct._distance;
      std::vector<int> neighbors = getNeighbors(point);
      for (int i = 0; i < neighbors.size(); i++) {
        int indexNeighbor = neighbors[i];
        Point neighbor = (*_handle[indexNeighbor])._point;
        float distBetween = getDistance(point, neighbor._point);
        if (pointStruct._distance + distBetween < neighbor._distance) {
          _parentIndices[indexNeighbor] = index;
          float d = pointStruct._distance + distBetween;
          if (_maxDistance < d && d != 1000) {
            _maxDistance = d;
          }
          (*_handle[indexNeighbor])._point._distance = d;
          (*_handle[indexNeighbor])._point._point.intensity = pointStruct._point.intensity;
          _cloudIn->points[indexNeighbor].intensity = pointStruct._point.intensity;
          _priorityQueue.decrease(_handle[indexNeighbor]);
        }
      }
    }
  }
}

float
SF_Dijkstra::getMaxDistance() const
{
  return _maxDistance;
}

#endif // SF_DIJKSTRA_HPP
