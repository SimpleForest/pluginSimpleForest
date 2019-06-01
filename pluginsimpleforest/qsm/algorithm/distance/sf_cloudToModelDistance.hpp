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

#ifndef SF_CLOUDTOMODELDISTANCE_HPP
#define SF_CLOUDTOMODELDISTANCE_HPP

#include "qsm/algorithm/distance/sf_cloudToModelDistance.h"

#include "pcl/sf_math.h"
#include <algorithm>

template<typename PointType>
float
Sf_CloudToModelDistance<PointType>::getDistance(const PointType& point, std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick)
{
  float distance = buildingBrick->getDistance(point.getVector3fMap());
  return adaptDistanceToMethod(distance);
}

template<typename PointType>
float
Sf_CloudToModelDistance<PointType>::getAngle(const PointType& point, std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick)
{
  return SF_Math<float>::getAngleBetweenDeg(point.getVector3fMap(), buildingBrick->getAxis());
}

template<typename PointType>
void
Sf_CloudToModelDistance<PointType>::initializeKdTree()
{
  _kdtreeQSM.reset(new typename pcl::KdTreeFLANN<PointType>());
  typename pcl::PointCloud<PointType>::Ptr centerCloud(new typename pcl::PointCloud<PointType>());
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = _tree->getBuildingBricks();
  for (size_t i = 0; i < buildingBricks.size(); i++) {
    Eigen::Vector3f pointEigen = buildingBricks[i]->getCenter();
    PointType point;
    point.getVector3fMap() = pointEigen;
    centerCloud->push_back(std::move(point));
  }
  _kdtreeQSM->setInputCloud(centerCloud);
}

template<typename PointType>
Sf_CloudToModelDistance<PointType>::Sf_CloudToModelDistance(std::shared_ptr<SF_ModelQSM> tree,
                                                            typename pcl::PointCloud<PointType>::Ptr cloud,
                                                            SF_CLoudToModelDistanceMethod& method,
                                                            float cropDistance,
                                                            int k)
  : _METHOD(method), _k(k), _cropDistance(cropDistance), _tree(tree), _cloud(cloud)
{
  _averageDistance = std::numeric_limits<float>::max();
  initializeKdTree();
  initializeGrowthLength();
  compute();
}

template<typename PointType>
Sf_CloudToModelDistance<PointType>::Sf_CloudToModelDistance(std::shared_ptr<SF_ModelQSM> tree,
                                                            typename pcl::PointCloud<PointType>::Ptr cloud,
                                                            SF_CloudToModelDistanceParameters& params)
  : Sf_CloudToModelDistance(tree, cloud, params._method, params._cropDistance, params._k)
{}

template<typename PointType>
std::vector<float>
Sf_CloudToModelDistance<PointType>::getCloudToModelDistances()
{
  std::vector<float> distances;
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = _tree->getBuildingBricks();
  for (size_t i = 0; i < _cloud->points.size(); i++) {
    PointType point = _cloud->points[i];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (_kdtreeQSM->nearestKSearch(point, _k, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
      float minDistance = maxError();
      std::shared_ptr<Sf_ModelAbstractBuildingbrick> bestBrick;
      for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
        std::shared_ptr<Sf_ModelAbstractBuildingbrick> neighboringBrick = buildingBricks[pointIdxRadiusSearch[j]];
        auto distance = getDistance(point, neighboringBrick);
        if (distance < minDistance) {
          bestBrick = neighboringBrick;
          minDistance = distance;
        }
      }
      if (_METHOD == SF_CLoudToModelDistanceMethod::GROWTHDISTANCE) {
        if (bestBrick != nullptr) {
          distances.push_back(std::min(-_growthLengths[bestBrick->getID()], -_MIN_GROWTH_LENGTH));
        } else {
          distances.push_back(-_MIN_GROWTH_LENGTH);
        }
      } else {
        distances.push_back(minDistance);
      }
    } else {
      if (_METHOD == SF_CLoudToModelDistanceMethod::GROWTHDISTANCE) {
        distances.push_back(_MIN_GROWTH_LENGTH);
      } else {
        distances.push_back(maxError());
      }
    }
  }
  return distances;
}

template<typename PointType>
void
Sf_CloudToModelDistance<PointType>::initializeGrowthLength()
{
  if (_METHOD != SF_CLoudToModelDistanceMethod::GROWTHDISTANCE) {
    return;
  }
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = _tree->getBuildingBricks();
  size_t id = 0;
  std::for_each(buildingBricks.begin(), buildingBricks.end(), [this, &id](std::shared_ptr<Sf_ModelAbstractBuildingbrick>& brick) {
    brick->setID(id++);
    _growthLengths.push_back(brick->getGrowthLength());
  });
}

template<typename PointType>
float
Sf_CloudToModelDistance<PointType>::adaptDistanceToMethod(float distance)
{
  switch (_METHOD) {
    case SF_CLoudToModelDistanceMethod::ZEROMOMENTUMORDER:
      distance = std::abs(distance);
      break;
    case SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDER:
      distance = std::abs(distance);
      break;
    case SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDERMSAC:
      distance = std::min(_cropDistance, distance);
      distance = std::abs(distance);
      break;
    case SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDER:
      distance = distance * distance;
      break;
    case SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDERMSAC:
      distance = std::min(_cropDistance, distance);
      distance = distance * distance;
      break;
    case SF_CLoudToModelDistanceMethod::GROWTHDISTANCE:
      distance = std::abs(distance);
      break;
  }
  return distance;
}

template<typename PointType>
float
Sf_CloudToModelDistance<PointType>::getAverageDistance() const
{
  return _averageDistance;
}

template<typename PointType>
std::vector<float>
Sf_CloudToModelDistance<PointType>::distances() const
{
  return _distances;
}

template<typename PointType>
void
Sf_CloudToModelDistance<PointType>::compute()
{
  _distances = getCloudToModelDistances();
  auto distancesCopy = _distances;
  switch (_METHOD) {
    case SF_CLoudToModelDistanceMethod::ZEROMOMENTUMORDER:
      _averageDistance = getNumberInliersNegative(distancesCopy);
      break;
    case SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDER:
      _averageDistance = SF_Math<float>::getMean(distancesCopy);
      break;
    case SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDERMSAC:
      _averageDistance = SF_Math<float>::getMean(distancesCopy);
      break;
    case SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDER:
      _averageDistance = std::sqrt(SF_Math<float>::getMean(distancesCopy));
      break;
    case SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDERMSAC:
      _averageDistance = std::sqrt(SF_Math<float>::getMean(distancesCopy));
      break;
    case SF_CLoudToModelDistanceMethod::GROWTHDISTANCE:
      _averageDistance = std::sqrt(SF_Math<float>::getMedian(distancesCopy));
      break;
    default:
      break;
  }
}

template<typename PointType>
float
Sf_CloudToModelDistance<PointType>::maxError() const
{
  return std::numeric_limits<float>::max();
}

template<typename PointType>
float
Sf_CloudToModelDistance<PointType>::getNumberInliersNegative(const std::vector<float>& distances)
{
  float sum = std::count_if(distances.begin(), distances.end(), [this](float distance) { return distance < _cropDistance; });
  return -sum;
}

#endif // SF_CLOUDTOMODELDISTANCE_HPP
