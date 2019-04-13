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

#include <algorithm>

#include "pcl/sf_math.h"
#include "sf_cloudToModelDistance.h"

void
Sf_CloudToModelDistance::initializeKdTree()
{
  _kdtreeQSM.reset(new pcl::KdTreeFLANN<pcl::PointXYZINormal>());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr centerCloud(new pcl::PointCloud<pcl::PointXYZINormal>());
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = _tree->getBuildingBricks();
  for (size_t i = 0; i < buildingBricks.size(); i++) {
    Eigen::Vector3f pointEigen = buildingBricks[i]->getCenter();
    pcl::PointXYZINormal point;
    point.x = pointEigen[0];
    point.y = pointEigen[1];
    point.z = pointEigen[2];
    centerCloud->push_back(point);
  }
  _kdtreeQSM->setInputCloud(centerCloud);
}

void
Sf_CloudToModelDistance::initializeGrowthLength()
{
  if (!(_METHOD == SF_CLoudToModelDistanceMethod::GROWTHDISTANCE)) {
    return;
  }
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = _tree->getBuildingBricks();
  size_t id = 0;
  std::for_each(buildingBricks.begin(), buildingBricks.end(), [this, &id](std::shared_ptr<Sf_ModelAbstractBuildingbrick>& brick) {
    brick->setID(id++);
    _growthLengths.push_back(brick->getGrowthLength());
  });
}

float
Sf_CloudToModelDistance::adaptDistanceToMethod(float distance)
{
  switch (_METHOD) {
    case SF_CLoudToModelDistanceMethod::ZEROMOMENTUMORDER:
      distance = std::abs(distance);
      break;
    case SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDER:
      distance = std::abs(distance);
      break;
    case SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDERMSAC:
      distance = std::abs(distance);
      distance = std::min(_INLIERDISTANCE, distance);
      break;
    case SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDER:
      distance = distance * distance;
      break;
    case SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDERMSAC:
      distance = distance * distance;
      distance = std::min(_INLIERDISTANCE * _INLIERDISTANCE, distance);
      break;
    case SF_CLoudToModelDistanceMethod::GROWTHDISTANCE:
      distance = std::abs(distance);
      break;
  }
  return distance;
}

float
Sf_CloudToModelDistance::getAverageDistance() const
{
  return _averageDistance;
}

std::vector<float>
Sf_CloudToModelDistance::distances() const
{
  return _distances;
}

float
Sf_CloudToModelDistance::getDistance(const pcl::PointXYZ& point, std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick)
{
  float distance = buildingBrick->getDistance(Eigen::Vector3f(point.x, point.y, point.z));
  return adaptDistanceToMethod(distance);
}

float
Sf_CloudToModelDistance::getDistance(const pcl::PointXYZINormal& point, std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick)
{
  float distance = buildingBrick->getDistance(Eigen::Vector3f(point.x, point.y, point.z));
  return adaptDistanceToMethod(distance);
}

float
Sf_CloudToModelDistance::getAngle(const pcl::PointXYZINormal& point, std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick)
{
  return SF_Math<float>::getAngleBetweenDeg(Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z), buildingBrick->getAxis());
}

void
Sf_CloudToModelDistance::compute()
{
  _distances = getCloudToModelDistances();
  std::vector<float> distances = cropDistances(_distances);
  switch (_METHOD) {
    case SF_CLoudToModelDistanceMethod::ZEROMOMENTUMORDER:
      _averageDistance = getNumberInliersNegative(distances);
      break;
    case SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDER:
      _averageDistance = SF_Math<float>::getMean(distances);
      break;
    case SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDERMSAC:
      _averageDistance = SF_Math<float>::getMean(distances);
      break;
    case SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDER:
      _averageDistance = std::sqrt(SF_Math<float>::getMean(distances));
      break;
    case SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDERMSAC:
      _averageDistance = std::sqrt(SF_Math<float>::getMean(distances));
      break;
    case SF_CLoudToModelDistanceMethod::GROWTHDISTANCE:
      _averageDistance = std::sqrt(SF_Math<float>::getMedian(distances));
      break;
    default:
      break;
  }
}

const std::vector<float>
Sf_CloudToModelDistance::cropDistances(std::vector<float> distances)
{
  if (_percentage >= 100) {
    return distances;
  }
  std::sort(distances.begin(), distances.end());
  size_t size = static_cast<size_t>(static_cast<float>(distances.size()) * static_cast<float>(_percentage) / 100.0f);
  std::vector<float> croppedDistances;
  for (size_t i = 0; i < size; i++) {
    croppedDistances.push_back(distances[i]);
  }
  return croppedDistances;
}

std::vector<float>
Sf_CloudToModelDistance::getCloudToModelDistances()
{
  std::vector<float> distances;
  int k = _k;
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = _tree->getBuildingBricks();
  for (size_t i = 0; i < _cloud->points.size(); i++) {
    pcl::PointXYZINormal point = _cloud->points[i];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (_kdtreeQSM->nearestKSearch(point, k, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
      float minDistance = maxError();
            float minDistanceAngle = maxError();
      std::shared_ptr<Sf_ModelAbstractBuildingbrick> bestBrick;
      for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
        std::shared_ptr<Sf_ModelAbstractBuildingbrick> neighboringBrick = buildingBricks[pointIdxRadiusSearch[j]];
        auto distance = getDistance(point, neighboringBrick);
                float distanceAngle;
                auto angle = getAngle(point, neighboringBrick);
                if (angle == 0) {
                  distanceAngle = maxError();
                } else {
                  distanceAngle = distance / angle;
                }

        if (distanceAngle < minDistanceAngle) {
//                    if (distance < minDistance) {
          bestBrick = neighboringBrick;
                    minDistanceAngle = distanceAngle;
          minDistance = distance;
        }
      }
      if (_METHOD == SF_CLoudToModelDistanceMethod::GROWTHDISTANCE) {
        if (bestBrick != nullptr) {
          distances.push_back(std::max(_growthLengths[bestBrick->getID()], _MIN_GROWTH_LENGTH));
        } else {
          distances.push_back(_MIN_GROWTH_LENGTH);
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

float
Sf_CloudToModelDistance::maxError() const
{
  return _INLIERDISTANCE * 2;
}

float
Sf_CloudToModelDistance::getNumberInliersNegative(const std::vector<float>& distances)
{
  float sum = 0;
  for (size_t i = 0; i < distances.size(); i++) {
    if (distances[i] < _INLIERDISTANCE)
      sum--;
  }
  return sum;
}

Sf_CloudToModelDistance::Sf_CloudToModelDistance(std::shared_ptr<SF_ModelQSM> tree,
                                                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                                                 SF_CLoudToModelDistanceMethod& method,
                                                 float inlierDistance,
                                                 int k,
                                                 int percentage)
  : _METHOD(method), _percentage(percentage), _k(k), _INLIERDISTANCE(inlierDistance), _tree(tree), _cloud(cloud)
{
  _averageDistance = std::numeric_limits<float>::max();
  initializeKdTree();
  initializeGrowthLength();
  compute();
}

Sf_CloudToModelDistance::Sf_CloudToModelDistance(std::shared_ptr<SF_ModelQSM> tree,
                                                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                                                 SF_CloudToModelDistanceParameters params)
  : _METHOD(params._method)
  , _percentage(params._robustPercentage)
  , _k(params._k)
  , _INLIERDISTANCE(params._inlierDistance)
  , _tree(tree)
  , _cloud(cloud)
{
  _averageDistance = std::numeric_limits<float>::max();
  initializeKdTree();
  initializeGrowthLength();
  compute();
}
