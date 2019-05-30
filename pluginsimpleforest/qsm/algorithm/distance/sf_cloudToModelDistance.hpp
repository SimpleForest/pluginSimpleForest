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
                                                            float inlierDistance,
                                                            int k)
  : _METHOD(method), _k(k), _INLIERDISTANCE(inlierDistance), _tree(tree), _cloud(cloud)
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
    :Sf_CloudToModelDistance(tree, cloud, params._method, params._inlierDistance, params._k)
{
}

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
        if (_METHOD == SF_CLoudToModelDistanceMethod::GROWTHDISTANCE) {
          distance = -_growthLengths[pointIdxRadiusSearch[j]];
        }
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

template<typename PointType>
float
Sf_CloudToModelDistance<PointType>::maxError() const
{
  return _INLIERDISTANCE * 2;
}

template<typename PointType>
float
Sf_CloudToModelDistance<PointType>::getNumberInliersNegative(const std::vector<float>& distances)
{
  float sum = std::count_if(distances.begin(), distances.end(), [this](float distance) { return distance < _INLIERDISTANCE; });
  return -sum;
}

#endif // SF_CLOUDTOMODELDISTANCE_HPP
