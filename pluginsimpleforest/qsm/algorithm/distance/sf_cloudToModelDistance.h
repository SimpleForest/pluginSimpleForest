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

#ifndef SF_CLOUD_TO_MODEL_DISTANCE_H
#define SF_CLOUD_TO_MODEL_DISTANCE_H

#include "qsm/model/sf_modelQSM.h"
#include "sf_cloudToModelDistanceParameters.h"
#include <pcl/kdtree/kdtree_flann.h>

template<typename PointType>
class Sf_CloudToModelDistance
{
  SF_CLoudToModelDistanceMethod _METHOD;
  int _percentage;
  int _k;
  float _INLIERDISTANCE;
  float _averageDistance;
  const float _MIN_GROWTH_LENGTH = 0.001f;
  std::vector<float> _distances;
  std::vector<float> _growthLengths;
  std::shared_ptr<SF_ModelQSM> _tree;
  typename pcl::KdTreeFLANN<PointType>::Ptr _kdtreeQSM;
  typename pcl::PointCloud<PointType>::Ptr _cloud;

  void initializeKdTree();
  void initializeGrowthLength();
  void compute();
  const std::vector<float> cropDistances(std::vector<float> distances);
  float getDistance(const PointType& point, std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick);
  float getAngle(const PointType& point, std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick);
  float getNumberInliersNegative(const std::vector<float>& distances);
  float adaptDistanceToMethod(float distance);
  std::vector<float> getCloudToModelDistances();
  float maxError() const;

public:
  Sf_CloudToModelDistance(std::shared_ptr<SF_ModelQSM> tree,
                          typename pcl::PointCloud<PointType>::Ptr cloud,
                          SF_CLoudToModelDistanceMethod& method,
                          float inlierDistance,
                          int k,
                          int percentage);
  Sf_CloudToModelDistance(std::shared_ptr<SF_ModelQSM> tree,
                          typename pcl::PointCloud<PointType>::Ptr cloud,
                          SF_CloudToModelDistanceParameters& params);
  float getAverageDistance() const;
  std::vector<float> distances() const;
};

#include "qsm/algorithm/distance/sf_cloudToModelDistance.hpp"

#endif // SF_CLOUD_TO_MODEL_DISTANCE_H
