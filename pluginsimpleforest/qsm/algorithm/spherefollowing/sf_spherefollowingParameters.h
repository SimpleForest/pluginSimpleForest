#ifndef SF_SPHEREFOLLOWING_PARAMETERS_H
#define SF_SPHEREFOLLOWING_PARAMETERS_H

#include <vector>
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
#include <pcl/segmentation/sac_segmentation.h>

struct SF_SphereFollowingOptimizationParameters {
  float _euclideanClusteringDistance = 0.03f;
  float _sphereRadiusMultiplier = 2.0f;
  float _epsilonSphere = 0.035f;
  float _minRadius = 0.07f;
  float _medianRadiusMultiplier = 3.0f;
  SF_SphereFollowingOptimizationParameters() {}
  SF_SphereFollowingOptimizationParameters(float euclideanClusteringDistance,
                                           float sphereRadiusMultiplier,
                                           float epsilonSphere, float minRadius)
      : _euclideanClusteringDistance(euclideanClusteringDistance),
        _sphereRadiusMultiplier(sphereRadiusMultiplier),
        _epsilonSphere(epsilonSphere), _minRadius(minRadius) {}
};

struct SF_SphereFollowingParameters {
  std::vector<SF_SphereFollowingOptimizationParameters> m_optimizationParams;
  int _fittingMethod = pcl::SAC_MLESAC;  
  float _heapDelta = 0.2f;
  float _inlierDistance = 0.03f;
  int _minPtsGeometry = 3;
  int _RANSACIterations = 100;
  float _minGlobalRadius = 0.04f;
  float _heightInitializationSlice = 0.1f;
  SF_SphereFollowingParameters() {}
};

#endif // SF_SPHEREFOLLOWING_PARAMETERS_H