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
#ifndef SF_MATH_H
#define SF_MATH_H

#include "pcl/point_types.h"

template<typename T>
class SF_Math
{
public:
  static const T _PI;
  static const T _RAD_TO_DEG;
  static const T _DEG_TO_RAD;
  static T getAngleBetweenDeg(Eigen::Vector3f axis1, Eigen::Vector3f axis2);
  static T distance(const Eigen::Vector3f& pointA, const Eigen::Vector3f& pointB);
  static T getAngleBetweenRad(Eigen::Vector3f axis1, Eigen::Vector3f axis2);
  static T getMedian(std::vector<T>& vec);
  static T getQuantile(std::vector<T>& vec, T quantile);
  static T getMean(std::vector<T>& vec);
  static T getStandardDeviation(std::vector<T>& vec);
  static T getStandardDeviation(std::vector<T>& vec, T mean);
  SF_Math();
};

#include "sf_math.hpp"

#endif // SF_MATH_H
