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
#ifndef SF_MATH_HPP
#define SF_MATH_HPP

#include "sf_math.h"

template <typename T> const T SF_Math<T>::_PI = 3.1415926;

template <typename T> const T SF_Math<T>::_RAD_TO_DEG = 180.0 / SF_Math::_PI;

template <typename T> const T SF_Math<T>::_DEG_TO_RAD = SF_Math::_PI / 180.0;

template <typename T>
T SF_Math<T>::getAngleBetweenDeg(Eigen::Vector3f axis1, Eigen::Vector3f axis2) {
  axis1.normalize();
  axis2.normalize();
  return acos(axis1.dot(axis2)) * SF_Math::_RAD_TO_DEG;
}

template <typename T>
T SF_Math<T>::distance(const Eigen::Vector3f &pointA,
                       const Eigen::Vector3f &pointB) {
  T distance = (pointA - pointB).norm();
  return distance;
}

template <typename T>
T SF_Math<T>::getAngleBetweenRad(Eigen::Vector3f axis1, Eigen::Vector3f axis2) {
  axis1.normalize();
  axis2.normalize();
  return acos(axis1.dot(axis2));
}

template <typename T> T SF_Math<T>::getMedian(std::vector<T> &vec) {
  size_t size = vec.size();
  if (size == 0) {
    return 0;
  } else {
    std::sort(vec.begin(), vec.end());
    if (size % 2 == 0) {
      return (vec[size / 2 - 1] + vec[size / 2]) / 2;
    } else {
      return vec[size / 2];
    }
  }
}

template <typename T> T SF_Math<T>::getMean(std::vector<T> &vec) {
  size_t size = vec.size();
  if (size == 0) {
    return 0;
  } else {
    T sum = 0;
    for (size_t i = 0; i < vec.size(); i++) {
      sum += vec[i];
    }
    return (sum / vec.size());
  }
}

#endif // SF_MATH_HPP
