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

#include <numeric>

template<typename T>
const T SF_Math<T>::_PI = 3.1415926;

template<typename T>
const T SF_Math<T>::_RAD_TO_DEG = 180.0 / SF_Math::_PI;

template<typename T>
const T SF_Math<T>::_DEG_TO_RAD = SF_Math::_PI / 180.0;

template<typename T>
T
SF_Math<T>::getAngleBetweenDegf(Eigen::Vector3f axis1, Eigen::Vector3f axis2)
{
  axis1.normalize();
  axis2.normalize();
  float angleBetween0And180 = acos(axis1.dot(axis2)) * SF_Math::_RAD_TO_DEG;
  return std::min(angleBetween0And180, static_cast<T>(180.0f - angleBetween0And180));
}

template<typename T>
T
SF_Math<T>::distancef(const Eigen::Vector3f& pointA, const Eigen::Vector3f& pointB)
{
  T distance = (pointA - pointB).norm();
  return distance;
}

template<typename T>
T
SF_Math<T>::getAngleBetweenRadf(Eigen::Vector3f axis1, Eigen::Vector3f axis2)
{
  axis1.normalize();
  axis2.normalize();
  return acos(axis1.dot(axis2));
}

template<typename T>
T
SF_Math<T>::getAngleBetweenDeg(Eigen::Vector3d axis1, Eigen::Vector3d axis2)
{
  axis1.normalize();
  axis2.normalize();
  T angleBetween0And180 = acos(axis1.dot(axis2)) * SF_Math::_RAD_TO_DEG;
  return std::min(angleBetween0And180, static_cast<T>(180.0 - angleBetween0And180));
}

template<typename T>
T
SF_Math<T>::distance(const Eigen::Vector3d& pointA, const Eigen::Vector3d& pointB)
{
  T distance = (pointA - pointB).norm();
  return distance;
}

template<typename T>
T
SF_Math<T>::getAngleBetweenRad(Eigen::Vector3d axis1, Eigen::Vector3d axis2)
{
  axis1.normalize();
  axis2.normalize();
  return acos(axis1.dot(axis2));
}

template<typename T>
T
SF_Math<T>::getMedian(std::vector<T>& vec)
{
  return getQuantile(vec, 0.5);
}

template<typename T>
T
SF_Math<T>::getQuantile(std::vector<T>& vec, T quantile)
{
  if (vec.size() == 0) {
    return 0;
  }
  size_t n = vec.size() * quantile;
  std::nth_element(vec.begin(), vec.begin() + n, vec.end());
  return vec[n];
}

template<typename T>
T
SF_Math<T>::getMean(std::vector<T>& vec)
{
  auto lambda = [&vec](double a, double b) { return a + b; };
  return (std::accumulate(vec.begin(), vec.end(), static_cast<T>(0), lambda) / vec.size());
}

template<typename T>
T
SF_Math<T>::getStandardDeviation(std::vector<T>& vec)
{
  return getStandardDeviation(vec, getMean(vec));
}

template<typename T>
T
SF_Math<T>::getStandardDeviation(std::vector<T>& vec, T mean)
{
  std::vector<float> diff(vec.size());
  std::transform(vec.begin(), vec.end(), diff.begin(), std::bind2nd(std::minus<float>(), mean));
  return (std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0f) / vec.size());
}

#endif // SF_MATH_HPP
