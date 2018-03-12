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

template <typename T>
const double SF_Math<T>::_PI = 3.1415926;

template <typename T>
const double SF_Math<T>::_RAD_TO_DEG = 180.0/SF_Math::_PI;

template <typename T>
const double SF_Math<T>::_DEG_TO_RAD = SF_Math::_PI/180.0;

template <typename T>
double SF_Math<T>::get_angle_between_DEG(Eigen::Vector3f  axis1, Eigen::Vector3f  axis2) {
    axis1.normalize();
    axis2.normalize();
    return acos(axis1.dot(axis2))*SF_Math::_RAD_TO_DEG;
}

template <typename T>
double SF_Math<T>::get_angle_between_RAD(Eigen::Vector3f axis1, Eigen::Vector3f axis2) {
    axis1.normalize();
    axis2.normalize();
    return acos(axis1.dot(axis2));
}



#endif // SF_MATH_HPP
