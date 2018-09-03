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

#ifndef SF_QSM_ALGORITHM_H
#define SF_QSM_ALGORITHM_H

#include <pcl/ModelCoefficients.h>
#include <boost/heap/fibonacci_heap.hpp>

#include "pcl/sf_math.h"

struct Cylinder
{
    float _distance;
    pcl::ModelCoefficients::Ptr _circleA;
    pcl::ModelCoefficients::Ptr _circleB;

    Cylinder(float distance, pcl::ModelCoefficients::Ptr circleA) {
        _distance = distance;
        _circleA   = circleA;
    }

    Cylinder(float distance, pcl::ModelCoefficients::Ptr circleA, pcl::ModelCoefficients::Ptr circleB) {
        Eigen::Vector3f pointA(circleA->values[0], circleA->values[1], circleA->values[2]);
        Eigen::Vector3f pointB(circleB->values[0], circleB->values[1], circleB->values[2]);
        _distance = distance + SF_Math<float>::distance(pointA, pointB);
        _circleA  = circleA;
        _circleB  = circleB;
    }

    void addCircle(pcl::ModelCoefficients::Ptr circleB) {
        Eigen::Vector3f pointA(_circleA->values[0], _circleA->values[1], _circleA->values[2]);
        Eigen::Vector3f pointB( circleB->values[0],  circleB->values[1],  circleB->values[2]);
        _distance = _distance + SF_Math<float>::distance(pointA, pointB);
        _circleB  = circleB;
    }
};

struct heapDataCylinder;

using HeapCylinder = boost::heap::fibonacci_heap<heapDataCylinder>;

struct heapDataCylinder {
    Cylinder _cylinder;
    HeapCylinder::handle_type handle;
    heapDataCylinder(Cylinder cylinder): _cylinder(cylinder),handle() {}
    bool operator<(heapDataCylinder const & second) const {
        return _cylinder._distance > second._cylinder._distance;
    }
};

#endif // SF_QSM_ALGORITHM_H
