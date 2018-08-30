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

#include "sf_model_cylinder_buildingbrick.h"
#include "pcl/sf_math.h"

float SF_Model_Cylinder_Buildingbrick::getRadius() {
    return _radius;
}

float SF_Model_Cylinder_Buildingbrick::getVolume() {
    float volume = SF_Math<float>::_PI*_radius*_radius*getLength();
    return volume;
}

float SF_Model_Cylinder_Buildingbrick::getLength() {
    float length = std::sqrt((_start[0] - _end[0])*(_start[0] - _end[0]) + (_start[1] - _end[1])*(_start[1] - _end[1]) + (_start[2] - _end[2])*(_start[2] - _end[2]));
    return length;
}

Eigen::Vector3f SF_Model_Cylinder_Buildingbrick::getCenter() {
    Eigen::Vector3f center((_start[0] + _end[0])/2, (_start[1] + _end[1])/2, (_start[2] + _end[2])/2);
    return center;
}

Eigen::Vector3f SF_Model_Cylinder_Buildingbrick::getPrincipleDirection() {
    Eigen::Vector3f principleDirection((_end[0] - _start[0]), (_end[1] - _start[1]), (_end[2] - _start[2]));
    return principleDirection;
}

SF_Model_Cylinder_Buildingbrick::SF_Model_Cylinder_Buildingbrick(pcl::ModelCoefficients::Ptr circleA, pcl::ModelCoefficients::Ptr circleB) {
    assert(circleA->values.size()==4 && circleB->values.size()==4);
    _start[0] = circleA->values[0];
    _start[1] = circleA->values[1];
    _start[2] = circleA->values[2];
    _end[0] = circleB->values[0];
    _end[1] = circleB->values[1];
    _end[2] = circleB->values[2];
    _radius = (circleA->values[3]+circleB->values[3])/2;
}

std::string SF_Model_Cylinder_Buildingbrick::toString() {
    std::string str("cylinder");
    str.append(", ");
    str.append(_ID);
    str.append(", ");
    std::shared_ptr<SF_Model_Abstract_Buildingbrick> parent = getParent();
    if(parent == nullptr) {
        str.append("-1");
    } else {
        str.append(parent.getID());
    }
    str.append(", ");
    str.append(_start[0]);
    str.append(", ");
    str.append(_start[1]);
    str.append(", ");
    str.append(_start[2]);
    str.append(", ");
    str.append(_end[0]);
    str.append(", ");
    str.append(_end[1]);
    str.append(", ");
    str.append(_end[2]);
    str.append(", ");
    str.append(_radius);
    str.append(", ");
    str.append(getVolume());
    str.append(", ");
    str.append(getGrowthVolume());
    str.append(", ");
    str.append(getLength());
    str.append(", ");
    str.append(getGrowthLength());
    str.append(", ");
    std::shared_ptr<SF_Model_Abstract_Segment> segment = getSegment();
    str.append(segment->toString());
    return str;

}

std::string SF_Model_Cylinder_Buildingbrick::toHeaderString() {
    std::string str ("type, ID, parentID, startX, startY, startZ, endX, endY, endZ, radius, volume, growthVolume, length, growthLength,");
    std::shared_ptr<SF_Model_Abstract_Segment> segment = getSegment();
    str.append(segment->toHeaderString());
    return str;
}
