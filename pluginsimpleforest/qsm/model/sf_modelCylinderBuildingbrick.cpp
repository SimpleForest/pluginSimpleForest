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

#include <Eigen/Dense>

#include "pcl/sf_math.h"
#include "sf_modelAbstractBuildingbrick.h"
#include "sf_modelAbstractSegment.h"
#include "sf_modelCylinderBuildingbrick.h"

float
Sf_ModelCylinderBuildingbrick::getRadius()
{
  return m_radius;
}

void
Sf_ModelCylinderBuildingbrick::setRadius(float radius, FittingType type)
{
  _fittingType = type;
  m_radius = radius;
}

void
Sf_ModelCylinderBuildingbrick::translate(Eigen::Vector3f translation)
{
  _start = _start + translation;
  _end = _end + translation;
}

float
Sf_ModelCylinderBuildingbrick::getVolume()
{
  float volume = SF_Math<float>::_PI * m_radius * m_radius * getLength();
  return volume;
}

float
Sf_ModelCylinderBuildingbrick::getLength()
{
  Eigen::Vector3f c = _end - _start;
  return c.norm();
}

float
Sf_ModelCylinderBuildingbrick::getDistance(const Eigen::Vector3f& point)
{
  float distToAxis = getDistanceToAxis(point);
  float distToSegment = getProjectedDistanceToSegment(point);
  float distToHull = distToAxis - m_radius;
  float distance = (std::sqrt((distToHull * distToHull) + (distToSegment * distToSegment)));
  return distance;
}

float
Sf_ModelCylinderBuildingbrick::getDistanceToAxis(const Eigen::Vector3f& point)
{
  Eigen::Vector3f a = point - _start;
  Eigen::Vector3f b = point - _end;
  Eigen::Vector3f c = _end - _start;
  Eigen::Vector3f d = a.cross(b);
  return d.norm() / c.norm();
}

float
Sf_ModelCylinderBuildingbrick::getProjectedDistanceToSegment(const Eigen::Vector3f& point)
{
  Eigen::Vector3f projection = getProjectionOnAxis(point);
  float distToStart = SF_Math<float>::distance(_start, projection);
  float distToEnd = SF_Math<float>::distance(_end, projection);
  float length = getLength();
  if (distToStart <= length && distToEnd <= length) {
    return 0;
  }
  return std::min(distToStart, distToEnd);
}

float
Sf_ModelCylinderBuildingbrick::getDistanceToInfinitHull(const Eigen::Vector3f& point)
{
  return getDistanceToAxis(point) - m_radius;
}

Eigen::Vector3f
Sf_ModelCylinderBuildingbrick::getProjectionOnAxis(const Eigen::Vector3f& point)
{
  Eigen::Vector3f a = point - _start;
  Eigen::Vector3f b = _end - _start;
  return (_start + (a.dot(b) / b.dot(b)) * b);
}

Eigen::Vector3f
Sf_ModelCylinderBuildingbrick::getCenter()
{
  return ((_start + _end) / 2);
}

Eigen::Vector3f
Sf_ModelCylinderBuildingbrick::getAxis()
{
  return (_end - _start);
}

void
Sf_ModelCylinderBuildingbrick::setStartEndRadius(const Eigen::Vector3f& start,
                                                 const Eigen::Vector3f& end,
                                                 float radius,
                                                 FittingType type)
{
  _start = start;
  _end = end;
  m_radius = radius;
  _fittingType = type;
}

Sf_ModelCylinderBuildingbrick::Sf_ModelCylinderBuildingbrick(pcl::ModelCoefficients::Ptr circleA, pcl::ModelCoefficients::Ptr circleB)
{
  assert(circleA->values.size() == 4 && circleB->values.size() == 4);
  _start[0] = circleA->values[0];
  _start[1] = circleA->values[1];
  _start[2] = circleA->values[2];
  _end[0] = circleB->values[0];
  _end[1] = circleB->values[1];
  _end[2] = circleB->values[2];
  m_radius = circleB->values[3];
  _fittingType = FittingType::SPHEREFOLLOWING;
}

std::string
Sf_ModelCylinderBuildingbrick::toString()
{
  std::string str("cylinder");
  str.append(", ");
  str.append(std::to_string(_ID));
  str.append(", ");
  std::shared_ptr<Sf_ModelAbstractBuildingbrick> parent = getParent();
  if (parent == nullptr) {
    str.append("-1");
  } else {
    str.append(std::to_string(parent->getID()));
  }
  str.append(", ");
  str.append(std::to_string(_start[0]));
  str.append(", ");
  str.append(std::to_string(_start[1]));
  str.append(", ");
  str.append(std::to_string(_start[2]));
  str.append(", ");
  str.append(std::to_string(_end[0]));
  str.append(", ");
  str.append(std::to_string(_end[1]));
  str.append(", ");
  str.append(std::to_string(_end[2]));
  str.append(", ");
  str.append(std::to_string(m_radius));
  str.append(", ");
  str.append(std::to_string(getVolume()));
  str.append(", ");
  str.append(std::to_string(getGrowthVolume()));
  str.append(", ");
  str.append(std::to_string(getLength()));
  str.append(", ");
  str.append(std::to_string(getGrowthLength()));
  str.append(", ");
  std::shared_ptr<SF_ModelAbstractSegment> segment = getSegment();
  str.append(segment->toString());
  return str;
}

std::string
Sf_ModelCylinderBuildingbrick::toHeaderString()
{
  std::string str("type, ID, parentID, startX, startY, startZ, endX, endY, "
                  "endZ, radius, volume, growthVolume, length, growthLength,");
  std::shared_ptr<SF_ModelAbstractSegment> segment = getSegment();
  str.append(segment->toHeaderString());
  return str;
}
