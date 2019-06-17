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
#include <clocale>

double
Sf_ModelCylinderBuildingbrick::getRadius()
{
  return m_radius;
}

void
Sf_ModelCylinderBuildingbrick::transform(const Eigen::Affine3f& transform)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ p1;
  p1.x = _start[0];
  p1.y = _start[1];
  p1.z = _start[2];
  pcl::PointXYZ p2;
  p2.x = _end[0];
  p2.y = _end[1];
  p2.z = _end[2];
  cloud.points.push_back(p1);
  cloud.points.push_back(p2);

  pcl::PointCloud<pcl::PointXYZ> transformed;
  pcl::transformPointCloud(cloud, transformed, transform);
  p1 = transformed.points.at(0);
  p2 = transformed.points.at(1);

  _start[0] = p1.x;
  _start[1] = p1.y;
  _start[2] = p1.z;
  _end[0] = p2.x;
  _end[1] = p2.y;
  _end[2] = p2.z;
}

void
Sf_ModelCylinderBuildingbrick::setRadius(double radius, FittingType type)
{
  _fittingType = type;
  m_radius = radius;
}

void
Sf_ModelCylinderBuildingbrick::translate(Eigen::Vector3d translation)
{
  _start = _start + translation;
  _end = _end + translation;
}

double
Sf_ModelCylinderBuildingbrick::getVolume()
{
  double volume = SF_Math<double>::_PI * m_radius * m_radius * getLength();
  return volume;
}

double
Sf_ModelCylinderBuildingbrick::getLength()
{
  Eigen::Vector3d c = _end - _start;
  return c.norm();
}

double
Sf_ModelCylinderBuildingbrick::getDistance(const Eigen::Vector3d& point)
{
  double distToAxis = getDistanceToAxis(point);
  double distToSegment = getProjectedDistanceToSegment(point);
  double distToHull = distToAxis - m_radius;
  double distance = (std::sqrt((distToHull * distToHull) + (distToSegment * distToSegment)));
  return distance;
}

double
Sf_ModelCylinderBuildingbrick::getDistanceToAxis(const Eigen::Vector3d& point)
{
  auto projection = getProjectionOnAxis(point);
  return (point - projection).norm();
}

double
Sf_ModelCylinderBuildingbrick::getProjectedDistanceToSegment(const Eigen::Vector3d& point)
{
  Eigen::Vector3d projection = getProjectionOnAxis(point);
  double distToStart = SF_Math<double>::distance(_start, projection);
  double distToEnd = SF_Math<double>::distance(_end, projection);
  double length = getLength();
  if (distToStart <= length && distToEnd <= length) {
    return 0;
  }
  return std::min(distToStart, distToEnd);
}

double
Sf_ModelCylinderBuildingbrick::getDistanceToInfinitHull(const Eigen::Vector3d& point)
{
  return getDistanceToAxis(point) - m_radius;
}

Eigen::Vector3d
Sf_ModelCylinderBuildingbrick::getProjectionOnAxis(const Eigen::Vector3d& point)
{
  Eigen::Vector3d a = point - _start;
  Eigen::Vector3d b = _end - _start;
  return (_start + (a.dot(b) / b.dot(b)) * b);
}

Eigen::Vector3d
Sf_ModelCylinderBuildingbrick::getCenter()
{
  return ((_start + _end) / 2);
}

Eigen::Vector3d
Sf_ModelCylinderBuildingbrick::getAxis()
{
  return (_end - _start);
}

void
Sf_ModelCylinderBuildingbrick::setStartEndRadius(const Eigen::Vector3d& start,
                                                 const Eigen::Vector3d& end,
                                                 double radius,
                                                 FittingType type)
{
  _start = start;
  _end = end;
  m_radius = radius;
  _fittingType = type;
}

void
Sf_ModelCylinderBuildingbrick::setCoefficients(pcl::ModelCoefficients::Ptr coefficients)
{
  if (coefficients->values.size() == 7) {
    auto oldStart = _start;
    auto oldEnd = _end;
    auto newAxis = Eigen::Vector3d(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    auto newPoint = Eigen::Vector3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3d a = oldStart - newPoint;
    Eigen::Vector3d b = newAxis;
    auto newStart = (newPoint + (a.dot(b) / b.dot(b)) * b);
    a = oldEnd - newPoint;
    auto newEnd = (newPoint + (a.dot(b) / b.dot(b)) * b);
    setStartEndRadius(newStart, newEnd, coefficients->values[6], FittingType::CYLINDERCORRECTION);
  }
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

Sf_ModelCylinderBuildingbrick::Sf_ModelCylinderBuildingbrick(Eigen::Vector3d start, Eigen::Vector3d end, double radius)
{
  _start = start;
  _end = end;
  m_radius = radius;
}

std::string
Sf_ModelCylinderBuildingbrick::toString()
{
  std::setlocale(LC_NUMERIC, "C");
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
