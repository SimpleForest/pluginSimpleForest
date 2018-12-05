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

#include "sf_modelAbstractBuildingbrick.h"
#include "sf_modelAbstractSegment.h"

float Sf_ModelAbstractBuildingbrick::getGrowthLength() {
  float growthLength = getLength();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> children =
      getChildren();
  for (size_t i = 0; i < children.size(); i++) {
    std::shared_ptr<Sf_ModelAbstractBuildingbrick> child = children.at(i);
    growthLength += child->getGrowthLength();
  }
  return growthLength;
}

float Sf_ModelAbstractBuildingbrick::getGrowthVolume() {
  float growthVolume = getVolume();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> children =
      getChildren();
  for (size_t i = 0; i < children.size(); i++) {
    std::shared_ptr<Sf_ModelAbstractBuildingbrick> child = children.at(i);
    growthVolume += child->getGrowthVolume();
  }
  return growthVolume;
}

size_t Sf_ModelAbstractBuildingbrick::getIndex() const { return _indexVector; }

float Sf_ModelAbstractBuildingbrick::getDistance(const pcl::PointXYZ &point) {
  Eigen::Vector3f p(point.x, point.y, point.z);
  return getDistance(p);
}

float Sf_ModelAbstractBuildingbrick::getDistance(
    const pcl::PointXYZINormal &point) {
  Eigen::Vector3f p(point.x, point.y, point.z);
  return getDistance(p);
}

void Sf_ModelAbstractBuildingbrick::setIndex(const size_t &index) {
  _indexVector = index;
}

size_t Sf_ModelAbstractBuildingbrick::getID() const { return _ID; }

std::shared_ptr<SF_ModelAbstractSegment>
Sf_ModelAbstractBuildingbrick::getSegment() {
  return _segment.lock();
}

void Sf_ModelAbstractBuildingbrick::setSegment(
    std::shared_ptr<SF_ModelAbstractSegment> segment) {
  _segment = segment;
}

void Sf_ModelAbstractBuildingbrick::setID(const size_t &ID) { _ID = ID; }

Eigen::Vector3f Sf_ModelAbstractBuildingbrick::getStart() const {
  return _start;
}

Eigen::Vector3f Sf_ModelAbstractBuildingbrick::getEnd() const { return _end; }

std::shared_ptr<Sf_ModelAbstractBuildingbrick>
Sf_ModelAbstractBuildingbrick::getParent() {
  std::shared_ptr<Sf_ModelAbstractBuildingbrick> parent =
      getSegment()->getParentBuildingBrick(_indexVector);
  return parent;
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
Sf_ModelAbstractBuildingbrick::getChildren() {
  std::shared_ptr<SF_ModelAbstractSegment> segment = getSegment();
  return segment->getChildBuildingBricks(_indexVector);
}

float Sf_ModelAbstractBuildingbrick::getBoundingSphereRadius() {
  float halfLength = getLength() / 2;
  float radius = getRadius();
  return std::sqrt(halfLength * halfLength + radius * radius);
}

Sf_ModelAbstractBuildingbrick::Sf_ModelAbstractBuildingbrick() {}
