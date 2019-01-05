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

#include "sf_modelAbstractSegment.h"
#include "pcl/sf_math.h"
#include "sf_modelQSM.h"

void SF_ModelAbstractSegment::setParent(
    const std::weak_ptr<SF_ModelAbstractSegment> &parent) {
  _parent = parent;
}

std::shared_ptr<SF_ModelAbstractSegment> SF_ModelAbstractSegment::getParent() {
  return _parent.lock();
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_ModelAbstractSegment::getBuildingBricks() const {
  return _buildingBricks;
}

std::string SF_ModelAbstractSegment::toString() {
  std::string str(std::to_string(_ID));
  str.append(", ");
  std::shared_ptr<SF_ModelAbstractSegment> parent = getParent();
  if (parent == nullptr) {
    str.append("-1");
  } else {
    str.append(std::to_string(parent->getID()));
  }
  str.append(", ");
  str.append(std::to_string(getRadius()));
  str.append(", ");
  str.append(std::to_string(
      _buildingBricks[_buildingBricks.size() / 2]->getGrowthVolume()));
  str.append(", ");
  str.append(std::to_string(getLength()));
  str.append(", ");
  str.append(std::to_string(
      _buildingBricks[_buildingBricks.size() / 2]->getGrowthLength()));
  str.append(", ");
  str.append(std::to_string(_branchOrder));
  str.append(", ");
  str.append(std::to_string(_reverseBranchOrder));
  str.append(", ");
  str.append(std::to_string(_reversePipeBranchOrder));
  str.append(", ");
  str.append(std::to_string(_branchID));
  str.append(", ");
  str.append(getTree()->toString());
  return str;
}

std::string SF_ModelAbstractSegment::toHeaderString() {
  std::string str("segmentID, parentSegmentID, segmentMedianRadius, "
                  "segmentGrowthVolume, segmentGrowthLength, branchOrder, "
                  "reverseBranchOrder, reversePipeBranchorder, branchID, ");
  str.append(getTree()->toHeaderString());
  return str;
}

Eigen::Vector3f SF_ModelAbstractSegment::getStart() const {
  Eigen::Vector3f start;
  if (!_buildingBricks.empty()) {
    start = _buildingBricks[0]->getStart();
  }
  return start;
}

Eigen::Vector3f SF_ModelAbstractSegment::getEnd() const {
  Eigen::Vector3f end;
  if (!_buildingBricks.empty()) {
    end = _buildingBricks[_buildingBricks.size() - 1]->getEnd();
  }
  return end;
}

void SF_ModelAbstractSegment::remove() {
    if(!isRoot()) {
        std::shared_ptr<SF_ModelAbstractSegment> parent = getParent();
        std::vector<std::shared_ptr<SF_ModelAbstractSegment> > parentsChildrenNew;
        std::vector<std::shared_ptr<SF_ModelAbstractSegment> > parentsChildren = parent->getChildSegments();
        std::for_each(parentsChildren.begin(), parentsChildren.end(), [&parentsChildrenNew, this](std::shared_ptr<SF_ModelAbstractSegment> parentsChild){
            if(parentsChild!= shared_from_this()) {
                parentsChildrenNew.push_back(parentsChild);
            }
        });
        parent->setChildSegments(parentsChildrenNew);
    }
}

float SF_ModelAbstractSegment::getRadius() const {
  std::vector<float> radii;
  for (size_t i = 0; i < _buildingBricks.size(); i++) {
    radii.push_back(_buildingBricks[i]->getRadius());
  }
  return SF_Math<float>::getMedian(radii);
}

float SF_ModelAbstractSegment::getVolume() const {
  float volume = 0;
  for (size_t i = 0; i < _buildingBricks.size(); i++) {
    volume += _buildingBricks[i]->getVolume();
  }
  return volume;
}

float SF_ModelAbstractSegment::getLength() const {
  float length = 0;
  for (size_t i = 0; i < _buildingBricks.size(); i++) {
    length += _buildingBricks[i]->getLength();
  }
  return length;
}

bool SF_ModelAbstractSegment::isRoot() const {
    if(getTree()->getRootSegment() != shared_from_this() ) {
        return false;
    }
    return true;
}

int SF_ModelAbstractSegment::getID() const { return _ID; }

void SF_ModelAbstractSegment::setID(int ID) { _ID = ID; }

std::vector<std::shared_ptr<SF_ModelAbstractSegment>>
SF_ModelAbstractSegment::getChildSegments() const {
  return _childSegments;
}

std::shared_ptr<SF_ModelQSM> SF_ModelAbstractSegment::getTree() const {
  return _tree.lock();
}

void SF_ModelAbstractSegment::setChildSegments(const std::vector<std::shared_ptr<SF_ModelAbstractSegment> > childSegments)
{
    _childSegments = childSegments;
}

int SF_ModelAbstractSegment::getParentID() {
    std::shared_ptr<SF_ModelAbstractSegment> parent = getParent();
    if (parent == nullptr) {
        return -1;
    } else {
    return parent->getID();
  }
}

void SF_ModelAbstractSegment::addChild(
    std::shared_ptr<SF_ModelAbstractSegment> child) {
  child->setParent(shared_from_this());
  _childSegments.push_back(child);
}

void SF_ModelAbstractSegment::addBuildingBrick(
    std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick) {
  buildingBrick->setIndex(_buildingBricks.size());
  buildingBrick->setSegment(shared_from_this());
  _buildingBricks.push_back(buildingBrick);
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_ModelAbstractSegment::getChildBuildingBricks(const size_t index) {
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> children;
  if (index < _buildingBricks.size() - 1) {
    children.push_back(_buildingBricks.at(index + 1));
  } else if (index == _buildingBricks.size() - 1) {
    for (size_t i = 0; i < _childSegments.size(); i++) {
      std::shared_ptr<Sf_ModelAbstractBuildingbrick> child =
          _childSegments.at(i)->getBuildingBricks()[0];
      children.push_back(child);
    }
  }
  return children;
}

std::shared_ptr<Sf_ModelAbstractBuildingbrick>
SF_ModelAbstractSegment::getParentBuildingBrick(const size_t index) {
  std::shared_ptr<Sf_ModelAbstractBuildingbrick> parent;
  if (index == 0) {
    std::shared_ptr<SF_ModelAbstractSegment> parentSegment = getParent();
    if (parentSegment != nullptr) {
      parent =
          parentSegment
              ->getBuildingBricks()[parentSegment->getBuildingBricks().size() -
                                    1];
    }
  } else {
    parent = _buildingBricks[index - 1];
  }
  return parent;
}

SF_ModelAbstractSegment::SF_ModelAbstractSegment(
    std::shared_ptr<SF_ModelQSM> tree)
    : _tree(tree) {
  _ID = -1;
  _branchOrder = -1;
  _reverseBranchOrder = -1;
  _reversePipeBranchOrder = -1;
  _branchID = -1;
}
