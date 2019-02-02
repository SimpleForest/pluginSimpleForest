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

void
SF_ModelAbstractSegment::setParent(const std::weak_ptr<SF_ModelAbstractSegment>& parent)
{
  m_parent = parent;
}

std::shared_ptr<SF_ModelAbstractSegment>
SF_ModelAbstractSegment::getParent()
{
  return m_parent.lock();
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_ModelAbstractSegment::getBuildingBricks() const
{
  return m_buildingBricks;
}

std::string
SF_ModelAbstractSegment::toString()
{
  std::string str(std::to_string(m_ID));
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
  str.append(std::to_string(m_buildingBricks[m_buildingBricks.size() / 2]->getGrowthVolume()));
  str.append(", ");
  str.append(std::to_string(getLength()));
  str.append(", ");
  str.append(std::to_string(m_buildingBricks[m_buildingBricks.size() / 2]->getGrowthLength()));
  str.append(", ");
  str.append(std::to_string(m_branchOrder));
  str.append(", ");
  str.append(std::to_string(m_reverseBranchOrder));
  str.append(", ");
  str.append(std::to_string(m_reversePipeBranchOrder));
  str.append(", ");
  str.append(std::to_string(m_branchID));
  str.append(", ");
  str.append(getTree()->toString());
  return str;
}

std::string
SF_ModelAbstractSegment::toHeaderString()
{
  std::string str("segmentID, parentSegmentID, segmentMedianRadius, "
                  "segmentGrowthVolume, segmentGrowthLength, branchOrder, "
                  "reverseBranchOrder, reversePipeBranchorder, branchID, ");
  str.append(getTree()->toHeaderString());
  return str;
}

Eigen::Vector3f
SF_ModelAbstractSegment::getStart() const
{
  Eigen::Vector3f start;
  if (!m_buildingBricks.empty()) {
    start = m_buildingBricks[0]->getStart();
  }
  return start;
}

Eigen::Vector3f
SF_ModelAbstractSegment::getEnd() const
{
  Eigen::Vector3f end;
  if (!m_buildingBricks.empty()) {
    end = m_buildingBricks[m_buildingBricks.size() - 1]->getEnd();
  }
  return end;
}

Eigen::Vector3f
SF_ModelAbstractSegment::getAxis() const
{
  return (getEnd() - getStart());
}

void
SF_ModelAbstractSegment::remove()
{
  if (!isRoot()) {
    std::shared_ptr<SF_ModelAbstractSegment> parent = getParent();
    std::vector<std::shared_ptr<SF_ModelAbstractSegment>> parentsChildren = parent->getChildren();
    std::vector<std::shared_ptr<SF_ModelAbstractSegment>>::iterator position = std::find(
      parentsChildren.begin(), parentsChildren.end(), shared_from_this());
    if (position != parentsChildren.end()) {
      parentsChildren.erase(position);
      parent->setChildSegments(parentsChildren);
    }
    m_parent.reset();
    m_tree.reset();
  }
}

float
SF_ModelAbstractSegment::getRadius() const
{
  std::vector<float> radii;
  for (size_t i = 0; i < m_buildingBricks.size(); i++) {
    radii.push_back(m_buildingBricks[i]->getRadius());
  }
  return SF_Math<float>::getMedian(radii);
}

float
SF_ModelAbstractSegment::getVolume() const
{
  float volume = 0;
  for (size_t i = 0; i < m_buildingBricks.size(); i++) {
    volume += m_buildingBricks[i]->getVolume();
  }
  return volume;
}

float
SF_ModelAbstractSegment::getGrowthVolume() const
{
  if (m_buildingBricks.size() >= 0) {
    size_t index = m_buildingBricks.size() / 2;
    return (m_buildingBricks[index]->getGrowthVolume());
  }
  return 0;
}

float
SF_ModelAbstractSegment::getGrowthLength() const
{
  if (m_buildingBricks.size() >= 0) {
    size_t index = m_buildingBricks.size() / 2;
    return (m_buildingBricks[index]->getGrowthLength());
  }
  return 0;
}

float
SF_ModelAbstractSegment::getLength() const
{
  float length = 0;
  for (size_t i = 0; i < m_buildingBricks.size(); i++) {
    length += m_buildingBricks[i]->getLength();
  }
  return length;
}

bool
SF_ModelAbstractSegment::isRoot()
{
  if (getParent()) {
    return false;
  }
  return true;
}

void
SF_ModelAbstractSegment::computeBranchOrder(int branchOrder)
{
  m_branchOrder = branchOrder;
  if (m_children.size() == 0) {
    m_children[0]->computeBranchOrder(branchOrder);
  }
  if (m_children.size() > 0) {
    m_children[0]->computeBranchOrder(branchOrder);
    std::for_each(m_children.begin() + 1, m_children.end(), [branchOrder](std::shared_ptr<SF_ModelAbstractSegment> child) {
      child->computeBranchOrder(branchOrder + 1);
    });
  }
}

void
SF_ModelAbstractSegment::computeReverseBranchOrder(int branchOrder)
{
  m_reversePipeBranchOrder = branchOrder;
  if (!isRoot()) {
    auto sibilings = getSiblings();
    bool allSibilingsComputed = true;
    std::for_each(sibilings.begin() + 1, sibilings.end(), [&allSibilingsComputed](std::shared_ptr<SF_ModelAbstractSegment> child) {
      if (child->getBranchOrder() == -1) {
        allSibilingsComputed = false;
      }
    });
    if (allSibilingsComputed) {
      auto parent = getParent();
      parent->computeReverseBranchOrder(branchOrder + 1);
    }
  }
}

void
SF_ModelAbstractSegment::computeReverseSummedBranchOrder(int branchOrder)
{
  m_reversePipeBranchOrder = branchOrder;
  if (!isRoot()) {
    auto sibilings = getSiblings();
    bool allSibilingsComputed = true;
    std::for_each(sibilings.begin() + 1, sibilings.end(), [&allSibilingsComputed](std::shared_ptr<SF_ModelAbstractSegment> child) {
      if (child->getBranchOrder() == -1) {
        allSibilingsComputed = false;
      }
    });
    if (allSibilingsComputed) {
      float sum = std::accumulate(sibilings.begin(), sibilings.end(), 0, [](int sum, std::shared_ptr<SF_ModelAbstractSegment> child) {
        return child->getReverseSummedBranchOrder() + sum;
      });
      auto parent = getParent();
      parent->computeReverseBranchOrder(++sum);
    }
  }
}

void
SF_ModelAbstractSegment::computeReversePipeBranchOrder(int branchOrder)
{
  m_reversePipeBranchOrder = branchOrder;
  if (!isRoot()) {
    auto sibilings = getSiblings();
    bool allSibilingsComputed = true;
    std::for_each(sibilings.begin() + 1, sibilings.end(), [&allSibilingsComputed](std::shared_ptr<SF_ModelAbstractSegment> child) {
      if (child->getBranchOrder() == -1) {
        allSibilingsComputed = false;
      }
    });
    if (allSibilingsComputed) {
      float sum = std::accumulate(sibilings.begin(), sibilings.end(), 0, [](int sum, std::shared_ptr<SF_ModelAbstractSegment> child) {
        return child->getReversePipeBranchOrder() * child->getReversePipeBranchOrder() + sum;
      });
      auto parent = getParent();
      parent->computeReverseBranchOrder(std::sqrt(sum));
    }
  }
}

void
SF_ModelAbstractSegment::initializeOrder()
{
  m_reverseBranchOrder = -1;
  m_reverseSummedBranchOrder - 1;
  m_reversePipeBranchOrder = -1;
  m_branchOrder = -1;
  std::for_each(
    m_children.begin(), m_children.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> child) { child->initializeOrder(); });
}

int
SF_ModelAbstractSegment::getID() const
{
  return m_ID;
}

void
SF_ModelAbstractSegment::setID(int ID)
{
  m_ID = ID;
}

std::vector<std::shared_ptr<SF_ModelAbstractSegment>>
SF_ModelAbstractSegment::getChildren() const
{
  return m_children;
}

std::vector<std::shared_ptr<SF_ModelAbstractSegment>>
SF_ModelAbstractSegment::getSiblings()
{
  if (!isRoot()) {
    auto parent = getParent();
    return parent->getChildren();
  }
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> children;
  children.push_back(shared_from_this());
  return children;
}

std::shared_ptr<SF_ModelQSM>
SF_ModelAbstractSegment::getTree() const
{
  return m_tree.lock();
}

void
SF_ModelAbstractSegment::setChildSegments(const std::vector<std::shared_ptr<SF_ModelAbstractSegment>> childSegments)
{
  m_children.clear();
  std::for_each(childSegments.begin(), childSegments.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> childSegment) {
    addChild(childSegment);
  });
}

void
SF_ModelAbstractSegment::setBuildingBricks(const std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>& buildingBricks)
{
  m_buildingBricks.clear();
  std::for_each(buildingBricks.begin(), buildingBricks.end(), [this](std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick) {
    addBuildingBrick(buildingBrick);
  });
}

void
SF_ModelAbstractSegment::sort(const SF_ModelAbstractSegment::SF_SORTTYPE sortType)
{
  std::sort(m_children.begin(),
            m_children.end(),
            [sortType](std::shared_ptr<SF_ModelAbstractSegment> child1, std::shared_ptr<SF_ModelAbstractSegment> child2) {
              switch (sortType) {
                case SF_SORTTYPE::GROWTH_LENGTH:
                  return (child1->getGrowthLength() > child2->getGrowthLength());
                  break;
                case SF_SORTTYPE::GROWTH_VOLUME:
                  return (child1->getGrowthVolume() > child2->getGrowthVolume());
                  break;
                case SF_SORTTYPE::RADIUS:
                  return (child1->getRadius() > child2->getRadius());
                  break;
                default:
                  std::cout << "Tree sorting with illegal sort type called." << std::endl;
                  break;
              }
              return true;
            });
}

int
SF_ModelAbstractSegment::getBranchOrder() const
{
  return m_branchOrder;
}

void
SF_ModelAbstractSegment::setBranchOrder(int branchOrder)
{
  m_branchOrder = branchOrder;
}

int
SF_ModelAbstractSegment::getReverseBranchOrder() const
{
  return m_reverseBranchOrder;
}

void
SF_ModelAbstractSegment::setReverseBranchOrder(int reverseBranchOrder)
{
  m_reverseBranchOrder = reverseBranchOrder;
}

int
SF_ModelAbstractSegment::getReversePipeBranchOrder() const
{
  return m_reversePipeBranchOrder;
}

void
SF_ModelAbstractSegment::setReversePipeBranchOrder(int reversePipeBranchOrder)
{
  m_reversePipeBranchOrder = reversePipeBranchOrder;
}

int
SF_ModelAbstractSegment::getReverseSummedBranchOrder() const
{
  return m_reverseSummedBranchOrder;
}

void
SF_ModelAbstractSegment::setReverseSummedBranchOrder(int reverseSummedBranchOrder)
{
  m_reverseSummedBranchOrder = reverseSummedBranchOrder;
}

int
SF_ModelAbstractSegment::getParentID()
{
  std::shared_ptr<SF_ModelAbstractSegment> parent = getParent();
  if (parent == nullptr) {
    return -1;
  } else {
    return parent->getID();
  }
}

void
SF_ModelAbstractSegment::addChild(std::shared_ptr<SF_ModelAbstractSegment> child)
{
  child->setParent(shared_from_this());
  m_children.push_back(child);
}

void
SF_ModelAbstractSegment::addBuildingBrick(std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick)
{
  buildingBrick->setIndex(m_buildingBricks.size());
  buildingBrick->setSegment(shared_from_this());
  m_buildingBricks.push_back(buildingBrick);
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_ModelAbstractSegment::getChildBuildingBricks(const size_t index)
{
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> children;
  if (index < m_buildingBricks.size() - 1) {
    children.push_back(m_buildingBricks.at(index + 1));
  } else if (index == m_buildingBricks.size() - 1) {
    for (size_t i = 0; i < m_children.size(); i++) {
      std::shared_ptr<Sf_ModelAbstractBuildingbrick> child = m_children.at(i)->getBuildingBricks()[0];
      children.push_back(child);
    }
  }
  return children;
}

std::shared_ptr<Sf_ModelAbstractBuildingbrick>
SF_ModelAbstractSegment::getParentBuildingBrick(const size_t index)
{
  std::shared_ptr<Sf_ModelAbstractBuildingbrick> parent;
  if (index == 0) {
    std::shared_ptr<SF_ModelAbstractSegment> parentSegment = getParent();
    if (parentSegment != nullptr) {
      parent = parentSegment->getBuildingBricks()[parentSegment->getBuildingBricks().size() - 1];
    }
  } else {
    parent = m_buildingBricks[index - 1];
  }
  return parent;
}

SF_ModelAbstractSegment::SF_ModelAbstractSegment(std::shared_ptr<SF_ModelQSM> tree) : m_tree(tree)
{
  m_ID = -1;
  m_branchOrder = -1;
  m_reverseBranchOrder = -1;
  m_reversePipeBranchOrder = -1;
  m_branchID = -1;
}
