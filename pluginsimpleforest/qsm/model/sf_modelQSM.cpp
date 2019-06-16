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

#include "sf_modelQSM.h"

std::shared_ptr<SF_ModelAbstractSegment>
SF_ModelQSM::getRootSegment() const
{
  return m_rootSegment;
}

void
SF_ModelQSM::setRootSegment(const std::shared_ptr<SF_ModelAbstractSegment>& rootSegment)
{
  m_rootSegment = rootSegment;
}

double
SF_ModelQSM::getVolume()
{
  m_rootSegment->getVolume();
  auto volumeQSM = m_rootSegment->getBuildingBricks().front()->getGrowthVolume();
  return volumeQSM + m_volumeCorrection;
}

void
SF_ModelQSM::setBranchorder(float twigPercentage)
{
  m_rootSegment->initializeOrder();
  m_rootSegment->computeBranchOrder(0);
  auto segments = getSegments();
  size_t index = 0;
  for (auto segment : segments) {
    segment->setID(index++);
  }
  index = 0;
  auto bricks = getBuildingBricks();
  for (auto brick : bricks) {
    brick->setID(index++);
  }

  while (m_rootSegment->getReverseBranchOrder() == -1) {
    std::vector<std::shared_ptr<SF_ModelAbstractSegment>> leaves = getLeaveSegments();
    std::for_each(
      leaves.begin(), leaves.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> leaf) { leaf->computeReverseBranchOrder(1); });
  }

  while (m_rootSegment->getReverseSummedBranchOrder() == -1) {
    std::vector<std::shared_ptr<SF_ModelAbstractSegment>> leaves = getLeaveSegments();
    for (auto leaf : leaves) {
      leaf->computeReverseSummedBranchOrder(1);
    }
  }

  index = 1;
  int indexTwig = -2;

  auto totalVolume = getVolume();
  for (auto segment : segments) {
    if (segment->getBranchOrder() == 0) {
      auto children = segment->getChildren();
      for (auto child : children) {
        if (child->getBranchOrder() != 0) {
          auto childVolume = child->getVolume();
          if (totalVolume == 0) {
            throw(std::logic_error("QSM has no Volume in branch ordering causing division by zero."));
          } else {
            auto fraction = childVolume / totalVolume;
            if (fraction < twigPercentage) {
              child->computeBranchID(--indexTwig);
            } else {
              child->computeBranchID(++index);
            }
          }
        }
      }
    }
  }
}

void
SF_ModelQSM::sort(SF_ModelAbstractSegment::SF_SORTTYPE type, float twigPercentage)
{
  m_rootSegment->sort(type);
  setBranchorder(twigPercentage);
}

bool
SF_ModelQSM::getHasCorrectedParameters() const
{
  return m_hasCorrectedParameters;
}

void
SF_ModelQSM::setHasCorrectedParameters(bool hasCorrectedParameters)
{
  m_hasCorrectedParameters = hasCorrectedParameters;
}

double
SF_ModelQSM::getAGrowthLength() const
{
  return m_aGrowthLength;
}

void
SF_ModelQSM::setAGrowthLength(double aGrowthLength)
{
  m_aGrowthLength = aGrowthLength;
}

double
SF_ModelQSM::getBGrowthLength() const
{
  return m_bGrowthLength;
}

void
SF_ModelQSM::setBGrowthLength(double bGrowthLength)
{
  m_bGrowthLength = bGrowthLength;
}

double
SF_ModelQSM::getCGrowthLength() const
{
  return m_cGrowthLength;
}

void
SF_ModelQSM::setCGrowthLength(double cGrowthLength)
{
  m_cGrowthLength = cGrowthLength;
}

double
SF_ModelQSM::getAGrowthVolume() const
{
  return m_aGrowthVolume;
}

void
SF_ModelQSM::setAGrowthVolume(double aGrowthVolume)
{
  m_aGrowthVolume = aGrowthVolume;
}

double
SF_ModelQSM::getBGrowthVolume() const
{
  return m_bGrowthVolume;
}

void
SF_ModelQSM::setBGrowthVolume(double bGrowthVolume)
{
  m_bGrowthVolume = bGrowthVolume;
}

double
SF_ModelQSM::getCGrowthVolume() const
{
  return m_cGrowthVolume;
}

void
SF_ModelQSM::setCGrowthVolume(double cGrowthVolume)
{
  m_cGrowthVolume = cGrowthVolume;
}

int
SF_ModelQSM::getID() const
{
  return _ID;
}

void
SF_ModelQSM::setID(int ID)
{
  _ID = ID;
}

void
SF_ModelQSM::setTranslation(const Eigen::Vector3d& translation)
{
  m_translation = translation;
}

SF_ModelQSM::SF_ModelQSM(const int ID) : _ID(ID), _species("unknownSpecies") {}

std::string
SF_ModelQSM::toString()
{
  std::string str(std::to_string(_ID));
  str.append(", ");
  str.append(_species);
  str.append(", ");
  str.append(std::to_string(m_translation.coeff(0)));
  str.append(", ");
  str.append(std::to_string(m_translation.coeff(1)));
  str.append(", ");
  str.append(std::to_string(m_translation.coeff(2)));
  str.append(", ");
  str.append(std::to_string(m_aGrowthVolume));
  str.append(", ");
  str.append(std::to_string(m_bGrowthVolume));
  str.append(", ");
  str.append(std::to_string(m_cGrowthVolume));
  str.append(", ");
  str.append(std::to_string(m_aGrowthLength));
  str.append(", ");
  str.append(std::to_string(m_bGrowthLength));
  str.append(", ");
  str.append(std::to_string(m_cGrowthLength));
  str.append("\n");
  return str;
}

std::string
SF_ModelQSM::toHeaderString()
{
  std::string str("treeID, treeSpecies, translateX, translateY, translatez, gvA, gvB, gvC, glA, glB, glC");
  str.append("\n");
  return str;
}

void
SF_ModelQSM::translate(const Eigen::Vector3d& translation)
{
  if (m_translation == -translation) {
    m_translation = Eigen::Vector3d(0, 0, 0);
  } else {
    m_translation = translation;
  }
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = getBuildingBricks();
  for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick : buildingBricks) {
    buildingBrick->translate(translation);
  }
}

void
SF_ModelQSM::transform(const Eigen::Affine3f& transform)
{
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = getBuildingBricks();
  for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick : buildingBricks) {
    buildingBrick->transform(transform);
  }
}

Eigen::Vector3d
SF_ModelQSM::translateToOrigin()
{
  Eigen::Vector3d translation(0, 0, 0);
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = getBuildingBricks();
  if (buildingBricks.empty()) {
    return translation;
  }
  for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick : buildingBricks) {
    translation += buildingBrick->getCenter();
  }
  translation /= buildingBricks.size();
  translate(translation);
  return translation;
}

std::shared_ptr<SF_ModelAbstractSegment>
SF_ModelQSM::crownStartSegment(double minPercentage)
{
  auto bricks = getBuildingBricks();
  double minZ = std::numeric_limits<double>::max();
  double maxZ = std::numeric_limits<double>::lowest();
  std::for_each(bricks.begin(), bricks.end(), [&minZ, &maxZ](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) {
    auto center = brick->getCenter();
    auto z = center[2];
    if (z < minZ)
      minZ = z;
    if (z > maxZ)
      maxZ = z;
  });
  auto centerHeight = minZ + (maxZ - minZ) / 2;

  auto segment = m_rootSegment;
  while (segment && segment->getChildren().size() > 0) {
    auto children = segment->getChildren();
    segment = children[0];
    if (children.size() >= 2) {
      double totalLengthStem = 0;
      double totalLengthBranches = 0;
      totalLengthStem = children[0]->getBuildingBricks()[0]->getGrowthLength();
      for (size_t i = 1; i < children.size(); i++) {
        totalLengthBranches += children[i]->getBuildingBricks()[0]->getGrowthLength();
      }
      double totalLengthAll = totalLengthBranches + totalLengthStem;
      double fraction = 0;
      if (totalLengthAll > 0) {
        fraction = totalLengthBranches / totalLengthAll;
      }
      if (fraction > minPercentage) {
        break;
      }
      if (segment->getStart()[2] < centerHeight) {
        break;
      }
    }
  }
  return segment;
}

std::shared_ptr<Sf_ModelAbstractBuildingbrick>
SF_ModelQSM::crownStartBrick(double minPercentage)
{
  auto segment = crownStartSegment(minPercentage);
  return segment->getBuildingBricks().back();
}

std::vector<std::shared_ptr<SF_ModelAbstractSegment>>
SF_ModelQSM::getSegments()
{
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments;
  if (m_rootSegment != nullptr) {
    segments = getSegments(m_rootSegment);
  }
  return segments;
}

std::vector<std::shared_ptr<SF_ModelAbstractSegment>>
SF_ModelQSM::getSegments(std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments;
  segments.push_back(segment);
  for (size_t i = 0; i < segment->getChildren().size(); i++) {
    std::shared_ptr<SF_ModelAbstractSegment> child = segment->getChildren()[i];
    std::vector<std::shared_ptr<SF_ModelAbstractSegment>> childSegments = getSegments(child);
    segments.insert(segments.end(), childSegments.begin(), childSegments.end());
  }
  return segments;
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_ModelQSM::getBuildingBricks()
{
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments = getSegments();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks;
  for (size_t i = 0; i < segments.size(); i++) {
    std::shared_ptr<SF_ModelAbstractSegment> segment = segments[i];
    std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> segmentBuildingBricks = segment->getBuildingBricks();
    for (size_t j = 0; j < segmentBuildingBricks.size(); j++) {
      std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick = segmentBuildingBricks[j];
      buildingBricks.push_back(buildingBrick);
    }
  }
  return buildingBricks;
}

std::vector<std::shared_ptr<SF_ModelAbstractSegment>>
SF_ModelQSM::getLeaveSegments()
{
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> leafes;
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments = getSegments();
  std::for_each(segments.begin(), segments.end(), [&leafes](std::shared_ptr<SF_ModelAbstractSegment> segment) {
    if (segment->getChildren().size() == 0) {
      leafes.push_back(segment);
    }
  });
  return leafes;
}
