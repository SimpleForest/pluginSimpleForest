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

void SF_ModelQSM::setBranchorder()
{
    m_rootSegment->initializeOrder();
    m_rootSegment->computeBranchOrder(0);
    while(m_rootSegment->getReverseBranchOrder() == -1)
    {
        std::vector<std::shared_ptr<SF_ModelAbstractSegment>> leaves = getLeaveSegments();
        std::for_each(leaves.begin(), leaves.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> leaf) {
            leaf->computeReverseBranchOrder(1);
        });
    }
    while(m_rootSegment->getReversePipeBranchOrder() == -1)
    {
        std::vector<std::shared_ptr<SF_ModelAbstractSegment>> leaves = getLeaveSegments();
        for(auto leaf : leaves) {
            leaf->computeReversePipeBranchOrder(1);
        }
    }
    while(m_rootSegment->getReverseSummedBranchOrder() == -1)
    {
        std::vector<std::shared_ptr<SF_ModelAbstractSegment>> leaves = getLeaveSegments();
        for(auto leaf : leaves) {
            leaf->computeReverseSummedBranchOrder(1);
        }
    }
}

void SF_ModelQSM::sort(SF_ModelAbstractSegment::SF_SORTTYPE type)
{
    m_rootSegment->sort(type);
}

SF_ModelQSM::SF_ModelQSM(const int ID) : _ID(ID), _species("unknownSpecies") {}

std::string
SF_ModelQSM::toString()
{
  std::string str(std::to_string(_ID));
  str.append(", ");
  str.append(_species);
  str.append("\n");
  return str;
}

std::string
SF_ModelQSM::toHeaderString()
{
  std::string str("treeID, treeSpecies");
  str.append("\n");
  return str;
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
