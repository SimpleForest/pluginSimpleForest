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

std::shared_ptr<SF_ModelAbstractSegment> SF_ModelQSM::getRootSegment() const {
  return _rootSegment;
}

void SF_ModelQSM::setRootSegment(
    const std::shared_ptr<SF_ModelAbstractSegment> &rootSegment) {
  _rootSegment = rootSegment;
}

SF_ModelQSM::SF_ModelQSM(const int ID) : _ID(ID), _species("unknownSpecies") {}

std::string SF_ModelQSM::toString() {
  std::string str(std::to_string(_ID));
  str.append(", ");
  str.append(_species);
  str.append("\n");
  return str;
}

std::string SF_ModelQSM::toHeaderString() {
  std::string str("treeID, treeSpecies");
  str.append("\n");
  return str;
}

std::vector<std::shared_ptr<SF_ModelAbstractSegment>>
SF_ModelQSM::getSegments() {
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments;
  if (_rootSegment != nullptr) {
    segments = getSegments(_rootSegment);
  }
  return segments;
}

std::vector<std::shared_ptr<SF_ModelAbstractSegment>>
SF_ModelQSM::getSegments(std::shared_ptr<SF_ModelAbstractSegment> segment) {
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments;
  segments.push_back(segment);
  for (size_t i = 0; i < segment->getChildSegments().size(); i++) {
    std::shared_ptr<SF_ModelAbstractSegment> child =
        segment->getChildSegments()[i];
    std::vector<std::shared_ptr<SF_ModelAbstractSegment>> childSegments =
        getSegments(child);
    segments.insert(segments.end(), childSegments.begin(), childSegments.end());
  }
  return segments;
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_ModelQSM::getBuildingBricks() {
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments =
      getSegments();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks;
  for (size_t i = 0; i < segments.size(); i++) {
    std::shared_ptr<SF_ModelAbstractSegment> segment = segments[i];
    std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
        segmentBuildingBricks = segment->getBuildingBricks();
    for (size_t j = 0; j < segmentBuildingBricks.size(); j++) {
      std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick =
          segmentBuildingBricks[j];
      buildingBricks.push_back(buildingBrick);
    }
  }
  return buildingBricks;
}
