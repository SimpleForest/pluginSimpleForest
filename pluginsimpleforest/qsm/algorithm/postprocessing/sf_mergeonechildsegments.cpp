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

#include "sf_mergeonechildsegments.h"

#include <iterator>

void
SF_MergeOneChildSegments::mergeRecursively(std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  while (segment->getChildSegments().size() == 1) {
    std::shared_ptr<SF_ModelAbstractSegment> segmentChild = segment->getChildSegments()[0];
    std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segmentsGrandChildren = segmentChild->getChildSegments();
    std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = segment->getBuildingBricks();
    std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricksChild = segmentChild->getBuildingBricks();
    buildingBricks.insert(
      buildingBricks.end(), std::make_move_iterator(buildingBricksChild.begin()), std::make_move_iterator(buildingBricksChild.end()));
    segment->setBuildingBricks(buildingBricks);
    segmentChild->remove();
    segment->setChildSegments(segmentsGrandChildren);
  }
}

SF_MergeOneChildSegments::SF_MergeOneChildSegments() {}

void
SF_MergeOneChildSegments::compute(std::shared_ptr<SF_ModelQSM> qsm)
{
  m_qsm = qsm;
  std::shared_ptr<SF_ModelAbstractSegment> segment = qsm->getRootSegment();
  mergeRecursively(segment);
}
