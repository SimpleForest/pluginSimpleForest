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

#include "sf_removefalseconnections.h"

#include "sf_mergeonechildsegments.h"
#include "sf_sortqsm.h"

#include "pcl/sf_math.h"

SF_RemoveFalseConnections::SF_RemoveFalseConnections() {}

void
SF_RemoveFalseConnections::compute(std::shared_ptr<SF_ModelQSM> qsm)
{
  m_qsm = qsm;
  SF_SortQSM sq;
  sq.compute(m_qsm);
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> leafes = m_qsm->getLeaveSegments();
  std::for_each(leafes.begin(), leafes.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> leaf) {
    if (leaf->getBuildingBricks().size() <= 1) {
      leaf->remove();
    }
  });
  SF_MergeOneChildSegments moc;
  moc.compute(m_qsm);
  sq.compute(m_qsm);

  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments = m_qsm->getSegments();
  std::for_each(segments.begin(), segments.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> segment) {
    if (!segment->isRoot()) {
      std::shared_ptr<SF_ModelAbstractSegment> parent = segment->getParent();
      std::vector<std::shared_ptr<SF_ModelAbstractSegment>> children = parent->getChildSegments();
      std::for_each(children.begin(), children.end(), [&segment, this](std::shared_ptr<SF_ModelAbstractSegment> child) {
        if (child != segment) {
          float angle = SF_Math<float>::getAngleBetweenDeg(segment->getAxis(), child->getAxis());
          if (angle < _M_MAXANGLE || angle > (180 - _M_MAXANGLE)) {
            if (segment->getBuildingBricks()[0]->getGrowthLength() > child->getBuildingBricks()[0]->getGrowthLength()) {
              child->remove();
            } else {
              segment->remove();
            }
          }
        }
      });
    }
  });
  moc.compute(m_qsm);
  sq.compute(m_qsm);
}
