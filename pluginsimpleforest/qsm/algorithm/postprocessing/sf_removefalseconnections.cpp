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

#include "pcl/sf_math.h"

SF_RemoveFalseConnections::SF_RemoveFalseConnections() {}

void
SF_RemoveFalseConnections::compute(std::shared_ptr<SF_ModelQSM> qsm)
{
  m_qsm = qsm;
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> leafes = m_qsm->getLeaveSegments();
  std::for_each(leafes.begin(), leafes.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> leaf) {
    if (leaf->getBuildingBricks().size() <= 1) {
      leaf->remove();
    } else {
      if (!leaf->isRoot()) {
        std::shared_ptr<SF_ModelAbstractSegment> parent = leaf->getParent();
        std::vector<std::shared_ptr<SF_ModelAbstractSegment>> children = parent->getChildSegments();
        std::for_each(children.begin(), children.end(), [&leaf, this](std::shared_ptr<SF_ModelAbstractSegment> child) {
          if (child != leaf) {
            float angle = SF_Math<float>::getAngleBetweenDeg(leaf->getAxis(), child->getAxis());
            if (angle < _M_MAXANGLE || angle > (180 - _M_MAXANGLE)) {
              child->remove();
            }
          }
        });
      }
    }
  });
  SF_MergeOneChildSegments moc;
  moc.compute(m_qsm);
}
