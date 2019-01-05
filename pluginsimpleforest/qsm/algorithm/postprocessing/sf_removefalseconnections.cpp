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

SF_RemoveFalseConnections::SF_RemoveFalseConnections() {
}

void SF_RemoveFalseConnections::compute(std::shared_ptr<SF_ModelQSM> qsm) {
    m_qsm = qsm;
    std::vector<std::shared_ptr<SF_ModelAbstractSegment>> leafes = m_qsm->getLeaveSegments();
    std::for_each(leafes.begin(), leafes.end(), [](std::shared_ptr<SF_ModelAbstractSegment> leaf){
        if(leaf->getBuildingBricks().size() == 1) {
            leaf->remove();
        }
    });
}
