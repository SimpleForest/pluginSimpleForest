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

#include "sf_sortqsm.h"

void SF_SortQSM::sortRecursively(std::shared_ptr<SF_ModelAbstractSegment> segment)
{
    std::vector<std::shared_ptr<SF_ModelAbstractSegment> > childSegments = segment->getChildSegments();
    std::sort(childSegments.begin(), childSegments.end(), [](std::shared_ptr<SF_ModelAbstractSegment> seg1,
                                                             std::shared_ptr<SF_ModelAbstractSegment> seg2){
        return (seg1->getBuildingBricks()[0]->getGrowthLength() > seg2->getBuildingBricks()[0]->getGrowthLength());
    });
    segment->setChildSegments(childSegments);
    std::for_each(childSegments.begin(), childSegments.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> seg){
           sortRecursively(seg)    ;
    });
}

SF_SortQSM::SF_SortQSM()
{

}

void SF_SortQSM::compute(std::shared_ptr<SF_ModelQSM> qsm)
{
    m_qsm = qsm;
    std::shared_ptr<SF_ModelAbstractSegment> segment = qsm->getRootSegment();
    sortRecursively(segment);
}
