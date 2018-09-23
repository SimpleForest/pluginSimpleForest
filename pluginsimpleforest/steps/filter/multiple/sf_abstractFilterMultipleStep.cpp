/****************************************************************************

 Copyright (C) 2017-2017 Jan Hackenberg, free software developer
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

#include "sf_abstractFilterMultipleStep.h"

SF_AbstractFilterMultipleStep::SF_AbstractFilterMultipleStep(CT_StepInitializeData &data_init):
    SF_AbstractFilterStep(data_init) {

}

void SF_AbstractFilterMultipleStep::writeOutputPerScence(CT_ResultGroup* outResult,
                                                         CT_PointCloudIndexVector *outputCluster,
                                                         CT_StandardItemGroup* group) {
    CT_StandardItemGroup* filter_grp = new CT_StandardItemGroup(_outGrp.completeName(),
                                                                outResult);
    group->addGroup(filter_grp);
    addSceneInSubgrpToGrp(filter_grp,
                          outResult,
                          outputCluster,
                          _outCloudCluster.completeName(),
                          _outGrpCluster.completeName());
}

void SF_AbstractFilterMultipleStep::writeOutput(CT_ResultGroup *outResult,
                                                std::vector<CT_PointCloudIndexVector *> clusterVec,
                                                CT_StandardItemGroup* group) {
    size_t size = clusterVec.size();
    for(size_t i = 0; i < size; i ++) {
        writeOutputPerScence(outResult, clusterVec.at(i), group);
    }
}
