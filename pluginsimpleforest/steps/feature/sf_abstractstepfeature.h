/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_abstractstepfeature.h is part of SimpleForest - a plugin for the
    Computree platform.

    SimpleForest is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SimpleForest is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with
    SimpleForest. If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#ifndef SF_ABSTRACTSTEPFEATURE_H
#define SF_ABSTRACTSTEPFEATURE_H

#include "steps/sf_abstractStep.h"


class SF_AbstractStepFeature:
        public SF_AbstractStep
{
protected:
    CT_AutoRenameModels _outGrpCluster;
    CT_AutoRenameModels _outCloudCluster;
    void writeOutputPerScence(CT_ResultGroup* outResult,
                              CT_PointCloudIndexVector *outputCluster,
                              CT_StandardItemGroup *group);
    void writeOutput(CT_ResultGroup* outResult,
                     std::vector<CT_PointCloudIndexVector *> clusterVec,
                     CT_StandardItemGroup *group);

public:
    SF_AbstractStepFeature(CT_StepInitializeData &dataInit);
};

#endif // SF_ABSTRACTSTEPFEATURE_H
