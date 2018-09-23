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

#include "sf_abstractFilterStep.h"

SF_AbstractFilterStep::SF_AbstractFilterStep(CT_StepInitializeData &data_init):
    SF_AbstractStep(data_init) {

}

void SF_AbstractFilterStep::addSceneInSubgrpToGrp(CT_StandardItemGroup *filterGrp,
                                                  CT_ResultGroup *outResult,
                                                  CT_PointCloudIndexVector *ctPointCloudIndex,
                                                  const QString &outCloudCompleteName,
                                                  const QString &subGrpCompleteName) {
    CT_StandardItemGroup* cloudGrp = new CT_StandardItemGroup(subGrpCompleteName,
                                                              outResult);
    filterGrp->addGroup(cloudGrp);
    CT_Scene* outScene = new CT_Scene(outCloudCompleteName,
                                      outResult,
                                      PS_REPOSITORY->registerPointCloudIndex(ctPointCloudIndex));
    outScene->updateBoundingBox();
    cloudGrp->addItemDrawable(outScene);
}

void SF_AbstractFilterStep::addSceneToFilterGrp(CT_StandardItemGroup *grp,
                                                CT_ResultGroup *outResult,
                                                CT_PointCloudIndexVector *ctPointCloudIndex,
                                                const QString &outCloudCompleteName) {
    CT_Scene* outScene = new CT_Scene(outCloudCompleteName,
                                      outResult,
                                      PS_REPOSITORY->registerPointCloudIndex(ctPointCloudIndex));
    outScene->updateBoundingBox();
    grp->addItemDrawable(outScene);
}
