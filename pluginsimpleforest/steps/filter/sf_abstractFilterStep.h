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
#ifndef SF_ABSTRACT_FILTER_STEP_H
#define SF_ABSTRACT_FILTER_STEP_H
#include <steps/sf_abstractStep.h>

class SF_AbstractFilterStep : public SF_AbstractStep
{
public:
  SF_AbstractFilterStep(CT_StepInitializeData& data_init);

protected:
  void addSceneInSubgrpToGrp(CT_StandardItemGroup* filterGrp,
                             CT_ResultGroup* outResult,
                             CT_PointCloudIndexVector* ctPointCloudIndex,
                             const QString& outCloudCompleteName,
                             const QString& subGrpCompleteName);
  void addSceneToFilterGrp(CT_StandardItemGroup* grp,
                           CT_ResultGroup* outResult,
                           CT_PointCloudIndexVector* ctPointCloudIndex,
                           const QString& outCloudCompleteName);
};

#endif // SF_ABSTRACT_FILTER_STEP_H
