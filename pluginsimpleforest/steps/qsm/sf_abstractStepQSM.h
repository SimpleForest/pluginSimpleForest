/****************************************************************************

 Copyright (C) 2017-2019 Dr. Jan Hackenberg, free software developer
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

#ifndef SF_ABSTRACTSTEPQSM_H
#define SF_ABSTRACTSTEPQSM_H

#include "steps/sf_abstractStep.h"

class SF_AbstractStepQSM : public SF_AbstractStep
{
  Q_OBJECT
public:
  SF_AbstractStepQSM(CT_StepInitializeData& dataInit);

protected:
  CT_TTreeGroup* constructTopology(CT_AbstractResult* result, std::shared_ptr<SF_ModelQSM> qsm);
  void constructStemRecursively(const CT_AbstractResult* result,
                                CT_TNodeGroup* stemNode,
                                std::shared_ptr<SF_ModelAbstractSegment> segment);
  void constructBranchRecursively(const CT_AbstractResult* result,
                                  CT_TNodeGroup* branchNode,
                                  std::shared_ptr<SF_ModelAbstractSegment> segment);
  CT_CylinderData* constructCylinderData(std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick);
  void setCylindersStem(const CT_AbstractResult* result, CT_TNodeGroup* stemNode, std::shared_ptr<SF_ModelAbstractSegment> segment);
  void setCylindersBranch(const CT_AbstractResult* result,
                          CT_TNodeGroup* branchNode,
                          std::shared_ptr<SF_ModelAbstractSegment> segment);
  template<typename T>
  void addQSM(CT_ResultGroup* outResult, QList<T> paramList, QString outResultGrpName, QString outSFQSMName);
  template<typename T>
  void addQSM(CT_ResultGroup* outResult, QList<T> paramList, QString outResultGrpName, QString outSFQSMName, QString outParamName);
  void addQSMToOutResult(CT_OutResultModelGroupToCopyPossibilities* resModelw, QString header, QString group);

  CT_AutoRenameModels _treeGroup;
  CT_AutoRenameModels _branchNodeGroup;
  CT_AutoRenameModels _stemNodeGroup;
  CT_AutoRenameModels _branchNodeCylinders;
  CT_AutoRenameModels _stemNodeCylinders;
};

template<typename T>
void
SF_AbstractStepQSM::addQSM(CT_ResultGroup* outResult, QList<T> paramList, QString outResultGrpName, QString outSFQSMName)
{
  CT_ResultGroupIterator outResIt(outResult, this, outResultGrpName);
  size_t index = 0;
  while (!isStopped() && outResIt.hasNext()) {
    T& params = paramList[index++];
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    std::shared_ptr<SF_ModelQSM> qsm = params._qsm;
    qsm->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME);
    SF_QSM_Item* qsmItem = new SF_QSM_Item(outSFQSMName, outResult, qsm);
    group->addItemDrawable(qsmItem);
    params.reset();
    CT_TTreeGroup* tree = constructTopology(outResult, qsm);
    group->addGroup(tree);
  }
}

template<typename T>
void
SF_AbstractStepQSM::addQSM(CT_ResultGroup* outResult,
                           QList<T> paramList,
                           QString outResultGrpName,
                           QString outSFQSMName,
                           QString outParamName)
{
  CT_ResultGroupIterator outResIt(outResult, this, outResultGrpName);
  size_t index = 0;
  while (!isStopped() && outResIt.hasNext()) {
    T& params = paramList[index++];
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    std::shared_ptr<SF_ModelQSM> qsm = params._qsm;
    qsm->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME);
    SF_QSM_Item* qsmItem = new SF_QSM_Item(outSFQSMName, outResult, qsm);
    group->addItemDrawable(qsmItem);
    params.reset();
    if (outParamName != "") {
      SF_SphereFollowing_Parameters_Item* paramItem = new SF_SphereFollowing_Parameters_Item(outParamName, outResult, params);
      group->addItemDrawable(paramItem);
    }
    CT_TTreeGroup* tree = constructTopology(outResult, qsm);
    group->addGroup(tree);
  }
}

#endif // SF_ABSTRACTSTEPQSM_H
