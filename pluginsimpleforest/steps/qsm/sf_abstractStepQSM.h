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

#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "steps/sf_abstractStep.h"

class SF_AbstractStepQSM : public SF_AbstractStep
{
  Q_OBJECT
public:
  SF_AbstractStepQSM(CT_StepInitializeData& dataInit);

protected:
  void addOutputFormat(CT_StepConfigurableDialog* configDialog);
  CT_TTreeGroup* constructTopology(CT_AbstractResult* result, std::shared_ptr<SF_ModelQSM> qsm);
  void constructStemRecursively(const CT_AbstractResult* result,
                                CT_TNodeGroup* stemNode,
                                std::shared_ptr<SF_ModelAbstractSegment> segment);
  void constructTwigRecursively(const CT_AbstractResult* result,
                                CT_TNodeGroup* twigNode,
                                std::shared_ptr<SF_ModelAbstractSegment> segment);
  void constructBranchRecursively(const CT_AbstractResult* result,
                                  CT_TNodeGroup* branchNode,
                                  std::shared_ptr<SF_ModelAbstractSegment> segment);
  CT_CylinderData* constructCylinderData(std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick);
  void setCylindersStem(const CT_AbstractResult* result, CT_TNodeGroup* stemNode, std::shared_ptr<SF_ModelAbstractSegment> segment);
  void setCylindersBranch(const CT_AbstractResult* result,
                          CT_TNodeGroup* branchNode,
                          std::shared_ptr<SF_ModelAbstractSegment> segment);
  void setCylindersTwig(const CT_AbstractResult* result, CT_TNodeGroup* twigNode, std::shared_ptr<SF_ModelAbstractSegment> segment);
  template<typename T>
  void addQSM(CT_ResultGroup* outResult, QList<T> paramList, QString outResultGrpName, QString outSFQSMName);
  template<typename T>
  void addQSM(CT_ResultGroup* outResult,
              QList<T> paramList,
              QString outResultGrpName,
              QString outSFQSMName,
              QString outParamName);
  void addQSMToOutResult(CT_OutResultModelGroupToCopyPossibilities* resModelw, QString header, QString group);

  CT_AutoRenameModels _treeGroup;
  CT_AutoRenameModels _branchNodeGroup;
  CT_AutoRenameModels _twigNodeGroup;
  CT_AutoRenameModels _stemNodeGroup;
  CT_AutoRenameModels _branchNodeCylinders;
  CT_AutoRenameModels _stemNodeCylinders;
  CT_AutoRenameModels _twigNodeCylinders;
  CT_AutoRenameModels _QSMCloud;
  CT_AutoRenameModels _QSMGrp;

  void addAttributesStem(const CT_AbstractResult* result, CT_Cylinder* cylinder, std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick);
  void addAttributesBranch(const CT_AbstractResult* result,
                           CT_Cylinder* cylinder,
                           std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick);
  void addAttributesTwig(const CT_AbstractResult* result, CT_Cylinder* cylinder, std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick);

  CT_AutoRenameModels m_stemGrowthVolume;
  CT_AutoRenameModels m_stemGrowthLength;
  CT_AutoRenameModels m_stemBranchID;
  CT_AutoRenameModels m_stemReversePipeOrder;

  CT_AutoRenameModels m_branchGrowthVolume;
  CT_AutoRenameModels m_branchGrowthLength;
  CT_AutoRenameModels m_branchBranchID;
  CT_AutoRenameModels m_branchReversePipeOrder;

  CT_AutoRenameModels m_twigGrowthVolume;
  CT_AutoRenameModels m_twigGrowthLength;
  CT_AutoRenameModels m_twigBranchID;
  CT_AutoRenameModels m_twigPipeOrder;

  bool m_splitQSM = true;
  double m_twigPercentage = 0.04;
  double m_stemPercentage = 0.2;
};

template<typename T>
void
SF_AbstractStepQSM::addQSM(CT_ResultGroup* outResult,
                           QList<T> paramList,
                           QString outResultGrpName,
                           QString outSFQSMName)
{
  CT_ResultGroupIterator outResIt(outResult, this, outResultGrpName);
  size_t index = 0;
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* qsmGrp = new CT_StandardItemGroup(_QSMGrp.completeName(), outResult);
    T& params = paramList[index++];
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();

    std::shared_ptr<SF_ModelQSM> qsm = params._qsm;
    if (!qsm) {
      _groupsToBeRemoved.push_back(group);
      continue;
    }
    qsm->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME, m_twigPercentage);

    CT_TTreeGroup* tree = constructTopology(outResult, qsm);
    group->addGroup(qsmGrp);
    qsmGrp->addGroup(tree);

    SF_QSM_Item* qsmItem = new SF_QSM_Item(outSFQSMName, outResult, qsm);
    qsmGrp->addItemDrawable(qsmItem);
    params.reset();
  }
  removeCorruptedScenes();
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
    CT_StandardItemGroup* qsmGrp = new CT_StandardItemGroup(_QSMGrp.completeName(), outResult);
    T& params = paramList[index++];
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    std::shared_ptr<SF_ModelQSM> qsm = params._qsm;
    if (!qsm) {
      _groupsToBeRemoved.push_back(group);
      continue;
    }
    qsm->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME, m_twigPercentage);

    CT_TTreeGroup* tree = constructTopology(outResult, qsm);
    group->addGroup(qsmGrp);
    qsmGrp->addGroup(tree);
    if (outParamName != "") {
      SF_SphereFollowing_Parameters_Item* paramItem = new SF_SphereFollowing_Parameters_Item(outParamName, outResult, params);
      qsmGrp->addItemDrawable(paramItem);
    }
    SF_QSM_Item* qsmItem = new SF_QSM_Item(outSFQSMName, outResult, qsm);
    qsmGrp->addItemDrawable(qsmItem);
    params.reset();
  }
  removeCorruptedScenes();
}

#endif // SF_ABSTRACTSTEPQSM_H
