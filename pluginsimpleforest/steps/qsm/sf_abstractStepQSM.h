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
  void addQSM(CT_ResultGroup* outResult, QList<T> paramList, QString outResultGrpName, QString outSFQSMName, QString inCloudName);
  template<typename T>
  void addQSM(CT_ResultGroup* outResult,
              QList<T> paramList,
              QString outResultGrpName,
              QString outSFQSMName,
              QString outParamName,
              QString inCloudName);
  void addQSMToOutResult(CT_OutResultModelGroupToCopyPossibilities* resModelw, QString header, QString group);

  CT_AutoRenameModels _treeGroup;
  CT_AutoRenameModels _branchNodeGroup;
  CT_AutoRenameModels _stemNodeGroup;
  CT_AutoRenameModels _branchNodeCylinders;
  CT_AutoRenameModels _stemNodeCylinders;
  CT_AutoRenameModels _QSMCloud;
  CT_AutoRenameModels _QSMGrp;
};

template<typename T>
void
SF_AbstractStepQSM::addQSM(CT_ResultGroup* outResult,
                           QList<T> paramList,
                           QString outResultGrpName,
                           QString outSFQSMName,
                           QString inCloudName)
{
  CT_ResultGroupIterator outResIt(outResult, this, outResultGrpName);
  size_t index = 0;
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* qsmGrp = new CT_StandardItemGroup(_QSMGrp.completeName(), outResult);
    T& params = paramList[index++];
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();

    std::shared_ptr<SF_ModelQSM> qsm = params._qsm;
    qsm->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME);

    CT_TTreeGroup* tree = constructTopology(outResult, qsm);
    group->addGroup(qsmGrp);
    qsmGrp->addGroup(tree);

    if (inCloudName != "") {
      const CT_AbstractItemDrawableWithPointCloud* ctCloud =
        (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(this, inCloudName);
      CT_PointCloudIndexVector* pointCloudIndexVector = new CT_PointCloudIndexVector();
      pointCloudIndexVector->setSortType(CT_AbstractCloudIndex::NotSorted);
      CT_PointIterator iter(ctCloud->getPointCloudIndex());
      std::vector<size_t> indices;
      while (iter.hasNext() && !isStopped()) {
        iter.next();
        size_t index = iter.currentGlobalIndex();
        indices.push_back(index);
      }
      std::sort(indices.begin(), indices.end());
      for (size_t i = 0; i < indices.size(); i++) {
        pointCloudIndexVector->addIndex(indices.at(i));
      }
      pointCloudIndexVector->setSortType(CT_PointCloudIndexVector::SortedInAscendingOrder);
      CT_Scene* outScene = new CT_Scene(
        _QSMCloud.completeName(), outResult, PS_REPOSITORY->registerPointCloudIndex(pointCloudIndexVector));
      outScene->updateBoundingBox();
      qsmGrp->addItemDrawable(outScene);
    }

    SF_QSM_Item* qsmItem = new SF_QSM_Item(outSFQSMName, outResult, qsm);
    qsmGrp->addItemDrawable(qsmItem);
    params.reset();
  }
}

template<typename T>
void
SF_AbstractStepQSM::addQSM(CT_ResultGroup* outResult,
                           QList<T> paramList,
                           QString outResultGrpName,
                           QString outSFQSMName,
                           QString outParamName,
                           QString inCloudName)
{
  CT_ResultGroupIterator outResIt(outResult, this, outResultGrpName);
  size_t index = 0;
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* qsmGrp = new CT_StandardItemGroup(_QSMGrp.completeName(), outResult);
    T& params = paramList[index++];
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    std::shared_ptr<SF_ModelQSM> qsm = params._qsm;
    qsm->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME);

    CT_TTreeGroup* tree = constructTopology(outResult, qsm);
    group->addGroup(qsmGrp);
    qsmGrp->addGroup(tree);
    if (inCloudName != "") {
      const CT_AbstractItemDrawableWithPointCloud* ctCloud =
        (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(this, inCloudName);
      CT_PointCloudIndexVector* pointCloudIndexVector = new CT_PointCloudIndexVector();
      pointCloudIndexVector->setSortType(CT_AbstractCloudIndex::NotSorted);
      CT_PointIterator iter(ctCloud->getPointCloudIndex());
      std::vector<size_t> indices;
      while (iter.hasNext() && !isStopped()) {
        iter.next();
        size_t index = iter.currentGlobalIndex();
        indices.push_back(index);
      }
      std::sort(indices.begin(), indices.end());
      for (size_t i = 0; i < indices.size(); i++) {
        pointCloudIndexVector->addIndex(indices.at(i));
      }
      pointCloudIndexVector->setSortType(CT_PointCloudIndexVector::SortedInAscendingOrder);
      CT_Scene* outScene = new CT_Scene(
        _QSMCloud.completeName(), outResult, PS_REPOSITORY->registerPointCloudIndex(pointCloudIndexVector));
      outScene->updateBoundingBox();
      qsmGrp->addItemDrawable(outScene);
    }
    if (outParamName != "") {
      SF_SphereFollowing_Parameters_Item* paramItem = new SF_SphereFollowing_Parameters_Item(outParamName, outResult, params);
      qsmGrp->addItemDrawable(paramItem);
    }
    SF_QSM_Item* qsmItem = new SF_QSM_Item(outSFQSMName, outResult, qsm);
    qsmGrp->addItemDrawable(qsmItem);
    params.reset();
  }
}

#endif // SF_ABSTRACTSTEPQSM_H
