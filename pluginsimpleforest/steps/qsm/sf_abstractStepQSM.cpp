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

#include "sf_abstractStepQSM.h"

SF_AbstractStepQSM::SF_AbstractStepQSM(CT_StepInitializeData& dataInit) : SF_AbstractStep(dataInit) {}

CT_TTreeGroup*
SF_AbstractStepQSM::constructTopology(CT_AbstractResult* result, std::shared_ptr<SF_ModelQSM> qsm)
{
  std::shared_ptr<SF_ModelAbstractSegment> sfRoot = qsm->getRootSegment();
  CT_TTreeGroup* tree = new CT_TTreeGroup(_treeGroup.completeName(), result);
  CT_TNodeGroup* root = new CT_TNodeGroup(_stemNodeGroup.completeName(), result);
  tree->setRootNode(root);
  setCylindersStem(result, root, sfRoot);
  constructStemRecursively(result, root, sfRoot);
  return tree;
}

void
SF_AbstractStepQSM::constructStemRecursively(const CT_AbstractResult* result,
                                             CT_TNodeGroup* stemNode,
                                             std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> children = segment->getChildren();
  int index = 0;
  for (std::shared_ptr<SF_ModelAbstractSegment> child : children) {
    if (index == 0) {
      CT_TNodeGroup* stemChild = new CT_TNodeGroup(_stemNodeGroup.completeName(), result);
      stemNode->addBranch(stemChild);
      setCylindersStem(result, stemChild, child);
      constructStemRecursively(result, stemChild, child);
    } else {
      CT_TNodeGroup* branchChild = new CT_TNodeGroup(_branchNodeGroup.completeName(), result);
      stemNode->addBranch(branchChild);
      setCylindersBranch(result, branchChild, child);
      constructBranchRecursively(result, branchChild, child);
    }
    index++;
  }
}

void
SF_AbstractStepQSM::constructBranchRecursively(const CT_AbstractResult* result,
                                               CT_TNodeGroup* branchNode,
                                               std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> children = segment->getChildren();
  for (std::shared_ptr<SF_ModelAbstractSegment> child : children) {
    CT_TNodeGroup* branchChild = new CT_TNodeGroup(_branchNodeGroup.completeName(), result);
    branchNode->addBranch(branchChild);
    setCylindersBranch(result, branchChild, child);
    constructBranchRecursively(result, branchChild, child);
  }
}

CT_CylinderData*
SF_AbstractStepQSM::constructCylinderData(std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick)
{
  Eigen::Vector3f start = brick->getStart();
  Eigen::Vector3f end = brick->getEnd();
  double radius = brick->getRadius();
  double length = brick->getLength();
  CT_CylinderData* data = new CT_CylinderData(Eigen::Vector3d(static_cast<double>((start[0] + end[0]) / 2),
                                                              static_cast<double>((start[1] + end[1]) / 2),
                                                              static_cast<double>((start[2] + end[2]) / 2)),
                                              Eigen::Vector3d(static_cast<double>(end[0] - start[0]),
                                                              static_cast<double>(end[1] - start[1]),
                                                              static_cast<double>(end[2] - start[2])),
                                              radius,
                                              length);
  return data;
}

void
SF_AbstractStepQSM::setCylindersStem(const CT_AbstractResult* result,
                                     CT_TNodeGroup* stemNode,
                                     std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks = segment->getBuildingBricks();
  for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick : bricks) {
    CT_TNodeGroup* cylinderGroup = new CT_TNodeGroup(_stemNodeGroup.completeName(), result);
    stemNode->addComponent(cylinderGroup);
    CT_CylinderData* data = constructCylinderData(brick);
    CT_Cylinder* cylinder = new CT_Cylinder(_stemNodeCylinders.completeName(), result, data);
    cylinderGroup->addItemDrawable(cylinder);
  }
}

void
SF_AbstractStepQSM::setCylindersBranch(const CT_AbstractResult* result,
                                       CT_TNodeGroup* branchNode,
                                       std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks = segment->getBuildingBricks();
  for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick : bricks) {
    CT_TNodeGroup* cylinderGroup = new CT_TNodeGroup(_branchNodeGroup.completeName(), result);
    branchNode->addComponent(cylinderGroup);
    CT_CylinderData* data = constructCylinderData(brick);
    CT_Cylinder* cylinder = new CT_Cylinder(_branchNodeCylinders.completeName(), result, data);
    cylinderGroup->addItemDrawable(cylinder);
  }
}

void
SF_AbstractStepQSM::addQSMToOutResult(CT_OutResultModelGroupToCopyPossibilities* resModelw, QString header, QString group)
{
  resModelw->addGroupModel(group, _QSMGrp, new CT_StandardItemGroup(), tr("QSM Group"));
  resModelw->addGroupModel(_QSMGrp, _treeGroup, new CT_TTreeGroup(), header);
  resModelw->addGroupModel(_treeGroup, _stemNodeGroup, new CT_TNodeGroup(), tr("QSM Stem"));
  resModelw->addItemModel(_stemNodeGroup, _stemNodeCylinders, new CT_Cylinder(), tr("QSM Stem Cylinder"));
  resModelw->addGroupModel(_treeGroup, _branchNodeGroup, new CT_TNodeGroup(), tr("QSM Branch"));
  resModelw->addItemModel(_branchNodeGroup, _branchNodeCylinders, new CT_Cylinder(), tr("QSM Branch Cylinder"));
  resModelw->addItemModel(_QSMGrp, _QSMCloud, new CT_Scene(), tr("QSM Cloud"));
}
