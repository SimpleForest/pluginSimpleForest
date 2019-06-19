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
    if (index == 0 || !m_splitQSM) {
      CT_TNodeGroup* stemChild = new CT_TNodeGroup(_stemNodeGroup.completeName(), result);
      stemNode->addBranch(stemChild);
      setCylindersStem(result, stemChild, child);
      constructStemRecursively(result, stemChild, child);
    } else {
      if (child->getBranchID() > -2) {
        CT_TNodeGroup* branchChild = new CT_TNodeGroup(_branchNodeGroup.completeName(), result);
        stemNode->addBranch(branchChild);
        setCylindersBranch(result, branchChild, child);
        constructBranchRecursively(result, branchChild, child);
      } else {
        CT_TNodeGroup* twigChild = new CT_TNodeGroup(_twigNodeGroup.completeName(), result);
        stemNode->addBranch(twigChild);
        setCylindersTwig(result, twigChild, child);
        constructTwigRecursively(result, twigChild, child);
      }
    }
    index++;
  }
}

void
SF_AbstractStepQSM::constructTwigRecursively(const CT_AbstractResult* result,
                                             CT_TNodeGroup* twigNode,
                                             std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> children = segment->getChildren();
  for (std::shared_ptr<SF_ModelAbstractSegment> child : children) {
    CT_TNodeGroup* twigChild = new CT_TNodeGroup(_twigNodeGroup.completeName(), result);
    twigNode->addBranch(twigChild);
    setCylindersTwig(result, twigChild, child);
    constructTwigRecursively(result, twigChild, child);
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
  Eigen::Vector3d start = brick->getStart();
  Eigen::Vector3d end = brick->getEnd();
  double radius = brick->getRadius();
  double length = brick->getLength();
  CT_CylinderData* data = new CT_CylinderData((start + end) / 2, Eigen::Vector3d(end - start), radius, length);
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
    addAttributesStem(result, cylinder, brick);
    cylinderGroup->addItemDrawable(cylinder);
  }
}

void
SF_AbstractStepQSM::addOutputFormat(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addBool("Split tree into stem, branches and twigs in output.", "", "", m_splitQSM);
}

void
SF_AbstractStepQSM::setCylindersTwig(const CT_AbstractResult* result,
                                     CT_TNodeGroup* twigNode,
                                     std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks = segment->getBuildingBricks();
  for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick : bricks) {
    CT_TNodeGroup* cylinderGroup = new CT_TNodeGroup(_twigNodeGroup.completeName(), result);
    twigNode->addComponent(cylinderGroup);
    CT_CylinderData* data = constructCylinderData(brick);
    CT_Cylinder* cylinder = new CT_Cylinder(_twigNodeCylinders.completeName(), result, data);
    addAttributesTwig(result, cylinder, brick);
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
    addAttributesBranch(result, cylinder, brick);
    cylinderGroup->addItemDrawable(cylinder);
  }
}

void
SF_AbstractStepQSM::addQSMToOutResult(CT_OutResultModelGroupToCopyPossibilities* resModelw, QString header, QString group)
{
    QString groupName = header;
    groupName.append(tr("result group"));
  resModelw->addGroupModel(group, _QSMGrp, new CT_StandardItemGroup(), groupName);
  QString ctTreeGroup = header;
  ctTreeGroup.append(tr("CT QSM structure group"));
  resModelw->addGroupModel(_QSMGrp, _treeGroup, new CT_TTreeGroup(), ctTreeGroup);
  resModelw->addGroupModel(_treeGroup, _stemNodeGroup, new CT_TNodeGroup(), tr("QSM Stem"));
  resModelw->addItemModel(_stemNodeGroup, _stemNodeCylinders, new CT_Cylinder(), tr("QSM Stem Cylinder"));
  resModelw->addItemAttributeModel(
    _stemNodeCylinders,
    m_stemGrowthVolume,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("log growthVolume"));
  resModelw->addItemAttributeModel(
    _stemNodeCylinders,
    m_stemGrowthLength,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("log growthLength"));
  resModelw->addItemAttributeModel(
    _stemNodeCylinders,
    m_stemBranchID,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("Branch ID"));
  resModelw->addItemAttributeModel(
    _stemNodeCylinders,
    m_stemReversePipeOrder,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("reversePipeOrder"));

  resModelw->addGroupModel(_treeGroup, _branchNodeGroup, new CT_TNodeGroup(), tr("QSM Branch"));
  resModelw->addItemModel(_branchNodeGroup, _branchNodeCylinders, new CT_Cylinder(), tr("QSM Branch Cylinder"));
  resModelw->addItemAttributeModel(
    _branchNodeCylinders,
    m_branchGrowthVolume,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("log growthVolume"));
  resModelw->addItemAttributeModel(
    _branchNodeCylinders,
    m_branchGrowthLength,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("log growthLength"));
  resModelw->addItemAttributeModel(
    _branchNodeCylinders,
    m_branchBranchID,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("Branch ID"));
  resModelw->addItemAttributeModel(
    _branchNodeCylinders,
    m_branchReversePipeOrder,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("reversePipeOrder"));

  resModelw->addGroupModel(_treeGroup, _twigNodeGroup, new CT_TNodeGroup(), tr("QSM Twig"));
  resModelw->addItemModel(_twigNodeGroup, _twigNodeCylinders, new CT_Cylinder(), tr("QSM Twig Cylinder"));
  resModelw->addItemAttributeModel(
    _twigNodeCylinders,
    m_twigGrowthVolume,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("log growthVolume"));
  resModelw->addItemAttributeModel(
    _twigNodeCylinders,
    m_twigGrowthLength,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("log growthLength"));
  resModelw->addItemAttributeModel(
    _twigNodeCylinders,
    m_twigBranchID,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("Branch ID"));
  resModelw->addItemAttributeModel(
    _twigNodeCylinders,
    m_twigPipeOrder,
    new CT_StdItemAttributeT<float>(NULL, PS_CATEGORY_MANAGER->findByUniqueName(CT_AbstractCategory::DATA_NUMBER), NULL, 0),
    tr("reversePipeOrder"));
}

void
SF_AbstractStepQSM::addAttributesBranch(const CT_AbstractResult* result,
                                        CT_Cylinder* cylinder,
                                        std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick)
{
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(
    m_branchGrowthVolume.completeName(), CT_AbstractCategory::DATA_NUMBER, result, std::log(brick->getGrowthVolume())));
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(
    m_branchGrowthLength.completeName(), CT_AbstractCategory::DATA_NUMBER, result, std::log(brick->getGrowthLength())));
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(
    m_branchBranchID.completeName(), CT_AbstractCategory::DATA_NUMBER, result, brick->getSegment()->getBranchID()));
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(m_branchReversePipeOrder.completeName(),
                                                             CT_AbstractCategory::DATA_NUMBER,
                                                             result,
                                                             brick->getSegment()->getReversePipeBranchOrder()));
}

void
SF_AbstractStepQSM::addAttributesTwig(const CT_AbstractResult* result,
                                      CT_Cylinder* cylinder,
                                      std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick)
{
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(
    m_twigGrowthVolume.completeName(), CT_AbstractCategory::DATA_NUMBER, result, std::log(brick->getGrowthVolume())));
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(
    m_twigGrowthLength.completeName(), CT_AbstractCategory::DATA_NUMBER, result, std::log(brick->getGrowthLength())));
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(
    m_twigBranchID.completeName(), CT_AbstractCategory::DATA_NUMBER, result, brick->getSegment()->getBranchID()));
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(
    m_twigPipeOrder.completeName(), CT_AbstractCategory::DATA_NUMBER, result, brick->getSegment()->getReversePipeBranchOrder()));
}

void
SF_AbstractStepQSM::addAttributesStem(const CT_AbstractResult* result,
                                      CT_Cylinder* cylinder,
                                      std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick)
{
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(
    m_stemGrowthVolume.completeName(), CT_AbstractCategory::DATA_NUMBER, result, std::log(brick->getGrowthVolume())));
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(
    m_stemGrowthLength.completeName(), CT_AbstractCategory::DATA_NUMBER, result, std::log(brick->getGrowthLength())));
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(
    m_stemBranchID.completeName(), CT_AbstractCategory::DATA_NUMBER, result, brick->getSegment()->getBranchID()));
  cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(m_stemReversePipeOrder.completeName(),
                                                             CT_AbstractCategory::DATA_NUMBER,
                                                             result,
                                                             brick->getSegment()->getReversePipeBranchOrder()));
}
