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

#include "sf_stepqsmrefitcylinders.h"

#include "steps/item/sf_spherefollowing_parameters_item.h"
#include "steps/qsm/postprocessing/sf_stepqsmrefitcylindersadapter.h"

#include <QtConcurrent/QtConcurrent>

#include <ct_itemdrawable/ct_cylinder.h>

SF_StepQSMRefitCylinders::SF_StepQSMRefitCylinders(CT_StepInitializeData& dataInit) : SF_AbstractStepSegmentation(dataInit) {}

SF_StepQSMRefitCylinders::~SF_StepQSMRefitCylinders() {}

QString
SF_StepQSMRefitCylinders::getStepDescription() const
{
  return tr("SphereFollowing Refit Cylinders");
}

QString
SF_StepQSMRefitCylinders::getStepDetailledDescription() const
{
  return tr("TODO ");
}

QString
SF_StepQSMRefitCylinders::getStepURL() const
{
  return tr("");
}

CT_VirtualAbstractStep*
SF_StepQSMRefitCylinders::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepQSMRefitCylinders(dataInit);
}

QStringList
SF_StepQSMRefitCylinders::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationSphereFollowing());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepQSMRefitCylinders::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  configDialog->addText("<b>Allometric correction</b>:");
  configDialog->addDouble("To even out the distribution and speed things up the cloud is "
                          "downscaled first to [<em><b>voxel size</b></em>] ",
                          " (m). ",
                          0.005,
                          0.03,
                          3,
                          _PP_voxelSize);
  configDialog->addDouble("For a fitted cylinder the angle between normal and cylinder axis has to be 90 -   "
                          " [<em><b>degree</b></em>] ",
                          " of cylinders. ",
                          1,
                          45,
                          1,
                          m_angle);
  configDialog->addDouble("Use as inlier distance  "
                          " [<em><b>inlierDistance</b></em>] ",
                          " of cylinders. ",
                          0.01,
                          2.0,
                          2,
                          m_inlierDistance);
  configDialog->addDouble("Use as percentage of radii deviation this inlier distance  "
                          " [<em><b>perc  inlierDistance</b></em>] ",
                          " . ",
                          0.01,
                          1.0,
                          2,
                          m_range);

  configDialog->addInt("Fit a cylinder only with "
                       " [<em><b>min Points</b></em>] ",
                       "",
                       3,
                       99,
                       m_minPts);
  configDialog->addInt("Use [<em><b>ransac iterations</b></em>] ", "", 10, 10000, m_ransacIterations);
  createPostConfigurationDialogCitation(configDialog);
}

void
SF_StepQSMRefitCylinders::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Result"));
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel("",
                          DEF_IN_GRP_CLUSTER,
                          CT_AbstractItemGroup::staticGetType(),
                          tr("QSM Group"),
                          "",
                          CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_QSM, SF_QSM_Item::staticGetType(), tr("QSM item"));
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Tree Cloud"));
}

void
SF_StepQSMRefitCylinders::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    resModelw->addGroupModel(DEF_IN_GRP_CLUSTER, _outCylinderGroup, new CT_StandardItemGroup(), tr("QSM Group"));
    resModelw->addItemModel(_outCylinderGroup, _outCylinders, new CT_Cylinder(), tr("QSM Cylinders"));
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, _outSFQSM, new SF_QSM_Item(), tr("QSM Item"));
  }
}

void
SF_StepQSMRefitCylinders::compute()
{
  const QList<CT_ResultGroup*>& out_result_list = getOutResultList();
  CT_ResultGroup* outResult = out_result_list.at(0);
  createParamList(outResult);
  writeLogger();
  QFuture<void> future = QtConcurrent::map(_paramList, SF_StepQSMRefitCylindersAdapter());
  while (!future.isFinished()) {
    setProgressByCounter(10.0f, 85.0f);
  }
  addQSM(outResult,
         _paramList,
         QString::fromUtf8(DEF_IN_GRP_CLUSTER),
         _outCylinders.completeName(),
         _outCylinderGroup.completeName(),
         _outSFQSM.completeName(),
         "");
  _paramList.clear();
}

void
SF_StepQSMRefitCylinders::createParamList(CT_ResultGroup* outResult)
{
  CT_ResultGroupIterator outResItCloud(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResItCloud.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResItCloud.next();
    const SF_QSM_Item* QSM_Item = (const SF_QSM_Item*)group->firstItemByINModelName(this, DEF_IN_QSM);
    const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(
      this, DEF_IN_CLOUD_SEED);
    SF_ParamRefitCylinders params;
    params._itemCpyCloudIn = ctCloud;
    auto qsm = QSM_Item->getQsm();
    params._qsm = qsm;
    params.m_minPts = m_minPts;
    params.m_voxelSize = _PP_voxelSize;
    params.m_range = m_range;
    params.m_inlierDistance = m_inlierDistance;
    params.m_ransacIterations = m_ransacIterations;
    params.m_angle = m_angle;
    _paramList.append(std::move(params));
  }
  _computationsTotal = _paramList.size();
}