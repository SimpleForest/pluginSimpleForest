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

#include "sf_stepQSMAllometricCorrection.h"

#include "steps/item/sf_spherefollowing_parameters_item.h"
#include "steps/qsm/modelling/sf_stepQSMAllometricCorrectionAdapter.h"

#include <QtConcurrent/QtConcurrent>

#include <ct_itemdrawable/ct_cylinder.h>

SF_StepQSMAllometricCorrection::SF_StepQSMAllometricCorrection(CT_StepInitializeData& dataInit) : SF_AbstractStepQSM(dataInit) {}

SF_StepQSMAllometricCorrection::~SF_StepQSMAllometricCorrection() {}

QString
SF_StepQSMAllometricCorrection::getStepDescription() const
{
  return tr("QSM Allometric Check");
}

QString
SF_StepQSMAllometricCorrection::getStepDetailledDescription() const
{
  return tr(
    "This step utilizes a relation between the growthlength or growthvolume between and the radius of a cylinder in form"
    " of the two parameter equation (radius = a*growthlength^b) or the three parameter equation (radius = a*growthlength^b +c)."
    " See PhD Hackenberg et al 2016 and following for the invention of those two parameters."
    " According to Xu et al(2006) the original function utilizing length instead of growth length is in agreement with."
    " the metabolic scaling theory.");
}

QString
SF_StepQSMAllometricCorrection::getStepURL() const
{
  return tr("");
}

CT_VirtualAbstractStep*
SF_StepQSMAllometricCorrection::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepQSMAllometricCorrection(dataInit);
}

QStringList
SF_StepQSMAllometricCorrection::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationSphereFollowing());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepQSMAllometricCorrection::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  configDialog->addText("<b>Allometric correction</b>:");
  configDialog->addDouble("The lower the correction factor, a number between 0 and 1, the more strict is the check"
                          " with[<em><b>threshold</b></em>] ",
                          " (m). ",
                          0.001,
                          0.999,
                          3,
                          _range);
  configDialog->addDouble("Each cylinder has a "
                          " [<em><b>min Radius</b></em>] ",
                          " (m). ",
                          0.0001,
                          0.01,
                          5,
                          _minRadius);
  configDialog->addBool(
    "Uses growthLength instead of growthVolume ", "", "growthLength if checked, growthvolume otherwise", m_useGrowthLength);

  configDialog->addBool("Estimate 3 parameter equations ", "", "3 parameters with intercept", m_withIntercept);
  configDialog->addDouble("Only use most inner cylinder representing the major branching structure "
                          " [<em><b>percentage of cylinders</b></em>] ",
                          " .",
                          0.1,
                          1.0,
                          2,
                          m_quantile);
  configDialog->addDouble("Use as inlier distance  "
                          " [<em><b>inlierDistance</b></em>] ",
                          " (m). ",
                          0.01,
                          2.0,
                          2,
                          m_inlierDistance);

  configDialog->addDouble("Use as power parameter  "
                          " [<em><b>power parameter</b></em>] ",
                          " . ",
                          0.333,
                          0.5,
                          3,
                          m_power);

  configDialog->addInt("Fit only with "
                       " [<em><b>min Points</b></em>] ",
                       "",
                       3,
                       99,
                       m_minPts);

  configDialog->addInt("Use "
                       " [<em><b>ransac iterations</b></em>] ",
                       "",
                       100,
                       10000,
                       m_ransacIterations);

  configDialog->addInt("Use "
                       " [<em><b>gauss newton iterations</b></em>] ",
                       "",
                       10,
                       50,
                       m_gaussNewtonIterations);
  configDialog->addFileChoice(tr("Choose export file"), CT_FileChoiceButton::OneNewFile, "csv (*.csv)", m_filePath);

  configDialog->addBool("Estimate [<em><b>power parameter</b></em>] ", "", "automatic search", m_estimateParams);
  createPostConfigurationDialogCitation(configDialog);
}

void
SF_StepQSMAllometricCorrection::createInResultModelListProtected()
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
}

void
SF_StepQSMAllometricCorrection::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    addQSMToOutResult(resModelw, QString("QSM Allometric correction"), QString::fromUtf8(DEF_IN_GRP_CLUSTER));
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, _outSFQSM, new SF_QSM_Item(), tr("QSM cylinders allometric corrected"));
  }
}

void
SF_StepQSMAllometricCorrection::compute()
{
  const QList<CT_ResultGroup*>& out_result_list = getOutResultList();
  CT_ResultGroup* outResult = out_result_list.at(0);
  createParamList(outResult);
  writeLogger();
  QFuture<void> future = QtConcurrent::map(_paramList, SF_QSMAllometricCheckAdapter());
  while (!future.isFinished()) {
    setProgressByCounter(10.0f, 85.0f);
  }
  SF_AbstractStepQSM::addQSM<SF_ParamAllometricCorrectionNeighboring>(
    outResult, _paramList, QString::fromUtf8(DEF_IN_GRP_CLUSTER), _outSFQSM.completeName());
  if (m_filePath.size() > 0) {
    QFile file(m_filePath.first());
    if (file.open(QFile::WriteOnly)) {
      for (int i = 0; i < _paramList.size(); i++) {
        QTextStream stream(&file);
        SF_ParamAllometricCorrectionNeighboring param = _paramList[i];
        auto qsm = param._qsm;
        qsm->setID(i);
        auto cylinders = qsm->getBuildingBricks();
        if (cylinders.size() == 0) {
          return;
        }
        stream << QString::fromStdString(cylinders.at(0)->toHeaderString()) << "\n";
        for (int j = 0; j < cylinders.size(); j++) {
          auto cylinder = cylinders[j];
          stream << QString::fromStdString(cylinder->toString());
        }
      }
    }
    file.close();
  }
  _paramList.clear();
}

void
SF_StepQSMAllometricCorrection::createParamList(CT_ResultGroup* outResult)
{
  CT_ResultGroupIterator outResItCloud(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResItCloud.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResItCloud.next();
    const SF_QSM_Item* QSM_Item = (const SF_QSM_Item*)group->firstItemByINModelName(this, DEF_IN_QSM);
    SF_ParamAllometricCorrectionNeighboring params;
    auto qsm = QSM_Item->getQsm();
    params._qsm = qsm;
    params._range = _range;
    params._minRadius = _minRadius;
    params.m_power = m_power;
    params.m_useGrowthLength = m_useGrowthLength;
    params.m_withIntercept = m_withIntercept;
    params.m_quantile = m_quantile;
    params.m_minPts = m_minPts;
    params.m_inlierDistance = m_inlierDistance;
    params.m_ransacIterations = m_ransacIterations;
    params.m_gaussNewtonIterations = m_gaussNewtonIterations;
    params.m_estimateParams = m_estimateParams;
    _paramList.append(std::move(params));
  }
  m_computationsTotal = _paramList.size();
}
