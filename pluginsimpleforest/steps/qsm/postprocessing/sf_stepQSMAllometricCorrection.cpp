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
#include "steps/qsm/postprocessing/sf_stepQSMAllometricCorrectionAdapter.h"

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
    "In Xu et al 2007 an early QSM radius predicting method using based on metabolic scaling theory is presented."
    "There the radius is predicted from the length to the tip. During the devlopment of Simpletree much more powerful parameters"
    " than traditional volume or length were invented. See Hackenberg 2015 for the recursive definition of the volume named "
    "growthvolume."
    "The two parameter equation (radius = a*growthlength^b) or the three parameter equation (radius = a*growthlength^b +c)."
    " showed more accuracy with the invented recusive defined parameters applied as independent variable in the power model."
    "Here we will apply the forumla of 2007 with the new parameter growthlength for QSM radius correction. We as well esimtate the"
    " utilized power parameter from the stable part of the QSM itself as propsed in the phd of Hackenberg.");
}

QString
SF_StepQSMAllometricCorrection::getStepURL() const
{
  return tr("http://simpleforest.org/");
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
  addOutputFormat(configDialog);
  configDialog->addText("<b>Allometric correction</b>:");

  configDialog->addBool("Estimate [<em><b>power parameter</b></em>] ", "", "automatic search", m_estimateParams);

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

  configDialog->addDouble("Use as inlier distance on log scaled radii "
                          " [<em><b>inlierDistance</b></em>] ",
                          " (m). ",
                          0.01,
                          0.03,
                          3,
                          m_inlierDistance);

  configDialog->addDouble("Use as power parameter if auto seach fails"
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

  createPostConfigurationDialogCitation(configDialog);
}

void
SF_StepQSMAllometricCorrection::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Input result Step Allometric Correction"));
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel("",
                          DEF_IN_GRP_CLUSTER,
                          CT_AbstractItemGroup::staticGetType(),
                          tr("QSM Group"),
                          "",
                          CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_QSM, SF_QSM_Item::staticGetType(), tr("internal QSM"));
}

void
SF_StepQSMAllometricCorrection::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    QString name = tr("QSM sphereFollowing allometric corrected ");
    QString sfCylinders = name;
    sfCylinders.append(tr("SF QSM plugin internal"));
    addQSMToOutResult(resModelw, name, QString::fromUtf8(DEF_IN_GRP_CLUSTER));
    resModelw->addItemModel(_QSMGrp, _outSFQSM, new SF_QSM_Item(), sfCylinders);
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
  _paramList.clear();
}

void
SF_StepQSMAllometricCorrection::createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText(
    QObject::tr("For this step please cite in addition an early method using allometric correction with length and radius:"),
    "Xu, H.; Gossett, N.; Chen B.");
  configDialog->addText("(section 5.1 Tree Allometry)", "<em>Knowledge and heuristic-based modeling of laser-scanned trees.</em>");
  configDialog->addText("", "ACM Transactions on Graphics <b>2007</b>, 19-es.");
  configDialog->addEmpty();
  configDialog->addText(
    QObject::tr(
      "For For the introduction of using defined parameters in such correction and autoestimating the power parameter cite also"),
    "Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P.");
  configDialog->addText(" (2.2. Tree Modeling - Allometric improvement)",
                        "<em>SimpleTree - An Efficient Open Source Tool to "
                        "Build Tree Models from TLS Clouds.</em>");
  configDialog->addText("", "Forests <b>2015</b>, 6, 4245-4294.");
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
    params.m_useGrowthLength = true;
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
