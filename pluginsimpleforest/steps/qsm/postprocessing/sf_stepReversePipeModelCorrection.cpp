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

#include "sf_stepReversePipeModelCorrection.h"

#include "steps/item/sf_spherefollowing_parameters_item.h"
#include "steps/qsm/postprocessing/sf_stepreversepipemodelcorrectionadapter.h"

#include <QtConcurrent/QtConcurrent>

#include <ct_itemdrawable/ct_cylinder.h>

SF_StepReversePipeModelCorrection::SF_StepReversePipeModelCorrection(CT_StepInitializeData& dataInit) : SF_AbstractStepQSM(dataInit) {}

SF_StepReversePipeModelCorrection::~SF_StepReversePipeModelCorrection() {}

QString
SF_StepReversePipeModelCorrection::getStepDescription() const
{
  return tr("QSM Reverse Pipe Model Filter");
}

QString
SF_StepReversePipeModelCorrection::getStepDetailledDescription() const
{
  return tr("The reverse pipe model order is defined as 1 for leave buildingbricks and should present the radius of the tip at a "
            "metabolic scaling unit of the tree."
            "According to the pipe model theory the parent diameter is close to equal to the child cylinder. The combind vessels "
            "inside the branch before the branch"
            " split up into subclusters of vessels in the side branches. The total vessel sum though remains the same."
            "So if we want to compute other cylinders reverse pipe model order we have to build the root meaned squared sum of the "
            "child cylinders reverse pipemodel order."
            " By doing so we have a good linear predictor for the radius. We simply do a robust computation and apply the result line "
            "fit prediced radius to all outliers.");
}

QString
SF_StepReversePipeModelCorrection::getStepURL() const
{
  return tr("http://simpleforest.org/");
}

CT_VirtualAbstractStep*
SF_StepReversePipeModelCorrection::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepReversePipeModelCorrection(dataInit);
}

QStringList
SF_StepReversePipeModelCorrection::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationSphereFollowing());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepReversePipeModelCorrection::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  addOutputFormat(configDialog);
  configDialog->addText("<b>Reverse pipe model filter</b>:");
  configDialog->addDouble("Use as inlier distance for the radius "
                          " [<em><b>inlierDistance</b></em>] ",
                          " (m). ",
                          0.01,
                          0.1,
                          3,
                          m_inlierDistance);

  configDialog->addInt("Fit only with "
                       " [<em><b>min cylinders</b></em>] ",
                       "",
                       3,
                       99,
                       m_minPts);

  configDialog->addDouble("Each cylinder has a "
                          " [<em><b>min Radius</b></em>] ",
                          " (m). ",
                          0.0001,
                          0.01,
                          5,
                          _minRadius);

  createPostConfigurationDialogCitation(configDialog);
}

void
SF_StepReversePipeModelCorrection::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel = createNewInResultModelForCopy(DEF_IN_RESULT,
                                                                        tr("Input result Step Reverse Pipe Model Filter"));
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
SF_StepReversePipeModelCorrection::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    QString name = tr("QSM reverse pipemodel ");
    QString sfCylinders = name;
    sfCylinders.append(tr("SF QSM plugin internal"));
    addQSMToOutResult(resModelw, name, QString::fromUtf8(DEF_IN_GRP_CLUSTER));
    resModelw->addItemModel(_QSMGrp, _outSFQSM, new SF_QSM_Item(), sfCylinders);
  }
}

void
SF_StepReversePipeModelCorrection::compute()
{
  const QList<CT_ResultGroup*>& out_result_list = getOutResultList();
  CT_ResultGroup* outResult = out_result_list.at(0);
  createParamList(outResult);
  writeLogger();
  QFuture<void> future = QtConcurrent::map(_paramList, SF_QSMReversePipeModelCorrectionAdapter());
  while (!future.isFinished()) {
    setProgressByCounter(10.0f, 85.0f);
  }
  SF_AbstractStepQSM::addQSM<SF_ParamReversePipeModelCorrection>(
    outResult, _paramList, QString::fromUtf8(DEF_IN_GRP_CLUSTER), _outSFQSM.completeName());
  _paramList.clear();
}

void
SF_StepReversePipeModelCorrection::createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText(
    QObject::tr(
      "For the importance of pipemodel theory already an early example (predict a pipemodel parameter, but not use it) was given in:"),
    "Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P.");
  configDialog->addText(" Figure 20. Improvement possibilities of the pipe model theory [43] with a Prunus avium model",
                        "<em>SimpleTree - An Efficient Open Source Tool to "
                        "Build Tree Models from TLS Clouds.</em>");
  configDialog->addText("", "Forests <b>2015</b>, 6, 4245-4294.");
  configDialog->addText(
    " apart from this the here used parameter was already introduced as output parameter in SimpleTree 4.x versions", "");
}

void
SF_StepReversePipeModelCorrection::createParamList(CT_ResultGroup* outResult)
{
  CT_ResultGroupIterator outResItCloud(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResItCloud.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResItCloud.next();
    const SF_QSM_Item* QSM_Item = (const SF_QSM_Item*)group->firstItemByINModelName(this, DEF_IN_QSM);
    SF_ParamReversePipeModelCorrection params;
    auto qsm = QSM_Item->getQsm();
    params._qsm = qsm;
    params.m_minPts = m_minPts;
    params.m_inlierDistance = m_inlierDistance;
    params.m_ransacIterations = m_ransacIterations;
    _paramList.append(std::move(params));
  }
  m_computationsTotal = _paramList.size();
}
