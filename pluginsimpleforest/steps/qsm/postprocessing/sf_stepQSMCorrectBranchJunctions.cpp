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

#include "sf_stepQSMCorrectBranchJunctions.h"

#include "steps/qsm/postprocessing/sf_stepQSMCorrectBranchJunctionsAdapter.h"

#include <ct_itemdrawable/ct_cylinder.h>

#include <QtConcurrent/QtConcurrent>

SF_StepQSMCorrectBranchJunctions::SF_StepQSMCorrectBranchJunctions(CT_StepInitializeData& dataInit) : SF_AbstractStepQSM(dataInit) {}

SF_StepQSMCorrectBranchJunctions::~SF_StepQSMCorrectBranchJunctions() {}

QString
SF_StepQSMCorrectBranchJunctions::getStepDescription() const
{
  return tr("QSM Correct branch junctions - not recommended");
}

QString
SF_StepQSMCorrectBranchJunctions::getStepDetailledDescription() const
{
  return tr("Often the first buildingbrick inside a segment is spatially badly alligned due to Spherefollowing routine."
            " Here we extrapolate the axis of the second cylinder towards the first cylinders direction. The first cylinder"
            " receives a new start point on this axis to be better alligned. We choose the intersection of the axis with"
            " the parent building brick as this start point.");
}

QString
SF_StepQSMCorrectBranchJunctions::getStepURL() const
{
  return tr("http://simpleforest.org/");
}

CT_VirtualAbstractStep*
SF_StepQSMCorrectBranchJunctions::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepQSMCorrectBranchJunctions(dataInit);
}

QStringList
SF_StepQSMCorrectBranchJunctions::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepQSMCorrectBranchJunctions::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  addOutputFormat(configDialog);
  configDialog->addText(QObject::tr("For invention of this step please cite  "
                                    "the following:"),
                        "Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P.");
  configDialog->addText("(section D.1. Post Processing)",
                        "<em>SimpleTree - An Efficient Open Source Tool to "
                        "Build Tree Models from TLS Clouds.</em>");
  configDialog->addText("", "Forests <b>2015</b>, 6, 4245-4294.");
  configDialog->addEmpty();
  createPostConfigurationDialogCitation(configDialog);
}

void
SF_StepQSMCorrectBranchJunctions::createInResultModelListProtected()
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
SF_StepQSMCorrectBranchJunctions::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    QString name = tr("QSM sphereFollowing corrected branch junctions ");
    QString sfCylinders = name;
    sfCylinders.append(tr("SF QSM plugin internal"));
    addQSMToOutResult(resModelw, name, QString::fromUtf8(DEF_IN_GRP_CLUSTER));
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, _outSFQSM, new SF_QSM_Item(), sfCylinders);
  }
}

void
SF_StepQSMCorrectBranchJunctions::compute()
{
  const QList<CT_ResultGroup*>& out_result_list = getOutResultList();
  CT_ResultGroup* outResult = out_result_list.at(0);
  createParamList(outResult);
  writeLogger();
  QFuture<void> future = QtConcurrent::map(_paramList, SF_StepQSMCorrectBranchJunctionsAdapter());
  while (!future.isFinished()) {
    setProgressByCounter(10.0f, 85.0f);
  }
  SF_AbstractStepQSM::addQSM<SF_ParamAllometricCorrectionNeighboring>(
    outResult, _paramList, QString::fromUtf8(DEF_IN_GRP_CLUSTER), _outSFQSM.completeName());
  _paramList.clear();
}

void
SF_StepQSMCorrectBranchJunctions::createParamList(CT_ResultGroup* outResult)
{
  CT_ResultGroupIterator outResItCloud(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResItCloud.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResItCloud.next();
    const SF_QSM_Item* QSM_Item = (const SF_QSM_Item*)group->firstItemByINModelName(this, DEF_IN_QSM);
    SF_ParamAllometricCorrectionNeighboring params;
    auto qsm = QSM_Item->getQsm();
    params._qsm = qsm;
    _paramList.append(std::move(params));
  }
  m_computationsTotal = _paramList.size();
}
