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

#include "sf_stepQSMMedianFilter.h"

#include "steps/item/sf_spherefollowing_parameters_item.h"

#include "qsm/algorithm/postprocessing/sf_qsmmedianfilter.h"

#include <QtConcurrent/QtConcurrent>

#include <ct_itemdrawable/ct_cylinder.h>

SF_StepQSMMedianFilter::SF_StepQSMMedianFilter(CT_StepInitializeData& dataInit) : SF_AbstractStepQSM(dataInit) {}

SF_StepQSMMedianFilter::~SF_StepQSMMedianFilter() {}

QString
SF_StepQSMMedianFilter::getStepDescription() const
{
  return tr("QSM Median Filter");
}

QString
SF_StepQSMMedianFilter::getStepDetailledDescription() const
{
  return tr("If over- or underestimated cylinders ared detected, their radii are adjusted. If a"
            " cylinder’s radius is larger or smaller than a percentage based threshold parameter based on the segments median radius"
            " it’s radius is set to the median radius.");
}

QString
SF_StepQSMMedianFilter::getStepURL() const
{
  return tr("http://simpleforest.org/");
}

CT_VirtualAbstractStep*
SF_StepQSMMedianFilter::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepQSMMedianFilter(dataInit);
}

QStringList
SF_StepQSMMedianFilter::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationSphereFollowing());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepQSMMedianFilter::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  addOutputFormat(configDialog);
  configDialog->addText("<b>Median filter</b>:");
  configDialog->addDouble("The [<em><b>percentage</b></em>] of the radius a cylinder is allowed to be larger or smaller than the "
                          " median radius within a segment: ",
                          " (m). ",
                          0.01,
                          0.99,
                          2,
                          m_percentage);
  configDialog->addEmpty();
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
SF_StepQSMMedianFilter::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Input result step QSM median filter"));
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
SF_StepQSMMedianFilter::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    QString name = tr("QSM sphereFollowing median filtered ");
    QString sfCylinders = name;
    sfCylinders.append(tr("SF QSM plugin internal"));
    addQSMToOutResult(resModelw, name, QString::fromUtf8(DEF_IN_GRP_CLUSTER));
    resModelw->addItemModel(_QSMGrp, _outSFQSM, new SF_QSM_Item(), sfCylinders);
  }
}

void
SF_StepQSMMedianFilter::compute()
{
  const QList<CT_ResultGroup*>& outResultList = getOutResultList();
  CT_ResultGroup* outResult = outResultList.at(0);
  CT_ResultGroupIterator outResItCloud(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResItCloud.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResItCloud.next();
    const SF_QSM_Item* QSM_Item = (const SF_QSM_Item*)group->firstItemByINModelName(this, DEF_IN_QSM);
    auto qsm = QSM_Item->getQsm();
    SF_QSMMedianFilter medianFilter;
    medianFilter.setPercentage(m_percentage);
    medianFilter.compute(qsm);
    SF_ParamQSMedian<SF_PointNormal> param;
    param.m_percentage = m_percentage;
    param._qsm = qsm;
    _paramList.push_back(param);
  }
  writeLogger();
  SF_AbstractStepQSM::addQSM<SF_ParamQSMedian<SF_PointNormal>>(
    outResult, _paramList, QString::fromUtf8(DEF_IN_GRP_CLUSTER), _outSFQSM.completeName());
  _paramList.clear();
}
