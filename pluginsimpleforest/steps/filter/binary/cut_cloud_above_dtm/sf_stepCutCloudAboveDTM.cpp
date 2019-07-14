/****************************************************************************

 Copyright (C) 2017-2018 Jan Hackenberg, free software developer
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

#include <QtConcurrent/QtConcurrent>

#include "sf_stepCutCloudAboveDTM.h"
#include "sf_stepCutCloudAboveDTMAdapter.h"

SF_StepCutCloudAboveDTM::SF_StepCutCloudAboveDTM(CT_StepInitializeData& dataInit) : SF_AbstractFilterBinaryStep(dataInit) {}

SF_StepCutCloudAboveDTM::~SF_StepCutCloudAboveDTM() {}

QString
SF_StepCutCloudAboveDTM::getStepDescription() const
{
  return tr("Cut Cloud Above DTM");
}

QString
SF_StepCutCloudAboveDTM::getStepDetailledDescription() const
{
  return tr("Cut Cloud Above DTM - Takes an input Scene and a DTM model. "
            "Splits the scene into two clouds UPPER and LOWER."
            "The user selects a threshold height and each point with a height "
            "below the threshold is put into cloud "
            "LOWER, into UPPPER otherwise. ");
}

QString
SF_StepCutCloudAboveDTM::getStepURL() const
{
  return tr("https://www.youtube.com/watch?v=5i6_Rtv-xEw");
}

CT_VirtualAbstractStep*
SF_StepCutCloudAboveDTM::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepCutCloudAboveDTM(dataInit);
}

QStringList
SF_StepCutCloudAboveDTM::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepCutCloudAboveDTM::createInResultModelListProtected()
{
  CT_InResultModelGroup* resModelDTM = createNewInResultModel(DEF_IN_RESULT_DTM, tr("DTM"));
  assert(resModelDTM != NULL);
  resModelDTM->setZeroOrMoreRootGroup();
  resModelDTM->addGroupModel(
    "", DEF_IN_DTMGRP, CT_AbstractItemGroup::staticGetType(), tr("DTM Grp In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModelDTM->addItemModel(DEF_IN_DTMGRP, DEF_IN_DTM, CT_Image2D<float>::staticGetType(), tr("DTM"));
  CT_InResultModelGroupToCopy* res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
  assert(res_model != NULL);
  res_model->setZeroOrMoreRootGroup();
  res_model->addGroupModel("",
                           DEF_IN_GRP_CLUSTER,
                           CT_AbstractItemGroup::staticGetType(),
                           tr("Group to be sliced"),
                           "",
                           CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  res_model->addItemModel(
    DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Cloud to be sliced"));
}

void
SF_StepCutCloudAboveDTM::createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addDouble("Over the input cloud is iterated. For each point the height <b>h</b> "
                          "above the DTM is computed. If <b>h</b> is lower than ",
                          " (m). ",
                          -0.1,
                          50,
                          2,
                          _cutHeight);
  configDialog->addText(" the point is put into the lower cloud, into the upper otherwise.");
}

void
SF_StepCutCloudAboveDTM::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog)
{}

void
SF_StepCutCloudAboveDTM::createPreConfigurationDialog()
{
  _isExpert = true;
}

void
SF_StepCutCloudAboveDTM::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    resModelw->addGroupModel(DEF_IN_GRP_CLUSTER, _outGrp, new CT_StandardItemGroup(), tr("Vertical Slice"));
    resModelw->addItemModel(_outGrp, _outCloud, new CT_Scene(), tr("Lower"));
    resModelw->addItemModel(_outGrp, _outNoise, new CT_Scene(), tr("Upper"));
  }
}

void
SF_StepCutCloudAboveDTM::adaptParametersToExpertLevel()
{}

void
SF_StepCutCloudAboveDTM::writeOutputPerScence(CT_ResultGroup* outResult, size_t i)
{
  SF_ParamDTMHeight<pcl::PointXYZ> param = _paramList.at(i);
  std::vector<CT_PointCloudIndexVector*> outputIndexList = createOutputVectors(param._sizeOutput);
  createOutputIndices(outputIndexList, param._outputIndices, param._itemCpyCloudIn);
  CT_StandardItemGroup* filterGrp = new CT_StandardItemGroup(_outGrp.completeName(), outResult);
  param._grpCpyGrp->addGroup(filterGrp);
  addSceneToFilterGrp(filterGrp, outResult, outputIndexList[0], _outCloud.completeName());
  addSceneToFilterGrp(filterGrp, outResult, outputIndexList[1], _outNoise.completeName());
}

void
SF_StepCutCloudAboveDTM::writeOutput(CT_ResultGroup* outResult)
{
  size_t size = _paramList.size();
  for (size_t i = 0; i < size; i++) {
    writeOutputPerScence(outResult, i);
  }
}

void
SF_StepCutCloudAboveDTM::compute()
{
  const QList<CT_ResultGroup*>& outResultList = getOutResultList();
  CT_ResultGroup* outResult = outResultList.at(0);
  identifyAndRemoveCorruptedScenes(outResult);
  createParamList(outResult);
  QFuture<void> future = QtConcurrent::map(_paramList, SF_StepCutCloudAboveDTMAdapter());
  setProgressByFuture(future, 10, 85);
  writeOutput(outResult);
  writeLogger();
  _paramList.clear();
}

void
SF_StepCutCloudAboveDTM::writeLogger()
{
  if (!_paramList.empty()) {
    auto strList = _paramList[0].toStringList();
    for (auto& str : strList) {
      PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
    }
    size_t lower = 0;
    size_t upper = 0;
    for (auto const& param : _paramList) {
      auto vector = param._outputIndices;
      for (auto i : vector) {
        lower += static_cast<size_t>(1 - i);
        upper += static_cast<size_t>(i);
      }
    }
    auto str2 = _paramList[0].toFilterString(lower, upper);
    PS_LOG->addMessage(LogInterface::info, LogInterface::step, str2);
  }
}

void
SF_StepCutCloudAboveDTM::createParamList(CT_ResultGroup* outResult)
{
  CT_ResultGroup* inDTMResult = getInputResults().at(0);
  CT_ResultItemIterator iterDTM(inDTMResult, this, DEF_IN_DTM);
  CT_Image2D<float>* dtm = (CT_Image2D<float>*)iterDTM.next();
  adaptParametersToExpertLevel();
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(
      this, DEF_IN_CLOUD_SEED);
    SF_ParamDTMHeight<pcl::PointXYZ> param;
    param._log = PS_LOG;
    param._dtmCT = dtm;
    param._sliceHeight = _cutHeight;
    param._sizeOutput = 2;
    param._itemCpyCloudIn = ctCloud;
    param._grpCpyGrp = group;
    _paramList.append(param);
  }
}
