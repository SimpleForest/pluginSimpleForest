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
#include "sf_stepGroundFilter.h"

#include "sf_stepGroundFilterAdapter.h"
#include <QtConcurrent/QtConcurrent>

#include "ct_itemdrawable/ct_pointsattributescolor.h"

SF_StepGroundFilter::SF_StepGroundFilter(CT_StepInitializeData& dataInit) : SF_AbstractFilterBinaryStep(dataInit) {}

SF_StepGroundFilter::~SF_StepGroundFilter() {}

QString
SF_StepGroundFilter::getStepDescription() const
{
  return tr("Ground Point Filter");
}

QString
SF_StepGroundFilter::getStepDetailledDescription() const
{
  return tr("Ground Point Filter - This Filter estimates for each point the normal. "
            "The angle between the normal vector and the z axis is computed. "
            "If the angle is small, the point is detected as ground, if large the "
            "point is considered non ground.");
}

QString
SF_StepGroundFilter::getStepURL() const
{
  return tr("https://www.youtube.com/watch?v=5i6_Rtv-xEw");
}

CT_VirtualAbstractStep*
SF_StepGroundFilter::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepGroundFilter(dataInit);
}

QStringList
SF_StepGroundFilter::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepGroundFilter::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
  assert(resModel != NULL);
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel("",
                          DEF_IN_GRP_CLUSTER,
                          CT_AbstractItemGroup::staticGetType(),
                          tr("Group to be denoised"),
                          "",
                          CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(
    DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Cloud to be denoised"));
}

void
SF_StepGroundFilter::createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addDouble("First the cloud is downscaled to a voxel size of  ", " (m). ", 0.015, 0.1, 3, _voxelSize);
  configDialog->addDouble("For each of the downscaled points its normal is "
                          "computed with a range search of  ",
                          "  (m). ",
                          0.025,
                          0.2,
                          3,
                          _radiusNormal);
  configDialog->addDouble("The angle for each point between the normal and the z axis is computed "
                          "and is not allowed to deviate more than ",
                          " ",
                          0.5,
                          180,
                          1,
                          _angle);
  configDialog->addText("degrees.");
}

void
SF_StepGroundFilter::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addStringChoice("Choose how many points should be removed", "", _numberPoints, _choiceNumberPoints);
  configDialog->addText("For bended trees select a weaker filter level.");
}

void
SF_StepGroundFilter::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, m_outCloudItem, new CT_PointsAttributesColor(), tr("Normal Direction"));
    resModelw->addGroupModel(DEF_IN_GRP_CLUSTER, _outGrp, new CT_StandardItemGroup(), tr("Ground Filter"));
    resModelw->addItemModel(_outGrp, _outCloud, new CT_Scene(), tr("Cloud"));
    resModelw->addItemModel(_outGrp, _outNoise, new CT_Scene(), tr("Noise"));
  }
}

void
SF_StepGroundFilter::adaptParametersToExpertLevel()
{
  if (!_isExpert) {
    _x = 0;
    _y = 0;
    _z = 1;
    _radiusNormal = 0.04;
    _voxelSize = 0.015;
    _sizeOutput = 2;
    if (_choiceNumberPoints == _few) {
      _angle = 15;
    } else if (_choiceNumberPoints == _intermediate) {
      _angle = 30;
    } else if (_choiceNumberPoints == _many) {
      _angle = 45;
    } else {
      throw std::runtime_error("SF_StepGroundFilter parameter adaptation error.");
    }
  }
}

void
SF_StepGroundFilter::writeOutputPerScence(CT_ResultGroup* outResult, size_t i)
{
  SF_ParamGroundFilter<SF_PointNormal> param = _paramList.at(static_cast<int>(i));
  std::vector<CT_PointCloudIndexVector*> outputIndexList = createOutputVectors(param._sizeOutput);
  createOutputIndices(outputIndexList, param._outputIndices, param._itemCpyCloudIn);
  CT_StandardItemGroup* filterGrp = new CT_StandardItemGroup(_outGrp.completeName(), outResult);
  param._grpCpyGrp->addGroup(filterGrp);
  addSceneToFilterGrp(filterGrp, outResult, outputIndexList[0], _outCloud.completeName());
  addSceneToFilterGrp(filterGrp, outResult, outputIndexList[1], _outNoise.completeName());
}

void
SF_StepGroundFilter::writeOutput(CT_ResultGroup* outResult)
{
  size_t size = _paramList.size();
  for (size_t i = 0; i < size; i++) {
    writeOutputPerScence(outResult, i);
  }
}

void
SF_StepGroundFilter::compute()
{
  const QList<CT_ResultGroup*>& outResultList = getOutResultList();
  CT_ResultGroup* outResult = outResultList.at(0);
  identifyAndRemoveCorruptedScenes(outResult);
  createParamList(outResult);
  QFuture<void> future = QtConcurrent::map(_paramList, SF_StepGroundFilterAdapter());
  setProgressByFuture(future, 10, 85);
  writeOutput(outResult);
  writeLogger();
  _paramList.clear();
}

void
SF_StepGroundFilter::writeLogger()
{
  if (!_paramList.empty()) {
    QStringList strList = _paramList[0].toStringList();
    for (QString& str : strList) {
      PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
    }
    size_t filtered = 0;
    size_t total = 0;
    for (SF_ParamGroundFilter<SF_PointNormal> const& param : _paramList) {
      auto vector = param._outputIndices;
      for (size_t i = 0; i < vector.size(); i++) {
        total++;
        filtered += static_cast<size_t>(vector[i]);
      }
    }
    QString str2 = _paramList[0].toFilterString(total, filtered);
    PS_LOG->addMessage(LogInterface::info, LogInterface::step, str2);
  }
}

void
SF_StepGroundFilter::createParamList(CT_ResultGroup* outResult)
{
  adaptParametersToExpertLevel();
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(
      this, DEF_IN_CLOUD_SEED);
    SF_ParamGroundFilter<SF_PointNormal> param;
    param._log = PS_LOG;
    param._x = _x;
    param._y = _y;
    param._z = _z;
    param._angle = _angle;
    param._radiusNormal = _radiusNormal;
    param._voxelSize = _voxelSize;
    param._sizeOutput = 2;
    param._itemCpyCloudIn = ctCloud;
    param._grpCpyGrp = group;
    _paramList.append(param);
  }
}
