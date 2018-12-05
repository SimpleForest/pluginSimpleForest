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

#include "sf_stepStemRANSACFilter.h"

#include "sf_stepStemFilterRANSACAdapter.h"
#include <QtConcurrent/QtConcurrent>

SF_StepStemRANSACFilter::SF_StepStemRANSACFilter(
    CT_StepInitializeData &dataInit)
    : SF_AbstractFilterBinaryStep(dataInit) {}

SF_StepStemRANSACFilter::~SF_StepStemRANSACFilter() {}

QString SF_StepStemRANSACFilter::getStepDescription() const {
  return tr("Stem Filter by RANSAC fitting");
}

QString SF_StepStemRANSACFilter::getStepDetailledDescription() const {
  return tr("Stem Filter by RANSAC - Fits robust RANSAC cylinders into one "
            "meter height horizontal slices. If the cylinder per slice passes "
            "z axis check, its inliers"
            "are put into the output cloud. All other points are noise.");
}

QString SF_StepStemRANSACFilter::getStepURL() const {
  return tr("https://www.youtube.com/watch?v=5i6_Rtv-xEw");
}

CT_VirtualAbstractStep *
SF_StepStemRANSACFilter::createNewInstance(CT_StepInitializeData &dataInit) {
  return new SF_StepStemRANSACFilter(dataInit);
}

QStringList SF_StepStemRANSACFilter::getStepRISCitations() const {
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void SF_StepStemRANSACFilter::createInResultModelListProtected() {
  CT_InResultModelGroupToCopy *resModel =
      createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
  assert(resModel != NULL);
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel("", DEF_IN_GRP_CLUSTER,
                          CT_AbstractItemGroup::staticGetType(),
                          tr("Group to be denoised"), "",
                          CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED,
                         CT_Scene::staticGetType(),
                         tr("Clouod to be denoised"));
}

void SF_StepStemRANSACFilter::createPostConfigurationDialogExpert(
    CT_StepConfigurableDialog *configDialog) {
  configDialog->addDouble("First the cloud is downscaled to a voxel size of  ",
                          " (m). ", 0.005, 0.1, 3, _voxelSize);
  configDialog->addDouble("For each of the downscaled points its normal is "
                          "computed with a range search of  ",
                          "  (m). ", 0.025, 0.2, 3, _radiusNormal);
  configDialog->addDouble(
      "Into 1 meter horizontal slices a RANSAC cylinder is fitted with   ",
      "  (m). ", 0.01, 0.5, 2, _inlierDistance);
  configDialog->addText(" inlier Distance. All inliers per slice are set to "
                        "stem if the cylinder passes the following test:");
  configDialog->addDouble(
      "The angle for each point between this cylinder axis and the z axis is "
      "computed and is not allowed to deviate more than ",
      " ", 0.5, 180, 1, _angle);
  configDialog->addText("degrees.");
}

void SF_StepStemRANSACFilter::createPostConfigurationDialogBeginner(
    CT_StepConfigurableDialog *configDialog) {
  configDialog->addStringChoice("Choose how many points should be removed", "",
                                _numberPoints, _choiceNumberPoints);
  configDialog->addText("For bended trees select a weaker filter level. Also "
                        "select weaker level for worse clouds.");
}

void SF_StepStemRANSACFilter::createOutResultModelListProtected() {
  CT_OutResultModelGroupToCopyPossibilities *resModelw =
      createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    resModelw->addGroupModel(DEF_IN_GRP_CLUSTER, _outGrp,
                             new CT_StandardItemGroup(),
                             tr("Stem RANSAC Filter"));
    resModelw->addItemModel(_outGrp, _outCloud, new CT_Scene(), tr("Cloud"));
    resModelw->addItemModel(_outGrp, _outNoise, new CT_Scene(), tr("Noise"));
  }
}

void SF_StepStemRANSACFilter::adaptParametersToExpertLevel() {
  if (!_isExpert) {
    if (_choiceNumberPoints == _few) {
      _x = 0;
      _y = 0;
      _z = 1;
      _angle = 15;
      _radiusNormal = 0.04;
      _inlierDistance = 0.05;
      _voxelSize = 0.015;
      _sizeOutput = 2;
    } else if (_choiceNumberPoints == _intermediate) {
      _x = 0;
      _y = 0;
      _z = 1;
      _angle = 30;
      _radiusNormal = 0.04;
      _inlierDistance = 0.1;
      _voxelSize = 0.015;
      _sizeOutput = 2;
    } else if (_choiceNumberPoints == _many) {
      _x = 0;
      _y = 0;
      _z = 1;
      _angle = 45;
      _radiusNormal = 0.04;
      _voxelSize = 0.015;
      _inlierDistance = 0.2;
      _sizeOutput = 2;
    }
  }
}

void SF_StepStemRANSACFilter::writeOutputPerScence(CT_ResultGroup *outResult,
                                                   size_t i) {
  SF_ParamStemRansacFilter param = _paramList.at(i);
  std::vector<CT_PointCloudIndexVector *> outputIndexList =
      createOutputVectors(param._sizeOutput);
  createOutputIndices(outputIndexList, param._outputIndices,
                      param._itemCpyCloudIn);
  CT_StandardItemGroup *filterGrp =
      new CT_StandardItemGroup(_outGrp.completeName(), outResult);
  param._grpCpyGrp->addGroup(filterGrp);
  addSceneToFilterGrp(filterGrp, outResult, outputIndexList[0],
                      _outCloud.completeName());
  addSceneToFilterGrp(filterGrp, outResult, outputIndexList[1],
                      _outNoise.completeName());
}

void SF_StepStemRANSACFilter::writeOutput(CT_ResultGroup *outResult) {
  size_t size = _paramList.size();
  for (size_t i = 0; i < size; i++) {
    writeOutputPerScence(outResult, i);
  }
}

void SF_StepStemRANSACFilter::compute() {
  const QList<CT_ResultGroup *> &outResultList = getOutResultList();
  CT_ResultGroup *outResult = outResultList.at(0);
  identifyAndRemoveCorruptedScenes(outResult);
  createParamList(outResult);
  QFuture<void> future =
      QtConcurrent::map(_paramList, SF_StepStemFilterRANSACAdapter());
  std::cout << "bar" << std::endl;
  setProgressByFuture(future, 10, 85);
  writeOutput(outResult);
  writeLogger();
  _paramList.clear();
}

void SF_StepStemRANSACFilter::writeLogger() {
  if (!_paramList.empty()) {
    auto strList = _paramList[0].toStringList();
    for (auto &str : strList) {
      PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
    }
    size_t filtered = 0;
    size_t total = 0;
    for (auto const &param : _paramList) {
      auto vector = param._outputIndices;
      for (auto i : vector) {
        total++;
        filtered += static_cast<size_t>(i);
      }
    }
    auto str2 = _paramList[0].toFilterString(total, filtered);
    PS_LOG->addMessage(LogInterface::info, LogInterface::step, str2);
  }
}

void SF_StepStemRANSACFilter::createParamList(CT_ResultGroup *outResult) {
  adaptParametersToExpertLevel();
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup *group = (CT_StandardItemGroup *)outResIt.next();
    const CT_AbstractItemDrawableWithPointCloud *ctCloud =
        (const CT_AbstractItemDrawableWithPointCloud *)
            group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
    SF_ParamStemRansacFilter param;
    param._log = PS_LOG;
    param._x = _x;
    param._y = _y;
    param._z = _z;
    param._inlierDistance = _inlierDistance;
    param._angle = _angle;
    param._radiusNormal = _radiusNormal;
    param._voxelSize = _voxelSize;
    param._sizeOutput = 2;
    param._itemCpyCloudIn = ctCloud;
    param._grpCpyGrp = group;
    _paramList.append(param);
  }
}
