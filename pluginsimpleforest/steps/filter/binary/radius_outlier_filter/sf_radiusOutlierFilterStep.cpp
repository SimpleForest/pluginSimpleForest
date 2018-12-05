/****************************************************************************

 Copyright (C) 2017-2017 Jan Hackenberg, free software developer
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
#include "sf_radiusOutlierFilterStep.h"
#include "steps/filter/binary/radius_outlier_filter/sf_radiusOutlierFilterAdapter.h"
#include <QtConcurrent/QtConcurrent>

#include "ct_itemdrawable/ct_pointsattributescolor.h"

SF_RadiusOutlierFilterStep::SF_RadiusOutlierFilterStep(
    CT_StepInitializeData &dataInit)
    : SF_AbstractFilterBinaryStep(dataInit) {
  _numberPoints.append(_clearSky);
}

SF_RadiusOutlierFilterStep::~SF_RadiusOutlierFilterStep() {}

QString SF_RadiusOutlierFilterStep::getStepDescription() const {
  return tr("Radius Outlier Filter");
}

QString SF_RadiusOutlierFilterStep::getStepDetailledDescription() const {
  return tr("Radius Outlier Filter - Frontend to PCL filter. Points with not "
            "enough neighbors in a supported range are eliminated.");
}

QString SF_RadiusOutlierFilterStep::getStepURL() const {
  return tr(
      "http://pointclouds.org/documentation/tutorials/remove_outliers.php");
}

CT_VirtualAbstractStep *
SF_RadiusOutlierFilterStep::createNewInstance(CT_StepInitializeData &dataInit) {
  return new SF_RadiusOutlierFilterStep(dataInit);
}

QStringList SF_RadiusOutlierFilterStep::getStepRISCitations() const {
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void SF_RadiusOutlierFilterStep::createInResultModelListProtected() {
  CT_InResultModelGroupToCopy *res_model =
      createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
  assert(res_model != NULL);
  res_model->setZeroOrMoreRootGroup();
  res_model->addGroupModel("", DEF_IN_GRP_CLUSTER,
                           CT_AbstractItemGroup::staticGetType(),
                           tr("Group to be denoised"), "",
                           CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  res_model->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED,
                          CT_Scene::staticGetType(),
                          tr("Cloud to be denoised"));
}

void SF_RadiusOutlierFilterStep::createPostConfigurationDialogExpert(
    CT_StepConfigurableDialog *configDialog) {
  configDialog->addDouble(
      "Looks for each point at its numbers of neighbors in range ", "", 0.01,
      4.1, 3, _radius);
  configDialog->addInt("A point is eliminated if it contains less than.",
                       " Points", 2, 99999, _minPts);
}

void SF_RadiusOutlierFilterStep::createPostConfigurationDialogBeginner(
    CT_StepConfigurableDialog *configDialog) {
  configDialog->addStringChoice("Choose how many points should be removed", "",
                                _numberPoints, _choiceNumberPoints);
  configDialog->addText("Low resulted clouds are affected more.");
}

void SF_RadiusOutlierFilterStep::createOutResultModelListProtected() {
  CT_OutResultModelGroupToCopyPossibilities *resModelw =
      createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, m_outCloudItem,
                            new CT_PointsAttributesColor(),
                            tr("Point Density"));
    resModelw->addGroupModel(DEF_IN_GRP_CLUSTER, _outGrp,
                             new CT_StandardItemGroup(),
                             tr("Radius Outlier Filter"));
    resModelw->addItemModel(_outGrp, _outCloud, new CT_Scene(), tr("Cloud"));
    resModelw->addItemModel(_outGrp, _outNoise, new CT_Scene(), tr("Noise"));
  }
}

void SF_RadiusOutlierFilterStep::writeOutputPerScence(CT_ResultGroup *outResult,
                                                      size_t i) {
  SF_ParamRadiusOutlierFilter<SF_PointNormal> param =
      _paramList.at(static_cast<int>(i));
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

void SF_RadiusOutlierFilterStep::writeOutput(CT_ResultGroup *outResult) {
  size_t size = _paramList.size();
  for (size_t i = 0; i < size; i++) {
    writeOutputPerScence(outResult, i);
  }
}

void SF_RadiusOutlierFilterStep::compute() {
  const QList<CT_ResultGroup *> &outResultList = getOutResultList();
  CT_ResultGroup *outResult = outResultList.at(0);
  identifyAndRemoveCorruptedScenes(outResult);
  createParamList(outResult);
  QFuture<void> future =
      QtConcurrent::map(_paramList, SF_RadiusOutlierFilterAdapter());
  setProgressByFuture(future, 10, 85);
  writeOutput(outResult);
  writeLogger();
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  size_t index = 0;
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup *group = (CT_StandardItemGroup *)outResIt.next();
    const CT_AbstractItemDrawableWithPointCloud *ct_cloud =
        (const CT_AbstractItemDrawableWithPointCloud *)
            group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
    SF_ParamRadiusOutlierFilter<SF_PointNormal> param = _paramList[index++];
    CT_PointsAttributesColor *colorAttribute = new CT_PointsAttributesColor(
        m_outCloudItem.completeName(), outResult,
        ct_cloud->getPointCloudIndexRegistered(), param._colors);
    group->addItemDrawable(colorAttribute);
  }
  _paramList.clear();
}

void SF_RadiusOutlierFilterStep::writeLogger() {
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

void SF_RadiusOutlierFilterStep::adaptParametersToExpertLevel() {
  if (!_isExpert) {
    _radius = 0.03;
    if (_choiceNumberPoints == _few) {
      _minPts = 5;
    } else if (_choiceNumberPoints == _intermediate) {
      _minPts = 18;
    } else if (_choiceNumberPoints == _clearSky) {
      _radius = 1.5;
      _minPts = 1000;
    } else if (_choiceNumberPoints == _many) {
      _minPts = 40;
    } else {
      throw std::runtime_error(
          "SF_RadiusOutlierFilterStep::adaptParametersToExpertLevel() illegal "
          "setting.");
    }
  }
}

void SF_RadiusOutlierFilterStep::createParamList(CT_ResultGroup *outResult) {
  adaptParametersToExpertLevel();
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup *group = (CT_StandardItemGroup *)outResIt.next();
    const CT_AbstractItemDrawableWithPointCloud *ctCloud =
        (const CT_AbstractItemDrawableWithPointCloud *)
            group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
    SF_ParamRadiusOutlierFilter<SF_PointNormal> param;
    param._log = PS_LOG;
    param._radius = _radius;
    param._minPts = _minPts;
    param._sizeOutput = 2;
    param._itemCpyCloudIn = ctCloud;
    param._grpCpyGrp = group;
    _paramList.append(param);
  }
}
