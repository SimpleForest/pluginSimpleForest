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

#include "sf_abstractStep.h"
#include "qsm/algorithm/distance/sf_cloudToModelDistance.h"
#include "steps/item/sf_spherefollowing_parameters_item.h"
#include <pcl/sample_consensus/method_types.h>

#include <ct_itemdrawable/ct_cylinder.h>

SF_AbstractStep::SF_AbstractStep(CT_StepInitializeData& dataInit) : CT_AbstractStep(dataInit)
{
  _progress = 0;
  _computationsDone = 0;
  _computationsTotal = 1;

  _SF_methodList.push_back(_RANSAC);
  _SF_methodList.push_back(_LMEDS);
  _SF_methodList.push_back(_MSAC);
  _SF_methodList.push_back(_RRANSAC);
  _SF_methodList.push_back(_RMSAC);
  _SF_methodList.push_back(_MLESAC);
  _SF_methodList.push_back(_PROSAC);

  _CMD_methodList.push_back(_ZEROMOMENTUMORDER);
  _CMD_methodList.push_back(_FIRSTMOMENTUMORDERMSAC);
  _CMD_methodList.push_back(_FIRSTMOMENTUMORDER);
  _CMD_methodList.push_back(_SECONDMOMENTUMORDERMSAC);
  _CMD_methodList.push_back(_SECONDMOMENTUMORDER);

  _pointDensities.push_back(_lowDensity);
  _pointDensities.push_back(_mediumDensity);
  _pointDensities.push_back(_highDensity);

  _numberPoints.push_back(_few);
  _numberPoints.push_back(_intermediate);
  _numberPoints.push_back(_many);

  _stepProgress.reset(new SF_StepProgress());
  QObject::connect(&(*_stepProgress), SIGNAL(computationDone()), this, SLOT(computationDone()));
}

void
SF_AbstractStep::computationDone()
{
  _computationsDone = _computationsDone + 1;
  _progress = _computationsDone / _computationsTotal;
}

void
SF_AbstractStep::addQSM(CT_ResultGroup* outResult,
                        QList<SF_ParamSpherefollowingAdvanced<SF_PointNormal>> paramList,
                        QString outResultGrpName,
                        QString outCylinderName,
                        QString outCylinderGrpName,
                        QString outSFQSMName,
                        QString outParamName)
{
  CT_ResultGroupIterator outResIt(outResult, this, outResultGrpName);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    std::for_each(
      paramList.begin(),
      paramList.end(),
      [this, group, outResult, outCylinderName, outCylinderGrpName, outSFQSMName, outParamName](
        SF_ParamSpherefollowingAdvanced<SF_PointNormal>& params) {
        std::shared_ptr<SF_ModelQSM> qsm = params._tree;
        SF_QSM_Item* qsmItem = new SF_QSM_Item(outSFQSMName, outResult, qsm);
        group->addItemDrawable(qsmItem);
        params.reset();
        SF_SphereFollowing_Parameters_Item* paramItem = new SF_SphereFollowing_Parameters_Item(outParamName, outResult, params);
        group->addItemDrawable(paramItem);
        std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = qsm->getBuildingBricks();
        std::for_each(buildingBricks.begin(),
                      buildingBricks.end(),
                      [&params, this, outResult, group, outCylinderName, outCylinderGrpName](
                        std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick) {
                        Eigen::Vector3f start = buildingBrick->getStart();
                        Eigen::Vector3f end = buildingBrick->getEnd();
                        double radius = buildingBrick->getRadius();
                        double length = buildingBrick->getLength();
                        CT_CylinderData* data = new CT_CylinderData(Eigen::Vector3d(static_cast<double>((start[0] + end[0]) / 2),
                                                                                    static_cast<double>((start[1] + end[1]) / 2),
                                                                                    static_cast<double>((start[2] + end[2]) / 2)),
                                                                    Eigen::Vector3d(static_cast<double>(end[0] - start[0]),
                                                                                    static_cast<double>(end[1] - start[1]),
                                                                                    static_cast<double>(end[2] - start[2])),
                                                                    radius,
                                                                    length);
                        CT_Cylinder* cylinder = new CT_Cylinder(outCylinderName, outResult, data);
                        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(outCylinderGrpName, outResult);
                        group->addGroup(cylinderGroup);
                        cylinderGroup->addItemDrawable(cylinder);
                      });
      });
  }
}

void
SF_AbstractStep::addQSM(CT_ResultGroup* outResult,
                        QList<SF_ParamSpherefollowingBasic<SF_PointNormal>> paramList,
                        QString outResultGrpName,
                        QString outCylinderName,
                        QString outCylinderGrpName,
                        QString outSFQSMName,
                        QString outParamName)
{
  CT_ResultGroupIterator outResIt(outResult, this, outResultGrpName);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    std::for_each(
      paramList.begin(),
      paramList.end(),
      [this, group, outResult, outCylinderName, outCylinderGrpName, outSFQSMName, outParamName](
        SF_ParamSpherefollowingBasic<SF_PointNormal>& params) {
        std::shared_ptr<SF_ModelQSM> qsm = params._tree;
        SF_QSM_Item* qsmItem = new SF_QSM_Item(outSFQSMName, outResult, qsm);
        group->addItemDrawable(qsmItem);
        params.reset();
        SF_SphereFollowing_Parameters_Item* paramItem = new SF_SphereFollowing_Parameters_Item(outParamName, outResult, params);
        group->addItemDrawable(paramItem);
        std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = qsm->getBuildingBricks();
        std::for_each(buildingBricks.begin(),
                      buildingBricks.end(),
                      [&params, this, outResult, group, outCylinderName, outCylinderGrpName](
                        std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick) {
                        Eigen::Vector3f start = buildingBrick->getStart();
                        Eigen::Vector3f end = buildingBrick->getEnd();
                        double radius = buildingBrick->getRadius();
                        double length = buildingBrick->getLength();
                        CT_CylinderData* data = new CT_CylinderData(Eigen::Vector3d(static_cast<double>((start[0] + end[0]) / 2),
                                                                                    static_cast<double>((start[1] + end[1]) / 2),
                                                                                    static_cast<double>((start[2] + end[2]) / 2)),
                                                                    Eigen::Vector3d(static_cast<double>(end[0] - start[0]),
                                                                                    static_cast<double>(end[1] - start[1]),
                                                                                    static_cast<double>(end[2] - start[2])),
                                                                    radius,
                                                                    length);
                        CT_Cylinder* cylinder = new CT_Cylinder(outCylinderName, outResult, data);
                        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(outCylinderGrpName, outResult);
                        group->addGroup(cylinderGroup);
                        cylinderGroup->addItemDrawable(cylinder);
                      });
      });
  }
}

void
SF_AbstractStep::addColors(CT_ResultGroup* outResult,
                           QList<SF_ParamQSM<SF_PointNormal>> paramList,
                           QString outResultGrpName,
                           QString outColorGrpName,
                           QString outColorName)
{
  size_t index = 0;
  CT_ResultGroupIterator outResIt2(outResult, this, outResultGrpName);
  while (!isStopped() && outResIt2.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt2.next();
    const CT_AbstractItemDrawableWithPointCloud* ct_cloud =
      (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(this, outColorGrpName);
    SF_ParamQSM<SF_PointNormal> param = paramList[index++];
    CT_PointsAttributesColor* colorAttribute = new CT_PointsAttributesColor(
      outColorName, outResult, ct_cloud->getPointCloudIndexRegistered(), param._colors);
    group->addItemDrawable(colorAttribute);
  }
}

void
SF_AbstractStep::addCitationRaumonen(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText(QObject::tr("For this step please cite in addition:"),
                        "Pasi Raumonen, Mikko Kaasalainen, Markku Åkerblom, "
                        "Sanna Kaasalainen, Harri Kaartinen, Mikko Vastaranta, "
                        "Markus Holopainen, Mathias Disney and Philip Lewis");
  configDialog->addText("",
                        "<em>Fast Automatic Precision Tree Models from "
                        "Terrestrial Laser Scanner Data.</em>");
  configDialog->addText("", "Remote Sensing<b>2013</b>, 5, 491-520.");
  configDialog->addEmpty();
}

void
SF_AbstractStep::addCitationPCL(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText(QObject::tr("For this step please cite in addition:"), "Rusu, Radu Bogdan and Cousins, Steve");
  configDialog->addText("", "<em>3d is here: Point cloud library (pcl).</em>");
  configDialog->addText("", "Robotics and Automation (ICRA)<b>2011</b>, IEEE International.");
  configDialog->addEmpty();
}

void
SF_AbstractStep::recursiveRemoveIfEmpty(CT_AbstractItemGroup* parent, CT_AbstractItemGroup* group)
{
  if (parent != NULL) {
    parent->removeGroup(group);
    if (parent->isEmpty()) {
      recursiveRemoveIfEmpty(parent->parentGroup(), parent);
    }
  } else {
    ((CT_ResultGroup*)group->result())->removeGroupSomethingInStructure(group);
  }
}

void
SF_AbstractStep::setProgressByFuture(QFuture<void>& future, float percentageIntervalStart, float percentageIntervalSize)
{
  float progressMin = future.progressMinimum();
  float progressSize = future.progressMaximum() - progressMin;
  while (!future.isFinished()) {
    setProgress(percentageIntervalStart + (percentageIntervalSize * (future.progressValue() - progressMin) / progressSize));
  }
}

void
SF_AbstractStep::setProgressByCounter(float percentageIntervalStart, float percentageIntervalSize)
{
  setProgress(percentageIntervalStart + (percentageIntervalSize * _progress));
}

CT_Scene*
SF_AbstractStep::mergeIndices(CT_ResultGroup* outResult, CT_StandardItemGroup* root, const QString defInnGrp, const QString defInCloud)
{
  CT_ResultGroupIterator outResIt(outResult, this, defInnGrp);
  CT_PointCloudIndexVector* mergedClouds = new CT_PointCloudIndexVector();
  mergedClouds->setSortType(CT_AbstractCloudIndex::NotSorted);
  std::vector<size_t> indices;
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(
      this, defInCloud);
    CT_PointIterator iter(ctCloud->getPointCloudIndex());
    while (iter.hasNext() && !isStopped()) {
      iter.next();
      size_t index = iter.currentGlobalIndex();
      indices.push_back(index);
    }
  }
  std::sort(indices.begin(), indices.end());
  for (size_t i = 0; i < indices.size(); i++) {
    mergedClouds->addIndex(indices.at(i));
  }
  mergedClouds->setSortType(CT_PointCloudIndexVector::SortedInAscendingOrder);
  CT_Scene* scene = new CT_Scene(defInCloud, outResult, PS_REPOSITORY->registerPointCloudIndex(mergedClouds));
  return scene;
}

void
SF_AbstractStep::createPreConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPreConfigurationDialog();
  configDialog->addBool("Uncheck to deactivate parameterization possibilities "
                        "of this step. Only recommended for beginners",
                        "",
                        "expert",
                        _isExpert);
}

Eigen::Vector3f
SF_AbstractStep::getMin(const CT_Scene* ctCloud)
{
  Eigen::Vector3f min(ctCloud->minX(), ctCloud->minY(), ctCloud->minZ());
  return min;
}

Eigen::Vector3f
SF_AbstractStep::getMax(const CT_Scene* ctCloud)
{
  Eigen::Vector3f max(ctCloud->maxX(), ctCloud->maxY(), ctCloud->maxZ());
  return max;
}

void
SF_AbstractStep::identifyAndRemoveCorruptedScenes(CT_ResultGroup* outResult)
{
  identifyCorruptedScenes(outResult);
  removeCorruptedScenes();
}

void
SF_AbstractStep::checkIsEmpty(CT_StandardItemGroup* group, const CT_AbstractItemDrawableWithPointCloud* ctCloud)
{
  if (ctCloud->getPointCloudIndex()->size() <= 0) {
    _groupsToBeRemoved.push_back(group);
  }
}

void
SF_AbstractStep::checkIsNullOrEmpty(const CT_AbstractItemDrawableWithPointCloud* ctCloud, CT_StandardItemGroup* group)
{
  if (ctCloud != NULL) {
    checkIsEmpty(group, ctCloud);
  } else {
    _groupsToBeRemoved.push_back(group);
  }
}

void
SF_AbstractStep::checkGrpAndCloud(CT_StandardItemGroup* group)
{
  if (group != NULL) {
    const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(
      this, DEF_IN_CLOUD_SEED);
    checkIsNullOrEmpty(ctCloud, group);
  } else {
    _groupsToBeRemoved.push_back(group);
  }
}

void
SF_AbstractStep::identifyCorruptedScenes(CT_ResultGroup* outResult, int progress)
{
  _groupsToBeRemoved.clear();
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    checkGrpAndCloud(group);
  }
  setProgress(progress);
}

void
SF_AbstractStep::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  if (!_isExpert) {
    createPostConfigurationDialogBeginner(configDialog);
  } else {
    createPostConfigurationDialogExpert(configDialog);
  }
  createPostConfigurationDialogCitation(configDialog);
}

void
SF_AbstractStep::createPostConfigurationDialogCitation(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addEmpty();
  configDialog->addText(QObject::tr("For general usage of the SimpleForest plugin please cite "
                                    "the following:"),
                        "Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P.");
  configDialog->addText("",
                        "<em>SimpleTree - An Efficient Open Source Tool to "
                        "Build Tree Models from TLS Clouds.</em>");
  configDialog->addText("", "Forests <b>2015</b>, 6, 4245-4294.");
  configDialog->addEmpty();
  createPostConfigurationDialogCitationSecond(configDialog);
}

void
SF_AbstractStep::removeCorruptedScenes(int progress)
{
  while (!_groupsToBeRemoved.isEmpty()) {
    CT_AbstractItemGroup* group = _groupsToBeRemoved.takeLast();
    recursiveRemoveIfEmpty(group->parentGroup(), group);
  }
  setProgress(progress);
}

std::vector<CT_PointCloudIndexVector*>
SF_AbstractStep::createOutputVectors(size_t numberOutput)
{
  std::vector<CT_PointCloudIndexVector*> result;
  for (size_t i = 0; i < numberOutput; i++) {
    CT_PointCloudIndexVector* vec = new CT_PointCloudIndexVector();
    result.push_back(vec);
  }
  return result;
}

const QString
SF_AbstractStep::getRISCitationSimpleTree() const
{
  return QString("TY  - JOUR\n"
                 "T1  - SimpleTree - an efficient open source tool to build "
                 "tree models from TLS clouds\n"
                 "A1  - Hackenberg, Jan\n"
                 "A1  - Spiecker, Heinrich\n"
                 "A1  - Calders, Kim\n"
                 "A1  - Disney, Mathias\n"
                 "A1  - Raumonen, Pasi\n"
                 "JO  - Forests\n"
                 "VL  - 6\n"
                 "IS  - 11\n"
                 "SP  - 4245\n"
                 "EP  - 4294\n"
                 "Y1  - 2015\n"
                 "PB  - Multidisciplinary Digital Publishing Institute\n"
                 "UL  - http://www.simpletree.uni-freiburg.de/\n"
                 "ER  - \n");
}

const QString
SF_AbstractStep::getRISCitationPCL() const
{
  return QString("TY  - CONF\n"
                 "T1  - 3d is here: Point cloud library (pcl)\n"
                 "A1  - Rusu, Radu Bogdan\n"
                 "A1  - Cousins, Steve\n"
                 "JO  - Robotics and Automation (ICRA), 2011 IEEE International "
                 "Conference on\n"
                 "SP  - 1\n"
                 "EP  - 4\n"
                 "SN  - 1612843859\n"
                 "Y1  - 2011\n"
                 "PB  - IEEE\n"
                 "UL  - "
                 "http://pointclouds.org/documentation/tutorials/statistical_outlier.php\n"
                 "ER  - \n");
}

const QString
SF_AbstractStep::getRISCitationRaumonen() const
{
  return QString("TY  - EJOU\n"
                 "T1  - Fast Automatic Precision Tree Models from Terrestrial "
                 "Laser Scanner Data\n"
                 "A1  - Raumonen, Pasi\n"
                 "A1  - Kaasalainen, Mikko\n"
                 "A1  - Akerblom, Markku\n"
                 "A1  - Kaasalainen, Sanna\n"
                 "A1  - Kaartinen, Harri\n"
                 "A1  - Vastaranta, Mikko\n"
                 "A1  - Holopainen, Markus\n"
                 "A1  - Disney, Mathias\n"
                 "A1  - Lewis, Philip\n"
                 "JO  - Remote Sensing\n"
                 "SP  - 5\n"
                 "EP  - 2\n"
                 "SN  - 2072-4292\n"
                 "Y1  - 2013\n"
                 "PB  - MDPI\n"
                 "UL  - http://www.mdpi.com/2072-4292/5/2/491\n"
                 "ER  - \n");
}

const QString
SF_AbstractStep::getRISCitationSphereFollowing() const
{
  return QString("TY  - JOUR\n"
                 "T1  - Highly Accurate Tree Models Derived from Terrestrial "
                 "Laser Scan Data: A Method Description\n"
                 "A1  - Hackenberg, Jan\n"
                 "A1  - Sheppard, Jonathan\n"
                 "A1  - Spiecker, Heinrich\n"
                 "A1  - Disney, Mathias\n"
                 "JO  - Forests\n"
                 "VL  - 5\n"
                 "IS  - 5\n"
                 "SP  - 1069\n"
                 "EP  - 1105\n"
                 "Y1  - 2014\n"
                 "PB  - Multidisciplinary Digital Publishing Institute\n"
                 "UL  - http://www.mdpi.com/1999-4907/5/5/1069\n"
                 "ER  - \n");
}

void
SF_AbstractStep::createOutputIndex(std::vector<CT_PointCloudIndexVector*>& indexVectors,
                                   const std::vector<int>& indices,
                                   size_t counter,
                                   CT_PointIterator& pointIt)
{
  pointIt.next();
  size_t indexCt = pointIt.currentGlobalIndex();
  if (counter < indices.size()) {
    size_t indexCloud = indices.at(counter);
    if (indexCloud < indexVectors.size()) {
      indexVectors[indexCloud]->addIndex(indexCt);
    } else {
      throw std::runtime_error("SF_AbstractStep::createOutputIndex(std::vector<"
                               "CT_PointCloudIndexVector *> &indexVectors,"
                               "const std::vector<int> &indices,"
                               "size_t counter,"
                               "CT_PointIterator & pointIt)");
    }
  } else {
    throw std::runtime_error("SF_AbstractStep::createOutputIndex(std::vector<"
                             "CT_PointCloudIndexVector *> &indexVectors,"
                             "const std::vector<int> &indices,"
                             "size_t counter,"
                             "CT_PointIterator & pointIt)");
  }
}

void
SF_AbstractStep::createOutputIndices(std::vector<CT_PointCloudIndexVector*>& indexVectors,
                                     const std::vector<int>& indices,
                                     const CT_AbstractItemDrawableWithPointCloud* itemCpyCloudIn)
{
  const CT_AbstractPointCloudIndex* pointCloudIndex = itemCpyCloudIn->getPointCloudIndex();
  CT_PointIterator pointIt(pointCloudIndex);
  size_t counter = 0;
  while (pointIt.hasNext()) {
    createOutputIndex(indexVectors, indices, counter++, pointIt);
  }
}

void
SF_AbstractStep::writeLogger()
{
  if (!_paramList.empty()) {
    QString str = _paramList[0].toString();
    PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
  }
}
