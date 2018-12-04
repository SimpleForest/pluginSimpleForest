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

#include "sf_stepMergeClouds.h"
#include "steps/filter/binary/statistical_outlier_filter/sf_statisticalOutlierRemovalAdapter.h"
#include <QtConcurrent/QtConcurrent>

#define DEF_IN_GRP1 "grp1"
#define DEF_IN_GRP2 "grp2"
#define DEF_IN_CLOUD1 "cloud1"
#define DEF_IN_CLOUD2 "cloud2"

SF_StepMergeClouds::SF_StepMergeClouds(CT_StepInitializeData &dataInit)
    : SF_AbstractFilterBinaryStep(dataInit) {}

SF_StepMergeClouds::~SF_StepMergeClouds() {}

QString SF_StepMergeClouds::getStepDescription() const {
  return tr("Merge clouds from two groups.");
}

QString SF_StepMergeClouds::getStepDetailledDescription() const {
  return tr("Merge clouds from two groups- Two clouds stored in two different "
            "result groups are selected. The first cloud of group 1 ist merged "
            "with the first cloud of group 2 and so on."
            "The second and first group should be derived from the same parent "
            "group, so two matched clouds contain points originally belonging "
            "together.");
}

QString SF_StepMergeClouds::getStepURL() const { return tr(""); }

CT_VirtualAbstractStep *
SF_StepMergeClouds::createNewInstance(CT_StepInitializeData &dataInit) {
  return new SF_StepMergeClouds(dataInit);
}

QStringList SF_StepMergeClouds::getStepRISCitations() const {
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  return _risCitationList;
}

void SF_StepMergeClouds::createInResultModelListProtected() {
  CT_InResultModelGroupToCopy *resModel =
      createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
  assert(resModel != NULL);
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel(
      "", DEF_IN_GRP1, CT_AbstractItemGroup::staticGetType(), tr("First Group"),
      "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP1, DEF_IN_CLOUD1, CT_Scene::staticGetType(),
                         tr("First Cloud"));
  resModel->addGroupModel(
      "", DEF_IN_GRP2, CT_AbstractItemGroup::staticGetType(),
      tr("Second Group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP2, DEF_IN_CLOUD2, CT_Scene::staticGetType(),
                         tr("Second Cloud"));
}

void SF_StepMergeClouds::createPostConfigurationDialogExpert(
    CT_StepConfigurableDialog *configDialog) {
  configDialog->addText("Two clouds stored in two different result groups are "
                        "selected. The first cloud of group 1 ist merged with "
                        "the first cloud of group 2 and so on.");
  configDialog->addText("The second and first group should be derived from the "
                        "same parent group, so two matched clouds contain "
                        "points originally belonging together.");
  configDialog->addText("The first group will also contain the merged result.");
}

void SF_StepMergeClouds::createPostConfigurationDialogBeginner(
    CT_StepConfigurableDialog *configDialog) {
  configDialog->addText("Two clouds stored in two different result groups are "
                        "selected. The first cloud of group 1 ist merged with "
                        "the first cloud of group 2 and so on.");
  configDialog->addText("The second and first group should be derived from the "
                        "same parent group, so two matched clouds contain "
                        "points originally belonging together.");
  configDialog->addText("The first group will also contain the merged result.");
}

void SF_StepMergeClouds::createOutResultModelListProtected() {
  CT_OutResultModelGroupToCopyPossibilities *resModelw =
      createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    resModelw->addItemModel(DEF_IN_GRP1, _outCloud, new CT_Scene(),
                            tr("Merged Clouds"));
  }
}

void SF_StepMergeClouds::adaptParametersToExpertLevel() {}

void SF_StepMergeClouds::writeOutputPerScence(CT_ResultGroup *outResult,
                                              size_t i) {}

void SF_StepMergeClouds::writeOutput(CT_ResultGroup *outResult) {}

void SF_StepMergeClouds::compute() {
  const QList<CT_ResultGroup *> &outResultList = getOutResultList();
  CT_ResultGroup *outResult = outResultList.at(0);
  CT_ResultGroupIterator outResIt1(outResult, this, DEF_IN_GRP1);
  CT_ResultGroupIterator outResIt2(outResult, this, DEF_IN_GRP2);
  std::vector<CT_AbstractItemDrawableWithPointCloud *> clouds1;
  std::vector<CT_AbstractItemDrawableWithPointCloud *> clouds2;
  while (!isStopped() && outResIt1.hasNext()) {
    CT_StandardItemGroup *group = (CT_StandardItemGroup *)outResIt1.next();
    CT_AbstractItemDrawableWithPointCloud *ct_cloud =
        (CT_AbstractItemDrawableWithPointCloud *)group->firstItemByINModelName(
            this, DEF_IN_CLOUD1);
    clouds1.push_back(ct_cloud);
  }
  while (!isStopped() && outResIt2.hasNext()) {
    CT_StandardItemGroup *group = (CT_StandardItemGroup *)outResIt2.next();
    CT_AbstractItemDrawableWithPointCloud *ct_cloud =
        (CT_AbstractItemDrawableWithPointCloud *)group->firstItemByINModelName(
            this, DEF_IN_CLOUD2);
    clouds2.push_back(ct_cloud);
  }
  std::cout << clouds1.size() << " ; " << clouds2.size() << std::endl;
  if (clouds1.size() == clouds2.size()) {
    CT_ResultGroupIterator outResIt3(outResult, this, DEF_IN_GRP1);
    for (size_t i = 0; i < clouds1.size(); i++) {
      CT_StandardItemGroup *group = (CT_StandardItemGroup *)outResIt3.next();
      CT_AbstractItemDrawableWithPointCloud *cloud1 = clouds1[i];
      CT_AbstractItemDrawableWithPointCloud *cloud2 = clouds2[i];
      const CT_AbstractPointCloudIndex *indices1 = cloud1->getPointCloudIndex();
      const CT_AbstractPointCloudIndex *indices2 = cloud2->getPointCloudIndex();
      CT_PointCloudIndexVector *vec = new CT_PointCloudIndexVector();
      std::vector<size_t> newIndices;
      CT_PointIterator it1(indices1);
      while (it1.hasNext()) {
        newIndices.push_back(it1.next().currentGlobalIndex());
      }
      CT_PointIterator it2(indices2);
      while (it2.hasNext()) {
        newIndices.push_back(it2.next().currentGlobalIndex());
      }
      std::sort(newIndices.begin(), newIndices.end());
      std::for_each(newIndices.begin(), newIndices.end(),
                    [vec](size_t index) { vec->addIndex(index); });
      CT_Scene *outScene =
          new CT_Scene(_outCloud.completeName(), outResult,
                       PS_REPOSITORY->registerPointCloudIndex(vec));
      outScene->updateBoundingBox();
      group->addItemDrawable(outScene);
    }
  }
}

void SF_StepMergeClouds::writeLogger() {}

void SF_StepMergeClouds::createParamList(CT_ResultGroup *outResult) {}
