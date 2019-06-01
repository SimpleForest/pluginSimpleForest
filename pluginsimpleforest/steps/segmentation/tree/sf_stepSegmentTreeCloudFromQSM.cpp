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

#include "sf_stepSegmentTreeCloudFromQSM.h"

#include "steps/item/sf_qsm_item.h"
#include "steps/segmentation/tree/sf_stepSegmentTreeCloudFromQSMAdapter.h"

#include <ct_itemdrawable/ct_pointsattributescolor.h>
#include <ct_itemdrawable/ct_pointsattributesscalartemplated.h>

#include <QtConcurrent/QtConcurrent>

SF_StepSegmentTreeCloudFromQSM::SF_StepSegmentTreeCloudFromQSM(CT_StepInitializeData& dataInit) : SF_AbstractStepSegmentation(dataInit)
{}

SF_StepSegmentTreeCloudFromQSM::~SF_StepSegmentTreeCloudFromQSM() {}

QString
SF_StepSegmentTreeCloudFromQSM::getStepDescription() const
{
  return tr("Clusters a tree cloud with the help of a QSM");
}

QString
SF_StepSegmentTreeCloudFromQSM::getStepDetailledDescription() const
{
  return tr("Each point is allocated to its nearest building brick"
            " of a QSM. The point retrieves the growth length of its nearest"
            " brick. According to their growth length, the tree points are"
            " subdivided into n clusters.");
}

QString
SF_StepSegmentTreeCloudFromQSM::getStepURL() const
{
  return tr("");
}

CT_VirtualAbstractStep*
SF_StepSegmentTreeCloudFromQSM::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepSegmentTreeCloudFromQSM(dataInit);
}

QStringList
SF_StepSegmentTreeCloudFromQSM::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepSegmentTreeCloudFromQSM::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Result"));
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel("",
                          DEF_IN_GRP_CLUSTER,
                          CT_AbstractItemGroup::staticGetType(),
                          tr("Tree Group"),
                          "",
                          CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Tree Cloud"));
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_QSM, SF_QSM_Item::staticGetType(), tr("QSM"));
}

void
SF_StepSegmentTreeCloudFromQSM::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  configDialog->addInt("Sorting by growth length the cloud is subdivided in  "
                       "[<em><b>numClstrs</b></em>] ",
                       " clusters.",
                       2,
                       6,
                       _numClstrs);
  createPostConfigurationDialogCitation(configDialog);
}

void
SF_StepSegmentTreeCloudFromQSM::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    resModelw->addItemAttributeModel(
      _outCloudCluster, m_clusterIndices, new CT_StdItemAttributeT<int>(CT_AbstractCategory::DATA_VALUE), tr("Tree Segment ID"));
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, m_outColorGrowthVolume, new CT_PointsAttributesColor(), tr("log GrowthVolume"));
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, m_outColorClusters, new CT_PointsAttributesColor(), tr("Cluster Colors"));
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, m_outClusterID, new CT_PointsAttributesScalarTemplated<int>(), tr("Cluster ID"));
  }
}

void
SF_StepSegmentTreeCloudFromQSM::compute()
{
  const QList<CT_ResultGroup*>& outResultList = getOutResultList();
  CT_ResultGroup* outResult = outResultList.at(0);
  identifyAndRemoveCorruptedScenes(outResult);
  createParamList(outResult);
  writeLogger();
  QFuture<void> future = QtConcurrent::map(_paramList, SF_SegmentTreeCloudFromQSMAdapter());
  setProgressByFuture(future, 10.0f, 85.0f);
  size_t index = 0;
  CT_ResultGroupIterator outResIt2(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt2.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt2.next();
    const CT_AbstractItemDrawableWithPointCloud* ct_cloud =
      (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
    SF_ParamSegmentTreeFromQSM<pcl::PointXYZINormal> param = _paramList[index++];
    CT_PointsAttributesColor* colorAttributeClusters = new CT_PointsAttributesColor(
      m_outColorClusters.completeName(), outResult, ct_cloud->getPointCloudIndexRegistered(), param._colorsClusters);
    group->addItemDrawable(colorAttributeClusters);
    CT_PointsAttributesColor* colorAttribute = new CT_PointsAttributesColor(
      m_outColorGrowthVolume.completeName(), outResult, ct_cloud->getPointCloudIndexRegistered(), param._colorsGrowthVolume);
    CT_PointsAttributesScalarTemplated<int>* idAttribute = new CT_PointsAttributesScalarTemplated<int>(
      m_outClusterID.completeName(), outResult, ct_cloud->getPointCloudIndexRegistered(), param._clusterIDs, 0, param._numClstrs - 1);
    group->addItemDrawable(idAttribute);
    group->addItemDrawable(colorAttribute);
  }
  _paramList.clear();
}

void
SF_StepSegmentTreeCloudFromQSM::createParamList(CT_ResultGroup* outResult)
{
  SF_CloudToModelDistanceParameters distanceParams;
  distanceParams._cropDistance = 1000000;
  distanceParams._k = 5;
  distanceParams._method = SF_CLoudToModelDistanceMethod::GROWTHDISTANCE;
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(
      this, DEF_IN_CLOUD_SEED);
    const SF_QSM_Item* sf_qsmItem = (const SF_QSM_Item*)group->firstItemByINModelName(this, DEF_IN_QSM);
    SF_ParamSegmentTreeFromQSM<pcl::PointXYZINormal> param;
    param._stepProgress = _stepProgress;
    param._distanceParams = distanceParams;
    param._numClstrs = _numClstrs;
    param._log = PS_LOG;
    param._itemCpyCloudIn = ctCloud;
    param._grpCpyGrp = group;
    param._qsm = sf_qsmItem->getQsm();
    _paramList.append(param);
  }
}
