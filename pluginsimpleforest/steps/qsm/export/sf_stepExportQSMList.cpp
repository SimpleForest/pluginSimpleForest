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

#include "sf_stepExportQSMList.h"

#include "file/export/cloud/sf_exportCloud.h"
#include "file/export/ply/sf_exportPly.h"
#include "file/export/qsm/sf_exportQSM.h"
#include "steps/item/sf_spherefollowing_parameters_item.h"
#include "steps/qsm/postprocessing//sf_stepQSMAllometricCorrectionAdapter.h"

#include <ct_itemdrawable/ct_cylinder.h>
#include <ct_itemdrawable/ct_fileheader.h>

SF_StepExportQSMList::SF_StepExportQSMList(CT_StepInitializeData& dataInit) : SF_AbstractStepQSM(dataInit) {}

SF_StepExportQSMList::~SF_StepExportQSMList() {}

QString
SF_StepExportQSMList::getStepDescription() const
{
  return tr("QSM list exporter");
}

QString
SF_StepExportQSMList::getStepDetailledDescription() const
{
  return tr("This step utilizes is an exporter for a list of qsms. For each QSM there will be produced a topological ordered "
            "geometrical description file, the point cloud and"
            " various visuzalitations of the QSM.");
}

QString
SF_StepExportQSMList::getStepURL() const
{
  return tr("http://simpleforest.org/");
}

CT_VirtualAbstractStep*
SF_StepExportQSMList::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepExportQSMList(dataInit);
}

QStringList
SF_StepExportQSMList::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationSphereFollowing());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepExportQSMList::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  configDialog->addText("<b>QSM List exporter</b>:");
  configDialog->addFileChoice(tr("Select an export folder"), CT_FileChoiceButton::OneExistingFolder, "", m_filePath);

  configDialog->addBool("Write ply files visualizing the growth volume with color ", "", "", m_writePlyGrowthVolume);
  configDialog->addBool("Write ply files visualizing the growth length with color ", "", "", m_writePlyGrowthLengthPly);
  configDialog->addBool("Write ply files visualizing the stem with color ", "", "", m_writePlyStem);
  configDialog->addBool("The input cloud will be written ", "", "", m_writeCloud);
  configDialog->addBool("If input cloud is exported, will it be downscaled ", "", "", m_downScaleCloud);
  configDialog->addBool("Write ply files visualizing the with color ", "", "", m_downScaleCloud);
  configDialog->addDouble(" to a voxel size of ", " (m).", 0.01, 0.1, 2, m_voxelSize);
  configDialog->addBool("Write cloud files in csv format for colored growthLength ", "", "", m_writeCloudGrowthLength);
  configDialog->addBool("Write cloud files in csv format for colored radius ", "", "", m_writeCloudRadius);
  configDialog->addBool("Write cloud files in csv format for colored fit quality ", "", "", m_writeCloudFitQuality);
}

void
SF_StepExportQSMList::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Input for exporter"));
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel("",
                          DEF_IN_GRP_CLUSTER,
                          CT_AbstractItemGroup::staticGetType(),
                          tr("QSM Group"),
                          "",
                          CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_QSM, SF_QSM_Item::staticGetType(), tr("internal QSM"));

  CT_InResultModelGroup* resModelName = createNewInResultModel(DEF_IN_RESULT2, tr("Input for name"), "", true);
  resModelName->setZeroOrMoreRootGroup();
  resModelName->addGroupModel("", DEF_IN_GRP_CLUSTER3, CT_AbstractItemGroup::staticGetType(), tr("Name Group"));
  resModelName->addItemModel(DEF_IN_GRP_CLUSTER3, DEF_IN_NAME, CT_FileHeader::staticGetType(), tr("Name"));

  CT_InResultModelGroup* resModelName2 = createNewInResultModel(DEF_IN_RESULT3, tr("Input for name"), "", true);
  resModelName2->setZeroOrMoreRootGroup();
  resModelName2->addGroupModel("", DEF_IN_GRP_CLUSTER2, CT_AbstractItemGroup::staticGetType(), tr("Name Group"));
  resModelName2->addItemModel(DEF_IN_GRP_CLUSTER2, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("QSM cloud"));
}

void
SF_StepExportQSMList::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
}

void
SF_StepExportQSMList::compute()
{
  const QList<CT_ResultGroup*>& outResultList = getOutResultList();
  CT_ResultGroup* outResultName = getInputResults().at(1);
  CT_ResultGroupIterator outResItName(outResultName, this, DEF_IN_GRP_CLUSTER3);
  std::vector<QString> names;
  while (!isStopped() && outResItName.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResItName.next();
    CT_FileHeader* header = (CT_FileHeader*)group->firstItemByINModelName(this, DEF_IN_NAME);
    QString str = header->getFileName();
    QString cropedFilename = str.split(".", QString::SkipEmptyParts).at(0);
//    cropedFilename.append("_");
    names.push_back(cropedFilename);
  }

  std::vector<const CT_AbstractItemDrawableWithPointCloud*> cloudCTs;
  CT_ResultGroup* outResultCloud = getInputResults().at(2);
  CT_ResultGroupIterator outResItCloud2(outResultCloud, this, DEF_IN_GRP_CLUSTER2);
  while (!isStopped() && outResItCloud2.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResItCloud2.next();
    const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(
      this, DEF_IN_CLOUD_SEED);
    cloudCTs.push_back(ctCloud);
  }

  CT_ResultGroup* outResult = outResultList.at(0);

  CT_ResultGroupIterator outResItCloud(outResult, this, DEF_IN_GRP_CLUSTER);
  bool hasTranslation = false;
  SF_ExportPly exportPly;
  Eigen::Vector3d translation;
  QString path = m_filePath.first();
  size_t id = 0;
  size_t index = 0;
  if (m_filePath.size() > 0) {
    while (!isStopped() && outResItCloud.hasNext()) {
      CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResItCloud.next();
      index = std::min(index, names.size() - 1);
      QString name = names.at(index);
      const CT_AbstractItemDrawableWithPointCloud* ctCloud = cloudCTs[index];
      index++;
      const SF_QSM_Item* QSM_Item = (const SF_QSM_Item*)group->firstItemByINModelName(this, DEF_IN_QSM);
      auto qsm = QSM_Item->getQsm();

      Sf_ConverterCTToPCL<SF_PointNormal> converter;
      converter.setItemCpyCloudInDeprecated(ctCloud);
      if (hasTranslation) {
        converter.compute(translation);
      } else {
        converter.compute();
        translation = converter.translation();
        hasTranslation = true;
      }
      SF_CloudNormal::Ptr cloud = converter.cloudTranslated();
      qsm->translate(-translation);
      qsm->setID(id++);
      qsm->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_LENGTH, m_twigPercentage);
      if (m_writeCloud) {
        if (m_downScaleCloud) {
          SF_CloudNormal::Ptr cloudDownscaled(new SF_CloudNormal);
          pcl::VoxelGrid<SF_PointNormal> sor;
          sor.setInputCloud(cloud);
          sor.setLeafSize(m_voxelSize, m_voxelSize, m_voxelSize);
          sor.filter(*cloudDownscaled);
          cloud = cloudDownscaled;
        }
        SF_ExportCloud exportCloud;
        if (m_writeCloudFitQuality)
          exportCloud.exportCloud(path, name, qsm, cloud, SF_ExportCloudPolicy::FIT_QUALITY);
        if (m_writeCloudRadius)
          exportCloud.exportCloud(path, name, qsm, cloud, SF_ExportCloudPolicy::RADIUS);
        if (m_writeCloudGrowthLength)
          exportCloud.exportCloud(path, name, qsm, cloud, SF_ExportCloudPolicy::GROWTHLENGTH);
      }
      if (m_writePlyStem)
        exportPly.exportQSM(path, name, qsm, SF_ExportPlyPolicy::STEM);
      if (m_writePlyGrowthLengthPly)
        exportPly.exportQSM(path, name, qsm, SF_ExportPlyPolicy::GROWTH_LENGTH);
      if (m_writePlyGrowthVolume)
        exportPly.exportQSM(path, name, qsm, SF_ExportPlyPolicy::GROWTH_VOLUME);
      SF_ExportQSM exportQSM;
      exportQSM.exportQSM(path, name, qsm);
      qsm->translate(translation);
    }
  }
  writeLogger();
}
