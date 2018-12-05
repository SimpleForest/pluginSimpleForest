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
#ifndef SF_ABSTRACT_STEP_H
#define SF_ABSTRACT_STEP_H

#include <QFuture>
#include <QObject>
#include <QString>
#include <QStringList>
#include <ct_itemdrawable/ct_grid3d_sparse.h>
#include <ct_itemdrawable/ct_scene.h>
#include <ct_iterator/ct_pointiterator.h>
#include <ct_pointcloudindex/ct_pointcloudindexvector.h>
#include <ct_result/ct_resultgroup.h>
#include <ct_result/model/inModel/ct_inresultmodelgrouptocopy.h>
#include <ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h>
#include <ct_step/abstract/ct_abstractstep.h>
#include <ct_view/ct_stepconfigurabledialog.h>
#include <steps/param/sf_paramAllSteps.h>

#include "cloud/filter/multiple/voxel/sf_filtervoxelclustering.h"
#include "converters/CT_To_PCL/sf_converterCTCloudToPCLCloud.h"

#define DEF_IN_RESULT "ires"
#define DEF_IN_GRP_CLUSTER "igrp"
#define DEF_IN_RESULT_DTM "idtmres"
#define DEF_IN_DTMGRP "igrpdtm"
#define DEF_IN_DTM "idtm"
#define DEF_IN_CLOUD_SEED "icloud"
#define DEF_IN_SCENE "igrp2"
#define DEF_IN_SCENE_CLOUD "icloud2"

class SF_AbstractStep : public CT_AbstractStep {
  Q_OBJECT

  QList<CT_AbstractItemGroup *> _groupsToBeRemoved;
  void checkIsEmpty(CT_StandardItemGroup *group,
                    const CT_AbstractItemDrawableWithPointCloud *ctCloud);
  void checkIsNullOrEmpty(const CT_AbstractItemDrawableWithPointCloud *ct_cloud,
                          CT_StandardItemGroup *group);
  void checkGrpAndCloud(CT_StandardItemGroup *group);
  void identifyCorruptedScenes(CT_ResultGroup *outResult, int progress = 4);
  void removeCorruptedScenes(int progress = 7);

protected:
  virtual void createInResultModelListProtected() = 0;
  virtual void createOutResultModelListProtected() = 0;
  void createPreConfigurationDialog();
  void createPostConfigurationDialog();
  virtual void createPostConfigurationDialogBeginner(
      CT_StepConfigurableDialog *configDialog) = 0;
  virtual void createPostConfigurationDialogExpert(
      CT_StepConfigurableDialog *configDialog) = 0;
  virtual void writeLogger();
  virtual void createPostConfigurationDialogCitation(
      CT_StepConfigurableDialog *configDialog);
  virtual void createPostConfigurationDialogCitationSecond(
      CT_StepConfigurableDialog *configDialog) {
    configDialog;
  }

  virtual void adaptParametersToExpertLevel() = 0;
  virtual void compute() = 0;

  void recursiveRemoveIfEmpty(CT_AbstractItemGroup *parent,
                              CT_AbstractItemGroup *group);
  void setProgressByFuture(QFuture<void> &future, float percentageIntervalStart,
                           float percentageIntervalSize);
  void createOutputIndices(
      std::vector<CT_PointCloudIndexVector *> &indexVectors,
      const std::vector<int> &indices,
      const CT_AbstractItemDrawableWithPointCloud *itemCpyCloudIn);
  void identifyAndRemoveCorruptedScenes(CT_ResultGroup *out_result);
  void createOutputIndex(std::vector<CT_PointCloudIndexVector *> &indexVectors,
                         const std::vector<int> &indices, size_t counter,
                         CT_PointIterator &pointIt);
  CT_Scene *mergeIndices(CT_ResultGroup *outResult, CT_StandardItemGroup *root,
                         const QString defInnGrp, const QString defInCloud);
  Eigen::Vector3f getMin(const CT_Scene *ctCloud);
  Eigen::Vector3f getMax(const CT_Scene *ctCloud);
  std::vector<CT_PointCloudIndexVector *>
  createOutputVectors(size_t numberOutput);

  QList<SF_ParamCT> _paramList;

  bool _isExpert = true;
  QString _RANSAC = "RANSAC  - RANdom SAmple Consensus";
  QString _LMEDS = "LMEDS   - Least Median of Squares";
  QString _MSAC = "MSAC    - M-Estimator SAmple Consensus";
  QString _RRANSAC = "RRANSAC - Randomized RANSAC";
  QString _RMSAC = "RMSAC   - Randomized MSAC";
  QString _MLESAC = "MLESAC  - Maximum LikeLihood Estimation SAmple Consensus";
  QString _PROSAC = "PROSAC  - PROgressive SAmple Consensus";
  QString _SF_methodChoice = _MLESAC;
  QStringList _SF_methodList;

  QString _ZEROMOMENTUMORDER =
      "ZEROMOMENTUMORDER       - maximize inliers of model";
  QString _FIRSTMOMENTUMORDERMSAC = "FIRSTMOMENTUMORDERMSAC  - minimize "
                                    "cropped (MSAC) average absolute distance";
  QString _FIRSTMOMENTUMORDER =
      "FIRSTMOMENTUMORDER      - minimize average absolute distance";
  QString _SECONDMOMENTUMORDERMSAC =
      "SECONDMOMENTUMORDERMSAC - minimize cropped (MSAC) root squared distance";
  QString _SECONDMOMENTUMORDER =
      "SECONDMOMENTUMORDER     - minimize average root squared distance";
  QString _CMD_methodChoice = _SECONDMOMENTUMORDERMSAC;
  QStringList _CMD_methodList;

  QStringList _pointDensities;
  QString _lowDensity = "low";
  QString _mediumDensity = "medium";
  QString _highDensity = "high";
  QString _choicePointDensity = _mediumDensity;

  QStringList _numberPoints;
  QString _few = "few";
  QString _intermediate = "intermediate";
  QString _many = "many";
  QString _choiceNumberPoints;

  const QString getRISCitationSimpleTree() const;
  const QString getRISCitationPCL() const;
  const QString getRISCitationRaumonen() const;
  void addCitationRaumonen(CT_StepConfigurableDialog *configDialog);

public:
  SF_AbstractStep(CT_StepInitializeData &dataInit);
  virtual CT_VirtualAbstractStep *
  createNewInstance(CT_StepInitializeData &dataInit) = 0;
};

#endif // SF_ABSTRACT_STEP_H
