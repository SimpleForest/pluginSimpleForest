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

#ifndef SF_STEP_SPHEREFOLLOWING_BASIC_H
#define SF_STEP_SPHEREFOLLOWING_BASIC_H

#include "steps/segmentation/sf_AbstractStepSegmentation.h"

class SF_StepSpherefollowingRoot : public SF_AbstractStepSegmentation
{
  Q_OBJECT

public:
  SF_StepSpherefollowingRoot(CT_StepInitializeData& dataInit);
  ~SF_StepSpherefollowingRoot();
  QString getStepDescription() const;
  QString getStepDetailledDescription() const;
  QString getStepURL() const;
  CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData& dataInit);
  QStringList getStepRISCitations() const;

protected:
  void createInResultModelListProtected();
  void createOutResultModelListProtected();
  void adaptParametersToExpertLevel();
  void createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog);
  void createParamList(CT_ResultGroup* out_result);
  virtual void createPreConfigurationDialog();
  virtual void createPostConfigurationDialog();

  void compute();
  QList<SF_ParamSpherefollowingBasic<SF_PointNormal>> _paramList;

private:
  QList<SF_ParamQSM<SF_PointNormal>> paramList();
  CT_AutoRenameModels m_outCloudItem;
  CT_AutoRenameModels _outCylinderGroup;
  CT_AutoRenameModels _outCylinders;
  CT_AutoRenameModels _outSFQSM;
  CT_AutoRenameModels _outParams;
  int toStringSFMethod();
  SF_CLoudToModelDistanceMethod toStringCMDMethod();
  double _PP_voxelSize = 0.02;
  double _PP_euclideanClusteringDistance = 0.1;
  double _SF_OPT_euclideanClusteringDistance = 0.03;
  double _SF_OPT_sphereRadiusMultiplier = 2;
  double _SF_OPT_sphereEpsilon = 0.035;
  bool _SF_parameterAutoSearch = true;
  int _SF_RANSACIiterations = 100;
  double _SF_minRadiusGlobal = 0.04;
  double _SF_inlierDistance = 0.03;
  int _SF_minPtsGeometry = 5;
  double _SF_heightInitializationSlice = 0.05;
  int _CMD_fittingMethod = toStringCMDMethod();
  int _CMD_k = 9;
  int _CMD_numClstrs = 1;
  double _CMD_inlierDistance = 0.15;

  QString _PARAMETERS_1 = "100%";
  QString _PARAMETERS_3 = "75%; 100%; 150%";
  QString _PARAMETERS_5 = "75%; 100%; 150%; 200%; 250%";
  QString _PARAMETERS_7 = "75%; 90%; 100%; 140%; 190%; 250%; 300%";
  QString _PARAMETERS_9 = "75%; 90%; 100%; 140%; 180%; 240%; 300%; 370; 450";
  QString _PARAMETERS_11 = "75%; 90%; 100%; 140%; 190%; 240%; 300%; 400%; 500%; 600%; 700%";
  QString _PARAMETERS_CHOICE_SPHERE_RADIUS_MULTIPLIER = _PARAMETERS_5;
  QString _PARAMETERS_CHOICE_SPHERE_EPSILON = _PARAMETERS_5;
  QString _PARAMETERS_CHOICE_EUCLIDEAN_CLUSTERING_DISTANCE = _PARAMETERS_5;
  QStringList _PARAMETERS_LIST_SPHERE_RADIUS_MULTIPLIER;
  QStringList _PARAMETERS_LIST_SPHERE_EPSILON;
  QStringList _PARAMETERS_LIST_EUCLIDEAN_CLUSTERING_DISTANCE;

  std::vector<double> paramsStringToNumber(const QString& UISelection);

  void configDialogAddSphereFollowingHyperParameters(CT_StepConfigurableDialog* configDialog);
  void configDialogAddSphereFollowingOptimizableParameters(CT_StepConfigurableDialog* configDialog);
  void configDialogGuruAddPreProcessing(CT_StepConfigurableDialog* configDialog);
  void configDialogGuruAddGridSearchCloudToModelDistance(CT_StepConfigurableDialog* configDialog);

  virtual void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
  virtual void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
};

#endif // SF_STEP_SPHEREFOLLOWING_BASIC_H
