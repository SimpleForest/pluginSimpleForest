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

#ifndef SF_SPHEREFOLLOWINGADVANCED_H
#define SF_SPHEREFOLLOWINGADVANCED_H

#define DEF_IN_ID "iID"
#define DEF_IN_PARAMS "iParams"

#include "steps/segmentation/sf_AbstractStepSegmentation.h"

class SF_StepSphereFollowingAdvanced : public SF_AbstractStepSegmentation
{
  Q_OBJECT

public:
  SF_StepSphereFollowingAdvanced(CT_StepInitializeData& dataInit);
  ~SF_StepSphereFollowingAdvanced();
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
  QList<SF_ParamSpherefollowingAdvanced<SF_PointNormal>> _paramList;

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
  double _NM_minSize = 0.5;
  int _NM_iterations = 100;
  int _SF_RANSACIiterations = 100;
  double _SF_minRadiusGlobal = 0.04;
  double _SF_inlierDistance = 0.03;
  int _SF_minPtsGeometry = 5;
  double _SF_heightInitializationSlice = 0.05;
  int _CMD_fittingMethod = toStringCMDMethod();
  int _CMD_k = 9;
  int _CMD_numClstrs = 1;
  double _CMD_inlierDistance = 0.15;

  std::vector<double> paramsStringToNumber(const QString& UISelection);

  void configDialogAddSphereFollowingNelderMead(CT_StepConfigurableDialog* configDialog);
  void configDialogGuruAddPreProcessing(CT_StepConfigurableDialog* configDialog);
  void configDialogGuruAddGridSearchCloudToModelDistance(CT_StepConfigurableDialog* configDialog);

  virtual void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
  virtual void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
};

#endif // SF_SPHEREFOLLOWINGADVANCED_H
