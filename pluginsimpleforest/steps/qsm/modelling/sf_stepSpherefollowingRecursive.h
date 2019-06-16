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

#ifndef SF_STEPSPHEREFOLLOWINGRECURSIVE_H
#define SF_STEPSPHEREFOLLOWINGRECURSIVE_H

#include "steps/qsm/modelling/sf_stepSpherefollowingBasic.h"

class SF_StepSphereFollowingRecursive : public SF_StepSpherefollowingBasic
{
  Q_OBJECT
public:
  SF_StepSphereFollowingRecursive(CT_StepInitializeData& dataInit);
  ~SF_StepSphereFollowingRecursive();
  QString getStepDescription() const;
  QString getStepDetailledDescription() const;
  QString getStepURL() const;
  CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData& dataInit);
  QStringList getStepRISCitations() const;

protected:
  void createInResultModelListProtected();
  void createOutResultModelListProtected();
  void createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog);
  void createParamList(CT_ResultGroup* out_result);
  virtual void createPreConfigurationDialog();
  virtual void createPostConfigurationDialog();

  void configRecursion(CT_StepConfigurableDialog* configDialog);

  void compute();
  QList<SF_ParamSpherefollowingRecursive<SF_PointNormal>> _paramList;

  double m_clusteringDistance = 0.03;
  double m_unfittedDistance = 0.2;
  double m_minPercentage = 0.003;

  QList<SF_ParamQSM<SF_PointNormal>> paramList();
  QString _PARAMETERS_CHOICE_SPHERE_RADIUS_MULTIPLIER = _PARAMETERS_5;
  QString _PARAMETERS_CHOICE_SPHERE_EPSILON = _PARAMETERS_5;
  QString _PARAMETERS_CHOICE_EUCLIDEAN_CLUSTERING_DISTANCE = _PARAMETERS_5;

  virtual void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
  virtual void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
};

#endif // SF_STEPSPHEREFOLLOWINGRECURSIVE_H
