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

#ifndef SF_QSMREFITCYLINDERS_H
#define SF_QSMREFITCYLINDERS_H

#include "steps/qsm/sf_abstractStepQSM.h"

class SF_StepQSMRefitCylinders : public SF_AbstractStepQSM
{
  Q_OBJECT

public:
  SF_StepQSMRefitCylinders(CT_StepInitializeData& dataInit);
  ~SF_StepQSMRefitCylinders();
  QString getStepDescription() const;
  QString getStepDetailledDescription() const;
  QString getStepURL() const;
  CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData& dataInit);
  QStringList getStepRISCitations() const;

protected:
  void createInResultModelListProtected();
  void createOutResultModelListProtected();
  void adaptParametersToExpertLevel() {}
  void createParamList(CT_ResultGroup* outResult);
  virtual void createPreConfigurationDialog() {}
  virtual void createPostConfigurationDialog();
  void compute();
  QList<SF_ParamRefitCylinders> _paramList;

private:
  CT_AutoRenameModels m_outCloudItem;
  CT_AutoRenameModels _outCylinderGroup;
  CT_AutoRenameModels _outCylinders;
  CT_AutoRenameModels _outSFQSM;
  int m_minPts = 10;
  double m_inlierDistance = 0.03;
  double m_range = 0.20;
  int m_ransacIterations = 100;
  double m_angle = 30;
  double _PP_voxelSize = 0.02;
  int toStringSFMethod();
  SF_CLoudToModelDistanceMethod toStringCMDMethod();

  virtual void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
  virtual void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
};

#endif // SF_QSMREFITCYLINDERS_H
