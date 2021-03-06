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

#ifndef SF_QSMALLOMETRICCHECK_H
#define SF_QSMALLOMETRICCHECK_H

#include "steps/qsm/sf_abstractStepQSM.h"

#include <QString>

class SF_StepQSMAllometricCorrection : public SF_AbstractStepQSM
{
  Q_OBJECT

public:
  SF_StepQSMAllometricCorrection(CT_StepInitializeData& dataInit);
  ~SF_StepQSMAllometricCorrection();
  QString getStepDescription() const;
  QString getStepDetailledDescription() const;
  QString getStepURL() const;
  CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData& dataInit);
  QStringList getStepRISCitations() const;

protected:
  void createInResultModelListProtected();
  void createOutResultModelListProtected();
  void adaptParametersToExpertLevel() {}
  void createParamList(CT_ResultGroup* out_result);
  virtual void createPreConfigurationDialog() {}
  virtual void createPostConfigurationDialog();
  void compute();
  QList<SF_ParamAllometricCorrectionNeighboring> _paramList;
  void createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog);

private:
  CT_AutoRenameModels m_outCloudItem;
  CT_AutoRenameModels _outCylinderGroup;
  CT_AutoRenameModels _outCylinders;
  CT_AutoRenameModels _outSFQSM;
  int toStringSFMethod();
  SF_CLoudToModelDistanceMethod toStringCMDMethod();

  double _range = 0.2;
  double _minRadius = 0.0025;

  double m_power = 1.0 / 2.49;
  bool m_useGrowthLength = true;
  bool m_withIntercept = false;
  double m_quantile = 0.5;
  int m_minPts = 5;
  double m_inlierDistance = 0.02;
  int m_ransacIterations = 1000;
  int m_gaussNewtonIterations = 20;
  bool m_estimateParams = true;

  virtual void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
  virtual void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
};

#endif // SF_QSMALLOMETRICCHECK_H
