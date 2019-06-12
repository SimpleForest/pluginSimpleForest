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

#ifndef SF_STEPQSMMEDIANFILTER_H
#define SF_STEPQSMMEDIANFILTER_H

#include "steps/qsm/sf_abstractStepQSM.h"

#include <QString>

class SF_StepQSMMedianFilter : public SF_AbstractStepQSM
{
  Q_OBJECT

public:
  SF_StepQSMMedianFilter(CT_StepInitializeData& dataInit);
  ~SF_StepQSMMedianFilter();
  QString getStepDescription() const;
  QString getStepDetailledDescription() const;
  QString getStepURL() const;
  CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData& dataInit);
  QStringList getStepRISCitations() const;

protected:
  void createInResultModelListProtected();
  void createOutResultModelListProtected();
  void adaptParametersToExpertLevel() {}
  virtual void createPreConfigurationDialog() {}
  virtual void createPostConfigurationDialog();
  void compute();
  QList<SF_ParamQSMedian<SF_PointNormal>> _paramList;

private:
  CT_AutoRenameModels m_outCloudItem;
  CT_AutoRenameModels _outCylinderGroup;
  CT_AutoRenameModels _outCylinders;
  CT_AutoRenameModels _outSFQSM;

  double m_percentage = 0.2;

  virtual void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
  virtual void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
};

#endif // SF_STEPQSMMEDIANFILTER_H
