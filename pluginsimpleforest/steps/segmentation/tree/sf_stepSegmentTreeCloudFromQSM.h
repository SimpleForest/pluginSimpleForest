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

#ifndef SF_SEGMENTTREECLOUDFROMQSM_H
#define SF_SEGMENTTREECLOUDFROMQSM_H

#include "steps/segmentation/sf_AbstractStepSegmentation.h"

class SF_StepSegmentTreeCloudFromQSM : public SF_AbstractStepSegmentation
{
  Q_OBJECT

public:
  SF_StepSegmentTreeCloudFromQSM(CT_StepInitializeData& dataInit);
  ~SF_StepSegmentTreeCloudFromQSM();
  QString getStepDescription() const;
  QString getStepDetailledDescription() const;
  QString getStepURL() const;
  CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData& dataInit);
  QStringList getStepRISCitations() const;

protected:
  void createInResultModelListProtected();
  void createPostConfigurationDialog();
  void createOutResultModelListProtected();
  void createPreConfigurationDialog() {}
  void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog) { configDialog->addEmpty(); }
  void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog) { configDialog->addEmpty(); }
  void adaptParametersToExpertLevel() {}
  void createParamList(CT_ResultGroup* outResult);
  void compute();

private:
  QList<SF_ParamSegmentTreeFromQSM<pcl::PointXYZINormal>> _paramList;
  int _numClstrs = 2;
  CT_AutoRenameModels m_outColorGrowthVolume;
  CT_AutoRenameModels m_outColorClusters;
  CT_AutoRenameModels m_clusterIndices;
  CT_AutoRenameModels m_outClusterID;
};

#endif // SF_SEGMENTTREECLOUDFROMQSM_H
