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

#ifndef SF_STEPEXPORTQSMLIST_H
#define SF_STEPEXPORTQSMLIST_H

#include "steps/qsm/sf_abstractStepQSM.h"

#include <converters/CT_To_PCL/sf_converterCTIDToPCLCloud.h>
#include <converters/CT_To_PCL/sf_converterCTToPCL.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>

class SF_StepExportQSMList : public SF_AbstractStepQSM
{
  Q_OBJECT

public:
  SF_StepExportQSMList(CT_StepInitializeData& dataInit);
  ~SF_StepExportQSMList();
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

private:
  int toStringSFMethod();
  SF_CLoudToModelDistanceMethod toStringCMDMethod();

  bool m_writePlyGrowthVolume = true;
  bool m_writePlyGrowthLengthPly = true;
  bool m_writePlyStem = true;
  bool m_writeCloudRadius = true;
  bool m_writeCloudGrowthLength = true;
  bool m_writeCloudFitQuality = true;
  bool m_writeCloud = true;
  bool m_downScaleCloud = true;
  double m_voxelSize = 0.03;
  QStringList m_filePath;

  virtual void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
  virtual void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
};

#endif // SF_STEPEXPORTQSMLIST_H
