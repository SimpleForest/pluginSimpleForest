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

#ifndef SF_EXPORTCLOUD_H
#define SF_EXPORTCLOUD_H

#include "file/export/sf_abstractExport.h"
#include "pcl/sf_math.h"
#include "pcl/sf_point.h"
#include "qsm/algorithm/distance/sf_cloudToModelDistance.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <random>

enum class SF_ExportCloudPolicy
{
  FIT_QUALITY = 0,
  RADIUS = 1,
  GROWTHLENGTH = 2
};

class SF_ExportCloud : public SF_AbstractExport
{
  SF_CloudNormal::Ptr m_cloud;
  SF_ExportCloudPolicy m_exportPolicy = SF_ExportCloudPolicy::FIT_QUALITY;
  std::vector<double> m_intensities;
  void getMinMax();
  std::vector<double> getLogarithm(std::vector<double>& vector);
  QString getFullPath(QString path);

public:
  SF_ExportCloud();
  void exportCloud(QString path,
                   QString cloudName,
                   std::shared_ptr<SF_ModelQSM> qsm,
                   SF_CloudNormal::Ptr cloud,
                   SF_ExportCloudPolicy exportPolicy);
};

#endif // SF_EXPORTCLOUD_H
