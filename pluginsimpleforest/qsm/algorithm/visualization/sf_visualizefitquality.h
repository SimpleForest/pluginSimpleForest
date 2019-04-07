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

#ifndef SF_VISUALIZEFITQUALITY_H
#define SF_VISUALIZEFITQUALITY_H

#include "qsm/algorithm/distance/sf_cloudToModelDistance.h"
#include "steps/param/sf_paramAllSteps.h"

class SF_VisualizeFitquality
{
  SF_CloudToModelDistanceParameters m_params;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr m_cloud;
  std::shared_ptr<SF_ModelQSM> _qsm;
  CT_ColorCloudStdVector* m_colors;

public:
  SF_VisualizeFitquality();
  void setParams(const SF_CloudToModelDistanceParameters& params);
  void setCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);
  void setQsm(std::shared_ptr<SF_ModelQSM> qsm);
  void compute();
  CT_ColorCloudStdVector* colors() const;
};

#endif // SF_VISUALIZEFITQUALITY_H
