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

#include "sf_visualizefitquality.h"

void
SF_VisualizeFitquality::setParams(const SF_CloudToModelDistanceParameters& params)
{
  m_params = params;
}

void
SF_VisualizeFitquality::setCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
{
  m_cloud = cloud;
}

void
SF_VisualizeFitquality::setQsm(std::shared_ptr<SF_ModelQSM> qsm)
{
  _qsm = qsm;
}

CT_ColorCloudStdVector*
SF_VisualizeFitquality::colors() const
{
  return m_colors;
}

SF_VisualizeFitquality::SF_VisualizeFitquality() {}

void
SF_VisualizeFitquality::compute()
{
  m_colors = new CT_ColorCloudStdVector(m_cloud->points.size());
  m_params._method = SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDERMSAC;
  Sf_CloudToModelDistance<pcl::PointXYZINormal> cmd(_qsm, m_cloud, m_params);
  std::vector<float> distances = cmd.distances();
  float _maxDistance = m_params._inlierDistance;

  for (size_t i = 0; i < m_cloud->points.size(); i++) {
    CT_Color& col = m_colors->colorAt(i);
    float distance = distances[i];
    if (distance >= _maxDistance) {
      col.r() = (0);
      col.g() = (0);
      col.b() = (255);
    } else {
      float perc = distance / _maxDistance;
      col.r() = (std::abs(255 * perc));
      col.g() = (std::abs(255 - 255 * perc));
      col.b() = (0);
    }
  }
}
