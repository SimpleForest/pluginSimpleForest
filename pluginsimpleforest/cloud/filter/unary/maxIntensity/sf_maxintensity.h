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

#ifndef SF_MAXINTENSITY_H
#define SF_MAXINTENSITY_H

#include "cloud/filter/unary/sf_abstractUnaryFilter.h"

/**
 * @brief The SF_AbstractMultipleFilter class Abstract class for manipulating
 * a templated PCL cloud producing n clusters
 */
template<typename PointType>
class SF_MaxIntensityFilter : public SF_AbstractUnaryFilter<PointType>
{
public:
  /**
   * @brief Standard constructor.
   */
  SF_MaxIntensityFilter();
  void setCloudIn(const typename pcl::PointCloud<PointType>::Ptr& cloudIn);
  void setMaxIntensity(float maxIntensity);
  void compute() override;

private:
  float m_maxIntensity = 0;
  typename pcl::PointCloud<PointType>::Ptr m_cloudIn;
};

template<typename PointType>
SF_MaxIntensityFilter<PointType>::SF_MaxIntensityFilter()
{}

template<typename PointType>
void
SF_MaxIntensityFilter<PointType>::setMaxIntensity(float maxIntensity)
{
  m_maxIntensity = maxIntensity;
}

template<typename PointType>
void
SF_MaxIntensityFilter<PointType>::compute()
{
  SF_AbstractUnaryFilter<PointType>::m_cloudOut.reset(new pcl::PointCloud<PointType>());
  for (size_t i = 0; i < m_cloudIn->points.size(); i++) {
    PointType& p = m_cloudIn->points[i];
    if (p.intensity <= m_maxIntensity) {
      SF_AbstractUnaryFilter<PointType>::m_cloudOut->points.push_back(p);
    }
  }
}

template<typename PointType>
void
SF_MaxIntensityFilter<PointType>::setCloudIn(const typename pcl::PointCloud<PointType>::Ptr& cloudIn)
{
  m_cloudIn = cloudIn;
}

#endif // SF_MAXINTENSITY_H
