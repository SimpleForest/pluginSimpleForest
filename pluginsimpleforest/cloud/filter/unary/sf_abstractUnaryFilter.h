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

#ifndef SF_ABSTRACTUNITARY_H
#define SF_ABSTRACTUNITARY_H

#include "cloud/filter/sf_abstractfilter.h"

/**
 * @brief The SF_AbstractMultipleFilter class Abstract class for manipulating
 * a templated PCL cloud producing n clusters
 */
template<typename PointType>
class SF_AbstractUnaryFilter : public SF_AbstractFilterI<PointType>
{
public:
  /**
   * @brief Standard constructor.
   */
  SF_AbstractUnaryFilter();

  typename pcl::PointCloud<PointType>::Ptr cloudOut() const;

protected:
  typename pcl::PointCloud<PointType>::Ptr m_cloudOut;
};

#endif // SF_FILTERMAXINTENSITY_H

template<typename PointType>
SF_AbstractUnaryFilter<PointType>::SF_AbstractUnaryFilter()
{}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr
SF_AbstractUnaryFilter<PointType>::cloudOut() const
{
  return m_cloudOut;
}
