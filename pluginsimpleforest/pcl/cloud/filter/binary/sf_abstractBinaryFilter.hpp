/****************************************************************************

 Copyright (C) 2017-2017 Jan Hackenberg, free software developer
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
#ifndef SF_BINARY_FILTER_HPP
#define SF_BINARY_FILTER_HPP
#include "sf_abstractBinaryFilter.h"

template<typename PointType>
Sf_AbstractBinaryFilter<PointType>::Sf_AbstractBinaryFilter()
{
  reset();
}

template<typename PointType>
void
Sf_AbstractBinaryFilter<PointType>::reset()
{
  SF_AbstractFilterDeprecated<PointType>::reset();
  _cloudOutFilteredNoise.reset(new pcl::PointCloud<PointType>);
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr
Sf_AbstractBinaryFilter<PointType>::getCloudOutFilteredNoise() const
{
  return _cloudOutFilteredNoise;
}

template<typename PointType>
double
Sf_AbstractBinaryFilter<PointType>::getPercentage()
{
  double in = SF_AbstractCloud<PointType>::_cloudIn->points.size();
  double out = SF_AbstractFilterDeprecated<PointType>::_cloudOutFiltered->points.size() * 100.0;
  if (in == 0) {
    return 0;
  }
  return (out / in);
}

template<typename PointType>
void
Sf_AbstractBinaryFilter<PointType>::createIndex(PointType point, float sqrd_distance)
{
  if (Sf_AbstractBinaryFilter<PointType>::equalsBySqrtDistance(sqrd_distance)) {
    SF_AbstractCloud<PointType>::_indices.push_back(0);
  } else {
    SF_AbstractCloud<PointType>::_indices.push_back(1);
    _cloudOutFilteredNoise->points.push_back(point);
  }
}

#endif // SF_BINARY_FILTER_HPP
