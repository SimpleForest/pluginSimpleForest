/****************************************************************************

 Copyright (C) 2017-2018 Jan Hackenberg, free software developer
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

#ifndef SF_DESCRIPTOR_HPP
#define SF_DESCRIPTOR_HPP

#include "sf_descriptor.h"

#include "pcl/sf_math.h"

template <typename PointType>
SF_Descriptor<PointType>::SF_Descriptor(
    typename pcl::PointCloud<PointType>::Ptr cloudIn)
    : _cloudIn(cloudIn) {}

template <typename PointType> float SF_Descriptor<PointType>::mean() const {
  return m_mean;
}

template <typename PointType> float SF_Descriptor<PointType>::sd() const {
  return m_sd;
}

template <typename PointType>
void SF_Descriptor<PointType>::setParameters(int k) {
  m_k = k;
}

template <typename PointType> void SF_Descriptor<PointType>::computeFeatures() {
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(_cloudIn);
  std::vector<float> distances;
  std::for_each(
      _cloudIn->points.begin(), _cloudIn->points.end(),
      [this, &kdtree, &distances](const PointType &point) {
        std::vector<int> indices;
        std::vector<float> squaredDistances;
        std::vector<float> closestDistances;
        if (kdtree.nearestKSearch(point, m_k, indices, squaredDistances) > 0) {
          size_t squaredDistancesIndex = 0;
          std::for_each(indices.begin(), indices.end(),
                        [this, &squaredDistances, &closestDistances,
                         &squaredDistancesIndex](int index) {
                          if (squaredDistances[squaredDistancesIndex] != 0) {
                            closestDistances.push_back(std::sqrt(
                                squaredDistances[squaredDistancesIndex]));
                          }
                          squaredDistancesIndex++;
                        });
          distances.push_back(SF_Math<float>::getMean(closestDistances));
        };
      });
  m_mean = SF_Math<float>::getMean(distances);
  m_sd = SF_Math<float>::getStandardDeviation(distances, m_mean);
}

#endif // SF_DESCRIPTOR_HPP
