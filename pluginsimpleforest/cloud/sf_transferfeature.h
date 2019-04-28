/****************************************************************************

    Copyright (C) 2017-2019 , Dr. Jan Hackenberg

    sf_principaldirection.h is part of SimpleForest - a plugin for the
    Computree platform.

    SimpleForest is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SimpleForest is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with
    SimpleForest. If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#ifndef SF_TRANSFERFEATURE_H
#define SF_TRANSFERFEATURE_H

#include "cloud/sf_abstractcloud.h"

#include <pcl/features/normal_3d.h>

template<typename PointType>
class SF_TransferFeature
{
  typename pcl::PointCloud<PointType>::Ptr m_tar;
  typename pcl::PointCloud<PointType>::Ptr m_src;

public:
  void setInputClouds(typename pcl::PointCloud<PointType>::Ptr src, typename pcl::PointCloud<PointType>::Ptr tar);
  void compute();
};

template<typename PointType>
void
SF_TransferFeature<PointType>::setInputClouds(typename pcl::PointCloud<PointType>::Ptr src,
                                              typename pcl::PointCloud<PointType>::Ptr tar)
{
  m_tar = tar;
  m_src = src;
}

template<typename PointType>
void
SF_TransferFeature<PointType>::compute()
{
  typename pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  tree->setInputCloud(m_src);
  for (size_t i = 0; i < m_tar->points.size(); i++) {
    PointType& tarPoint = m_tar->points[i];
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    if (tree->nearestKSearch(tarPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      PointType newTarPoint = m_src->points[pointIdxNKNSearch[0]];
      newTarPoint.getVector3fMap() = tarPoint.getVector3fMap();
      tarPoint = std::move(newTarPoint);
    }
  }
}

#endif // SF_TRANSFERFEATURE_H
