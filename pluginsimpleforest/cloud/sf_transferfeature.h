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
    pcl::PointCloud<PointType>::Ptr m_src;
    pcl::PointCloud<PointType>::Ptr m_tar;
public:
    SF_TransferFeature();
    void setInputClouds(pcl::PointCloud<PointType>::Ptr src, pcl::PointCloud<PointType>::Ptr tar);
    void compute();
};

template<typename PointType>
void SF_TransferFeature::setInputClouds(pcl::PointCloud::Ptr src, pcl::PointCloud::Ptr tar)
{
    m_src = src;
    m_tar = tar;
}

template<typename PointType>
void SF_TransferFeature::compute()
{
    typename pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(m_src);
    for(size_t i = 0; i < m_tar->points.size(); i++)
    {
        PointType tarPoint = m_tar->points[i];
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if ( tree->nearestKSearch (tarPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            PointType srcPoint = m_src->points[pointIdxNKNSearch[0]];
            srcPoint.x = tarPoint.x;
            srcPoint.y = tarPoint.y;
            srcPoint.z = tarPoint.z;
            m_tar->points[i] = srcPoint;
        }
    }
}

#endif // SF_TRANSFERFEATURE_H
