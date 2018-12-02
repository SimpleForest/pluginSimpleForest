/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_filtervoxelclustering.hpp is part of SimpleForest - a plugin for the
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

#ifndef SF_FILTERVOXELCLUSTERING_HPP
#define SF_FILTERVOXELCLUSTERING_HPP

#include "sf_filtervoxelclustering.h"
#include <ct_itemdrawable/ct_grid3d_sparse.h>
#include <utility>

template<typename PointType>
SF_VoxelClustering<PointType>::SF_VoxelClustering()
{

}

template<typename PointType>
void SF_VoxelClustering<PointType>::compute()
{
    initialize();
    std::unique_ptr<CT_Grid3D_Sparse<int> > clusterIndices (CT_Grid3D_Sparse<int>::createGrid3DFromXYZCoords(
                                                                     NULL,
                                                                     NULL,
                                                                     m_min.x,
                                                                     m_min.y,
                                                                     m_min.z,
                                                                     m_max.x,
                                                                     m_max.y,
                                                                     m_max.z,
                                                                     m_param.m_voxelSize,
                                                                     -2,
                                                                     -1));
    int numberInitializedClouds = 0;
    for(size_t i = 0; i < m_param.m_cloud.first->points.size(); i++)
    {
        PointType point = m_param.m_cloud.first->points[i];
        size_t CTIndex = m_param.m_cloud.second[i];
        size_t index;
        clusterIndices->indexAtXYZ(point.x,
                                   point.y,
                                   point.z,
                                   index);
        int clusterIndex = clusterIndices->valueAtIndex(index);
        if(clusterIndex <= -1) {
            typename pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);
            std::vector<size_t> CT_indices;
            SF_AbstractMultipleFilter<PointType>::m_clusterOut.push_back(std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t> >
                                                                         (cluster, CT_indices));
            clusterIndex = numberInitializedClouds++;
            clusterIndices->setValueAtIndex(index,
                                            clusterIndex);
        }
        SF_AbstractMultipleFilter<PointType>::m_clusterOut[clusterIndex].first->points.push_back(std::move(point));
        SF_AbstractMultipleFilter<PointType>::m_clusterOut[clusterIndex].second.push_back(std::move(CTIndex));
    }
    m_param.m_clustersOut = SF_AbstractMultipleFilter<PointType>::m_clusterOut;
}


template<typename PointType>
void SF_VoxelClustering<PointType>::initialize()
{
     pcl::getMinMax3D (*m_param.m_cloud.first, m_min, m_max);
}

template<typename PointType>
void SF_VoxelClustering<PointType>::setParam(const SF_ParameterSetVoxelization<PointType> &param)
{
    m_param = param;
}

#endif // SF_FILTERVOXELCLUSTERING_HPP
