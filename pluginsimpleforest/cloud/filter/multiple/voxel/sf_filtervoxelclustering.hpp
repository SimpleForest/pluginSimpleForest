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

template<typename PointType>
SF_VoxelClustering::SF_VoxelClustering(const SF_ParameterSetVoxelization &param):
    SF_AbstractMultipleFilter(param.m_cloud), m_param(param)
{
    initialize();
}

template<typename PointType>
void SF_VoxelClustering<PointType>::compute()
{
    std::unique_ptr<CT_Grid3D_Sparse<int> > clusterIndices (CT_Grid3D_Sparse<int>::createGrid3DFromXYZCoords(
                                                                     NULL,
                                                                     NULL,
                                                                     m_min[0],
                                                                     m_min[1],
                                                                     m_min[2],
                                                                     m_max[0],
                                                                     m_max[1],
                                                                     m_max[2],
                                                                     m_param.m_voxelSize,
                                                                     -2,
                                                                     -1));
    std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, std::vector<size_t> > > clusters;
    int numberInitializedClouds = 0;
    for(size_t i = 0; i < m_param.m_cloud.first->points().size(); i++)
    {
        PointType point = m_param.m_cloud.first->points()[i];
        size_t CTIndex = m_param.m_cloud.second[i];
        size_t index;
        clusterIndices->indexAtXYZ(point.x,
                                   point.y,
                                   point.z,
                                   index);
        int clusterIndex = clusterIndices->valueAtIndex(index);
        if(clusterIndex <= -1) {
            pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);
            std::vector<size_t> CT_indices;
            clusters.push_back(std::pair<pcl::PointCloud<PointType>::Ptr, std::vector<size_t> > (cluster,
                                                                                                 CT_indices));
            clusterIndex = numberInitializedClouds++;
            clusterIndices->setValueAtIndex(index,
                                            clusterIndex);
        }
        clusters[clusterIndex].first->points.push_back(point);
        clusters[clusterIndex].second.push_back(CTIndex);
    }
}


template<typename PointType>
void SF_VoxelClustering<PointType>::initialize()
{
     pcl::getMinMax3D (*m_param.cloud.first, m_min, m_max);
}



#endif // SF_FILTERVOXELCLUSTERING_HPP
