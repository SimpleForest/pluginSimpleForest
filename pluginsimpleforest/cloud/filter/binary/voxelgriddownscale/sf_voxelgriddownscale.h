/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_voxelgriddownscale.h is part of SimpleForest - a plugin for the
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

#ifndef SF_VOXELGRIDDOWNSCALE_H
#define SF_VOXELGRIDDOWNSCALE_H

#include "cloud/filter/binary/sf_abstractbinaryfilter.h"
#include "cloud/filter/multiple/voxel/sf_filtervoxelclustering.h"

#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>

/**
 * @brief The SF_VoxelGridDownscale class. The for the input cloud
 * a grid wi    th a userchosen voxelsize is generated. Points contained
 * in a cell build in a first step a subcloud. The center of mass for
 * that cloud is build first. In a final step each closest neighbor
 * of the center of mass is put in the first cloud, all other points
 * are stored in a second cloud.
 */
template <typename PointType>
class SF_VoxelGridDownscale:
        public SF_AbstractBinaryFilter
{
public:
    /**
     * @brief Standard constructor.
     */
    SF_VoxelGridDownscale();
    /**
     * @brief compute Does the actual voxelgrid downscaling of \ref m_cloudIn.
     */
    void compute() override;
private:
    /**
     * @brief computeCentroids Computes the centroid for each cluster
     * @param clusters The input clusters
     * @return A cloud consisting of a centroid per cluster
     */
    pcl::PointCloud<PointType>::Ptr computeCentroids(const std::vector<std::pair<pcl::PointCloud<PointType>::Ptr,
                                                     std::vector<size_t> > > &clusters);
    /**
     * @brief downScale Downscales each cluster to the closest point to the centroid which is stored in
     * \ref m_clusterOut, the first member. Other points are stored in second cloud as second member in
     * \ref m_clusterOut.
     * @param clusters The input clusters
     * @param centroids The input centroids
     */
    void downScale(const std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, std::vector<size_t> > > &clusters,
                   pcl::PointCloud<PointType>::Ptr centroids);
};

#include "sf_voxelgriddownscale.hpp"

#endif // SF_VOXELGRIDDOWNSCALE_H


