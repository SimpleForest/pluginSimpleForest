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

#ifndef SF_EUCLIDEAN_CLUSTERING_HPP
#define SF_EUCLIDEAN_CLUSTERING_HPP

#include "sf_euclidean_clustering.h"
#include "converters/CT_To_PCL/sf_converter_ct_to_pcl.h"

#include <pcl/segmentation/extract_clusters.h>

template <typename PointType>
Sf_Euclidean_Clustering<PointType>::convertAndDownScale() {
    SF_Converter_CT_To_PCL<PointType> converterCloud;
    converterCloud.setItemCpyCloudIn(_params._itemCpyCloudIn);
    converterCloud.compute();
    SF_Abstract_Cloud<PointType>::_cloud_in = converterCloud.get_cloud_translated();
    Sf_Multiple_Filter<PointType>::_downScaledCloud = down_scale(_params._cellSize);
}

template <typename PointType>
Sf_Euclidean_Clustering<PointType>::extractClouds(std::vector<pcl::PointIndices> & clusterIndices) {
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it) {
      pcl::PointCloud<PointType>::Ptr cloudCluster (new pcl::PointCloud<PointType>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloudCluster->points.push_back (Sf_Multiple_Filter<PointType>::_downScaledCloud->points[*pit]);
      cloudCluster->width = cloudCluster->points.size ();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;
      Sf_Multiple_Filter<PointType>::_clouds.push_back(cloudCluster);
    }
}

template <typename PointType>
Sf_Euclidean_Clustering<PointType>::extractClusterIndices(std::vector<pcl::PointIndices> & clusterIndices) {
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (Sf_Multiple_Filter<PointType>::_downScaledCloud);
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (_params._euclideanDistance);
    ec.setMinClusterSize (_params._minSize);
    ec.setMaxClusterSize (std::numeric_limits<int>::max());
    ec.setSearchMethod (tree);
    ec.setInputCloud (Sf_Multiple_Filter<PointType>::_downScaledCloud);
    ec.extract (clusterIndices);
}

template <typename PointType>
void  Sf_Euclidean_Clustering<PointType>::compute(SF_Param_Euclidean_Clustering& params) {
    _params = params;
    convertAndDownScale();
    std::vector<pcl::PointIndices> clusterIndices;
    extractClusterIndices(clusterIndices);
    extractClouds(clusterIndices);
    create_indices();
}

#endif // SF_EUCLIDEAN_CLUSTERING_HPP
