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
#ifndef SF_ABRACT_CLOUD_H
#define SF_ABRACT_CLOUD_H

#include <pcl/sf_point.h>
#include <steps/param/sf_abstract_param.h>

template <typename PointType>
class SF_Abstract_Cloud {
protected:
    std::vector<int> _indices;
    const float _MIN_DISTANCE = 0.0001;
    const float _MIN_SQUARED_DISTANCE = _MIN_DISTANCE*_MIN_DISTANCE;

    bool equals_by_sqrt_distance(float sqrt_distance);
    typename pcl::PointCloud<PointType>::Ptr _cloud_in;
    virtual void reset() = 0;
    void search_kd_tree(size_t index, typename
                        pcl::KdTreeFLANN<PointType> &kdtree);
    void iterate_over_cloud(pcl::KdTreeFLANN<PointType> &kdtree);
    typename pcl::PointCloud<PointType>::Ptr down_scale(float voxel_size);
    void extract_neighbors_by_range(typename pcl::KdTree<PointType>::Ptr kdtree, PointType p, typename pcl::PointCloud<PointType>::Ptr neighborhood, float range);
    void extract_neighbors_by_knn(typename pcl::KdTree<PointType>::Ptr kdtree, PointType p, typename pcl::PointCloud<PointType>::Ptr neighborhood, int k);
    void extract_neighborhood_by_index_list(std::vector<int> pointIndex, typename pcl::PointCloud<PointType>::Ptr neighborhood);


    virtual void create_indices() = 0;
    virtual void create_index(PointType point,
                         float sqrd_distance) = 0;

public:
    SF_Abstract_Cloud();
    SF_Abstract_Cloud(const typename pcl::PointCloud<PointType>::Ptr cloud_in);
    std::vector<int> get_indices() const;
    void set_cloud_in(const typename pcl::PointCloud<PointType>::Ptr &cloud_in);
};

#include "sf_abstract_cloud.hpp"

#endif // SF_ABRACT_CLOUD_H

