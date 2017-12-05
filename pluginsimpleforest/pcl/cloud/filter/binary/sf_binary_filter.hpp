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
#include"sf_binary_filter.h"

template <typename PointType>
Sf_Binary_Filter<PointType>::Sf_Binary_Filter(typename  pcl::PointCloud<PointType>::Ptr cloud_in): SF_Abstract_Filter<PointType>(cloud_in)
{

    reset();
}

template<typename PointType>
void Sf_Binary_Filter<PointType>::create_index(PointType point,
                                               float sqrd_distance)
{
    if(Sf_Binary_Filter<PointType>::equals_by_sqrt_distance(sqrd_distance))
    {_indices.push_back(0);}
    else
    {_indices.push_back(1);
        _cloud_out_filtered_noise->points.push_back(point);}
}

template<typename PointType>
void Sf_Binary_Filter<PointType>::reset()
{
    _cloud_out_filtered.reset(new pcl::PointCloud<PointType>);
    _cloud_out_filtered_noise.reset(new pcl::PointCloud<PointType>);
    _indices.clear();
}


template<typename PointType>
void Sf_Binary_Filter<PointType>::search_kd_tree(size_t index,
                                                 pcl::KdTreeFLANN<PointType> & kdtree)
{
    PointType point = Sf_Binary_Filter<PointType>::_cloud_in->at(index);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    kdtree.nearestKSearch (point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    create_index(point, pointNKNSquaredDistance[0]);
}

template<typename PointType>
void Sf_Binary_Filter<PointType>::iterate_over_cloud(pcl::KdTreeFLANN<PointType> &kdtree)
{
    size_t size = Sf_Binary_Filter<PointType>::_cloud_in->size();
    for(size_t i = 0; i < size; i++)
    {
        search_kd_tree(i,kdtree);
    }
}

template <typename PointType>
void Sf_Binary_Filter<PointType>::create_indices()
{
    if(_cloud_out_filtered->points.size()>0)
    {
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud (_cloud_out_filtered);
    iterate_over_cloud(kdtree);
    }
    else
    {
        size_t size = Sf_Binary_Filter<PointType>::_cloud_in->size();
        for(size_t i = 0; i < size; i++)
        {
            _indices.push_back(1);
        }
    }
}


template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr Sf_Binary_Filter<PointType>::get_cloud_out_filtered_noise() const
{
    return _cloud_out_filtered_noise;
}


template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr Sf_Binary_Filter<PointType>::get_cloud_out_filtered() const
{
    return _cloud_out_filtered;
}

template <typename PointType>
std::vector<int> Sf_Binary_Filter<PointType>::get_indices() const
{
   return _indices;
}




#endif // SF_BINARY_FILTER_HPP
