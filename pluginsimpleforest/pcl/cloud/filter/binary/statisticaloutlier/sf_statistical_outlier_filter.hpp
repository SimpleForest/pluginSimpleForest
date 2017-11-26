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
#ifndef SF_STATISTICAL_OUTLIER_FILTER_HPP
#define SF_STATISTICAL_OUTLIER_FILTER_HPP

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/cloud/filter/binary/statisticaloutlier/sf_statistical_outlier_filter.h>
template <typename PointType>
SF_Statistical_Outlier_Filter<PointType>::SF_Statistical_Outlier_Filter(typename pcl::PointCloud<PointType>::Ptr cloud_in): Sf_Binary_Filter<PointType>(cloud_in)
{
    SF_Statistical_Outlier_Filter<PointType>::reset();
}


template <typename PointType>
void  SF_Statistical_Outlier_Filter<PointType>::compute(const SF_Param_Statistical_Outlier_Filter &params)
{
    SF_Statistical_Outlier_Filter<PointType>::statistical_outlier_filter_iteratively(params);
    SF_Statistical_Outlier_Filter<PointType>::create_indices();
}

template <typename PointType>
void SF_Statistical_Outlier_Filter<PointType>::statistical_outlier_filter(SF_Param_Statistical_Outlier_Filter std_params,typename pcl::PointCloud<PointType>::Ptr cloud)
{
    SF_Statistical_Outlier_Filter<PointType>::_cloud_out_filtered.reset(new  pcl::PointCloud<PointType>());
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (std_params._k);
    sor.setStddevMulThresh (std_params._std_mult);
    sor.filter (*SF_Statistical_Outlier_Filter<PointType>::_cloud_out_filtered);
}

template <typename PointType>
void SF_Statistical_Outlier_Filter<PointType>::statistical_outlier_filter_iteratively(SF_Param_Statistical_Outlier_Filter std_params)
{
    typename pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>());
    cloud = SF_Statistical_Outlier_Filter<PointType>::_cloud_in;
    SF_Statistical_Outlier_Filter<PointType>::iterate(std_params, cloud);
}

template <typename PointType>
void SF_Statistical_Outlier_Filter<PointType>::iterate(SF_Param_Statistical_Outlier_Filter params, typename pcl::PointCloud<PointType>::Ptr cloud)
{
    int iterations = params._iterations;
    while(iterations > 0)
    {
        SF_Statistical_Outlier_Filter<PointType>::statistical_outlier_filter(params, cloud);
        cloud = SF_Statistical_Outlier_Filter<PointType>::_cloud_out_filtered;
        iterations--;
    }
}

template<typename PointType>
void SF_Statistical_Outlier_Filter<PointType>::reset()
{
    Sf_Binary_Filter<PointType>::reset();
}


#endif // SF_STATISTICAL_OUTLIER_FILTER_HPP
