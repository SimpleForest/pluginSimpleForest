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
#ifndef SF_RADIUS_OUTLIER_FILTER_HPP
#define SF_RADIUS_OUTLIER_FILTER_HPP
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <QDebug>
#include "pcl/cloud/filter/binary/radiusoutlier/sf_radius_outlier_filter.h"
#include "converters/CT_To_PCL/sf_converterCTToPCL.h"

template <typename PointType>
SF_Radius_Outlier_Filter<PointType>::SF_Radius_Outlier_Filter() {
    Sf_Binary_Filter<PointType>::reset();
}

template <typename PointType>
void  SF_Radius_Outlier_Filter<PointType>::compute(const SF_Param_Radius_Outlier_Filter<PointType> &params) {
    if(SF_AbstractCloud<PointType>::_cloudIn->points.size() >= 1) {
        SF_Radius_Outlier_Filter<PointType>::radius_outlier_filter(params);
    } else {
        qDebug() << "SF_Radius_Outlier_Filter<PointType>::compute - not enough points.";
    }
    SF_Radius_Outlier_Filter<PointType>::createIndices();
}

template <typename PointType>
void SF_Radius_Outlier_Filter<PointType>::radius_outlier_filter(typename SF_Param_Radius_Outlier_Filter<PointType> std_params) {
    if(std_params._radius>0.1f) {
        Sf_ConverterCTToPCL<PointType> converter;
        converter.setItemCpyCloudIn(std_params._itemCpyCloudIn);
        pcl::PointCloud<PointType>::Ptr downscaled_cloud (new pcl::PointCloud<PointType>);
        converter.downScale(std_params._radius/5,downscaled_cloud);
        pcl::PointCloud<PointType>::Ptr downscaled_cloud_neighbors_number  (new pcl::PointCloud<PointType>);
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud (downscaled_cloud);
        for(size_t i = 0; i < downscaled_cloud->points.size(); i++) {
            PointType p =  downscaled_cloud->points[i];
            float numberneighbors = 0;
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if ( kdtree.radiusSearch (p, std_params._radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
                for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
                    numberneighbors += downscaled_cloud->points[ pointIdxRadiusSearch[j] ].intensity;
            }
            p.intensity = numberneighbors;
            downscaled_cloud_neighbors_number->push_back(p);
        }
        pcl::KdTreeFLANN<PointType> kdtree_nn;
        kdtree_nn.setInputCloud (downscaled_cloud_neighbors_number);
        for(size_t i = 0; i < SF_AbstractCloud<PointType>::_cloudIn->points.size(); i++) {
            PointType p = SF_AbstractCloud<PointType>::_cloudIn->points[i];
            std::vector<int> pointIdxNNSearch;
            std::vector<float> pointNNSquaredDistance;
            if ( kdtree.nearestKSearch (p, 1, pointIdxNNSearch, pointNNSquaredDistance) > 0 ) {
                if(downscaled_cloud_neighbors_number->points[ pointIdxNNSearch[0] ].intensity>= std_params._min_Pts) {
                    SF_AbstractFilter<PointType>::_cloudOutFiltered->points.push_back(p);
                }
            }
        }
    } else {
        pcl::RadiusOutlierRemoval<PointType> outrem;
        outrem.setInputCloud (SF_AbstractCloud<PointType>::_cloudIn);
        outrem.setRadiusSearch(std_params._radius);
        outrem.setMinNeighborsInRadius( std_params._min_Pts);
        outrem.filter (*SF_AbstractFilter<PointType>::_cloudOutFiltered);
    }

}

#endif // SF_RADIUS_OUTLIER_FILTER_HPP
