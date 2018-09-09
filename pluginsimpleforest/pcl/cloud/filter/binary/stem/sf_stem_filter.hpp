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
#ifndef SF_STEM_FILTER_HPP
#define SF_STEM_FILTER_HPP

#include "pcl/cloud/filter/binary/stem/sf_stem_filter.h"
#include "pcl/cloud/feature/growth_direction/sf_growth_direction.h"

template<typename PointType>
void SF_Stem_Filter<PointType>::transfer_stem(const SF_Param_Stem_Filter<PointType> &params, typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud,
                                              typename pcl::PointCloud<PointType>::Ptr cloud_with_growth_direction) {
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud (cloud_with_growth_direction);
    for(size_t i = 0; i <  SF_Stem_Filter<PointType>::_cloud_in->points.size(); i++) {
        PointType p =  SF_Stem_Filter<PointType>::_cloud_in->points.at(i);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( kdtree.nearestKSearch ( p, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
            PointType gd_point = cloud_with_growth_direction->points[pointIdxRadiusSearch[0]];
            Eigen::Vector3f axis1;
            axis1[0] = params._x;
            axis1[1] = params._y;
            axis1[2] = params._z;
            Eigen::Vector3f axis2;
            axis2[0] = gd_point.normal_x;
            axis2[1] = gd_point.normal_y;
            axis2[2] = gd_point.normal_z;
            double deg = SF_Math<double>::getAngleBetweenDeg(axis1,axis2);
            if(deg < params._angle || deg > (180-params._angle)) {
                SF_Stem_Filter<PointType>::_cloud_out_filtered->points.push_back(p);
            } else {
                SF_Stem_Filter<PointType>::_cloud_out_filtered_noise->points.push_back(p);
            }
        }
    }
}

template<typename PointType>
void SF_Stem_Filter<PointType>::set_params(SF_Param_Stem_Filter<PointType> &params) {
    _params = params;
}

template<typename PointType>
void SF_Stem_Filter<PointType>::compute() {
    SF_Stem_Filter<PointType>::_cloud_out_filtered_noise.reset(new typename pcl::PointCloud<PointType>);
    SF_Stem_Filter<PointType>::_cloud_out_filtered.reset(new typename pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr down_scaled_cloud = down_scale(_params._voxel_size);
    pcl::PointCloud<PointType>::Ptr cloud_with_growth_direction(new SF_Cloud_Normal());
    SF_Growth_Direction<PointType, PointType> gd(down_scaled_cloud, cloud_with_growth_direction);
    gd.set_parameters(_params._radius_normal,_params._radius_growth_direction);
    gd.compute_features();
    transfer_stem(_params,SF_Abstract_Cloud<PointType>::_cloud_in,cloud_with_growth_direction);
    SF_Stem_Filter<PointType>::create_indices();
}

template<typename PointType>
SF_Stem_Filter<PointType>::SF_Stem_Filter() {
    Sf_Binary_Filter<PointType>::reset();
}

#endif // SF_STEM_FILTER_HPP
