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
#include <QDebug>
#include <pcl/cloud/filter/binary/radiusoutlier/sf_radius_outlier_filter.h>
template <typename PointType>
SF_Radius_Outlier_Filter<PointType>::SF_Radius_Outlier_Filter() {
    Sf_Binary_Filter<PointType>::reset();
}


template <typename PointType>
void  SF_Radius_Outlier_Filter<PointType>::compute(const SF_Param_Radius_Outlier_Filter<PointType> &params) {
    if(SF_Abstract_Cloud<PointType>::_cloud_in->points.size() >= 20) {
        SF_Radius_Outlier_Filter<PointType>::radius_outlier_filter(params);
    } else {
        qDebug() << "SF_Radius_Outlier_Filter<PointType>::compute - not enough points.";
    }
    SF_Radius_Outlier_Filter<PointType>::create_indices();
}

template <typename PointType>
void SF_Radius_Outlier_Filter<PointType>::radius_outlier_filter(SF_Param_Radius_Outlier_Filter<PointType> std_params) {
    pcl::RadiusOutlierRemoval<PointType> outrem;
    outrem.setInputCloud (SF_Abstract_Cloud<PointType>::_cloud_in);
    outrem.setRadiusSearch(std_params._radius);
    outrem.setMinNeighborsInRadius( std_params._min_Pts);
    outrem.filter (*SF_Abstract_Filter<PointType>::_cloud_out_filtered);
}

#endif // SF_RADIUS_OUTLIER_FILTER_HPP
