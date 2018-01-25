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
#ifndef SF_RADIUS_OUTLIER_FILTER_H
#define SF_RADIUS_OUTLIER_FILTER_H

#include <pcl/cloud/filter/binary/sf_binary_filter.h>

template <typename PointType>
class SF_Radius_Outlier_Filter: public Sf_Binary_Filter<PointType> {

    void radius_outlier_filter(SF_Param_Radius_Outlier_Filter<PointType> std_params);

public:

    SF_Radius_Outlier_Filter(typename pcl::PointCloud<PointType>::Ptr cloud_in);

    virtual void compute(const SF_Param_Radius_Outlier_Filter<PointType> &params);

};

#include <pcl/cloud/filter/binary/radiusoutlier/sf_radius_outlier_filter.hpp>

#endif // SF_RADIUS_OUTLIER_FILTER_H
