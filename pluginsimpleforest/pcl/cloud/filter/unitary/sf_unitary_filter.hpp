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
#ifndef SF_UNITARY_FILTER_HPP
#define SF_UNITARY_FILTER_HPP
#include "sf_unitary_filter.h"
template <typename PointType>
SF_Unitary_Filter<PointType>::SF_Unitary_Filter() {
    reset();
}

template<typename PointType>
void SF_Unitary_Filter<PointType>::reset() {
    SF_Abstract_Filter<PointType>::_cloud_out_filtered.reset(new pcl::PointCloud<PointType>);
    SF_Abstract_Filter<PointType>::_indices.clear();
}

template<typename PointType>
void SF_Unitary_Filter<PointType>::create_index(PointType point,
                                                float sqrd_distance)
{
    if(SF_Unitary_Filter<PointType>::equals_by_sqrt_distance(sqrd_distance))
    {SF_Abstract_Cloud<PointType>::_indices.push_back(0);}
    else
    {SF_Abstract_Cloud<PointType>::_indices.push_back(-1);}
}

#endif // SF_UNITARY_FILTER_HPP
