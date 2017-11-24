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
#ifndef SF_ABSTRACT_CLOUD_HPP
#define SF_ABSTRACT_CLOUD_HPP

#include <pcl/cloud/sf_abstract_cloud.h>

template <typename PointType>
SF_Abstract_Cloud<PointType>::SF_Abstract_Cloud(typename pcl::PointCloud<PointType>::Ptr cloud_in):
     _cloud_in(cloud_in)
{

}
template <typename PointType>
bool SF_Abstract_Cloud<PointType>::equals_by_sqrt_distance(float sqrt_distance)
{
    if(sqrt_distance <_MIN_SQUARED_DISTANCE)
        return true;
    return false;
}



#endif // SF_ABSTRACT_CLOUD_HPP
