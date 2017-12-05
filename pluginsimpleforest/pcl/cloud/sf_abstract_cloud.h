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
#ifndef SF_ABRACT_CLOUD_H
#define SF_ABRACT_CLOUD_H

#include <pcl/sf_point.h>
#include <steps/param/sf_abstract_param.h>

template <typename PointType>
class SF_Abstract_Cloud
{
protected:

    const float _MIN_DISTANCE = 0.0001;

    const float _MIN_SQUARED_DISTANCE = _MIN_DISTANCE*_MIN_DISTANCE;

    bool equals_by_sqrt_distance(float sqrt_distance);

    typename pcl::PointCloud<PointType>::Ptr _cloud_in;

    virtual void reset() = 0;

public:

    SF_Abstract_Cloud(typename pcl::PointCloud<PointType>::Ptr cloud_in);

};

#include "sf_abstract_cloud.hpp"

#endif // SF_ABRACT_CLOUD_H
