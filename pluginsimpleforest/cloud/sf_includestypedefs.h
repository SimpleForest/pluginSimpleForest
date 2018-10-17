/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_includestypedefs.h is part of SimpleForest - a plugin for the
    Computree platform.

    SimpleForest is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SimpleForest is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with
    SimpleForest. If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#ifndef SF_INCLUDESTYPEDEFS_H
#define SF_INCLUDESTYPEDEFS_H
#define PCL_NO_PRECOMPILE

#include <utility>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ SF_Point;
typedef pcl::PointCloud<SF_Point> SF_Cloud;

typedef pcl::PointXYZINormal SF_PointNormal;
typedef pcl::PointCloud<SF_PointNormal> SF_CloudNormal;

#endif // SF_INCLUDESTYPEDEFS_H
