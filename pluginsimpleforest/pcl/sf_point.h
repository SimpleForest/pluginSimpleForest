/****************************************************************************

 Copyright (C) 2017 Jan Hackenberg
 All rights reserved.

 Contact : jan.hackenberg@posteo.de

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

 PluginForest is an extended version of the SimpleTree platform.

*****************************************************************************/
#ifndef SF_POINT_H
#define SF_POINT_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

struct SF_Point_ID
{
    inline SF_Point_ID (float _x, float _y, float _z):x(_x),y(_y),z(_z),id(-1)
    {
    }

    inline SF_Point_ID ():x(0),y(0),z(0),id(-1)
    {
    }

    bool operator < (const SF_Point_ID & other){
        return id < other.id;
    }

    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    int id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (SF_Point_ID,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (int, id, id)
                                   )

typedef pcl::PointXYZ SF_Point;
typedef pcl::PointCloud<SF_Point> SF_Cloud;

typedef pcl::PointXYZINormal SF_Point_N;
typedef pcl::PointCloud<SF_Point_N> SF_Cloud_Normal;

typedef SF_Point_ID SF_Point_ID;
typedef pcl::PointCloud<SF_Point_ID> PointCloudSF_ID;


#endif // SF_POINT_H
