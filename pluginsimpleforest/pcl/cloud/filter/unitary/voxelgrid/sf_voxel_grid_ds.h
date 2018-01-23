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
#ifndef SF_VOXEL_GRID_DS_H
#define SF_VOXEL_GRID_DS_H

#include <pcl/cloud/filter/unitary/sf_unitary_filter.h>
#include "pcl/filters/voxel_grid.h"

template <typename PointType>
class SF_Voxel_Grid_DS: public SF_Unitary_Filter<PointType>
{

    void voxel_grid_downscale(SF_Param_Voxel_Grid_Downscale<PointType> std_params);

public:

    SF_Voxel_Grid_DS(typename pcl::PointCloud<PointType>::Ptr cloud_in);

    virtual void compute(const SF_Param_Voxel_Grid_Downscale<PointType> &params);

    void set_leaf_size(typename pcl::VoxelGrid<pcl::PointCloud<PointType> > &sor);

};

#include "sf_voxel_grid_ds.hpp"

#endif // SF_VOXEL_GRID_DS_H
