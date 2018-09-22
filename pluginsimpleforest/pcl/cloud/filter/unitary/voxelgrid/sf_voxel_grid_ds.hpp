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
#ifndef SF_VOXEL_GRID_DS_HPP
#define SF_VOXEL_GRID_DS_HPP

#include "sf_voxel_grid_ds.h"
template <typename PointType>
SF_Voxel_Grid_DS<PointType>::SF_Voxel_Grid_DS() {
    SF_Unitary_Filter<PointType>::reset();
}


template <typename PointType>
void  SF_Voxel_Grid_DS<PointType>::compute(const SF_Param_Voxel_Grid_Downscale<PointType> &params) {
    SF_Voxel_Grid_DS<PointType>::voxel_grid_downscale(params);
    SF_Voxel_Grid_DS<PointType>::createIndices();
}

template <typename PointType>
void SF_Voxel_Grid_DS<PointType>::voxel_grid_downscale(SF_Param_Voxel_Grid_Downscale<PointType> std_params) {
    pcl::VoxelGrid<pcl::PointCloud<PointType> > sor;
    sor.setInputCloud (SF_AbstractFilter<PointType>::_cloudIn);
    if(std_params.is_even) {
        sor.setLeafSize (std_params.voxel_size, std_params.voxel_size, std_params.voxel_size);
    } else {
        sor.setLeafSize (std_params.voxel_size_x, std_params.voxel_size_y, std_params.voxel_size_z);
    }
    sor.filter (* SF_AbstractFilter<PointType>::_cloudOutFiltered);
}

#endif // SF_VOXEL_GRID_DS_HPP
