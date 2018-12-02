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
#ifndef SF_CONVERTER_CT_TO_PCL_H
#define SF_CONVERTER_CT_TO_PCL_H

#include <pcl/sf_point.h>

#include <converters/sf_abstractConverter.h>
#include <ct_itemdrawable/ct_grid3d_sparse.h>
#include "ct_iterator/ct_pointiterator.h"

template <typename PointType>
class Sf_ConverterCTToPCL:
        public SF_AbstractConverter
{
public:
    Sf_ConverterCTToPCL();
    void compute();
    void downScale(float range,
                    typename pcl::PointCloud<PointType>::Ptr downscaledCloud);
    typename
    pcl::PointCloud<PointType>::Ptr cloudTranslated() const;
    typename
    pcl::PointCloud<PointType>::Ptr getCloudOriginal() const;
private:
    typename pcl::PointCloud<PointType>::Ptr _cloudTranslated;
    typename pcl::PointCloud<PointType>::Ptr _cloudOriginal;
    void iterateCloudAndConvert(const CT_AbstractPointCloudIndex *index);
    void convertPoint(CT_PointIterator &it);
    void convert();

    std::vector<typename pcl::PointCloud<PointType>::Ptr> mergeSubCloudsToVector(CT_Grid3D_Sparse<int>* indices);
    void mergeSubCloudsToVector(CT_Grid3D_Sparse<int>* indices,
                                std::vector<typename pcl::PointCloud<PointType>::Ptr> cloudsVec,
                                typename pcl::PointCloud<PointType>::Ptr downscaled_cloud);
};


#endif // SF_CONVERTER_CT_TO_PCL_H

#include "sf_converterCTToPCL.hpp"
