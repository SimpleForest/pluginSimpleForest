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
#ifndef SF_ABSTRACT_PARAM_H
#define SF_ABSTRACT_PARAM_H

#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_result/model/outModel/ct_outresultmodelgroup.h"
#include "pcl/sf_point.h"





struct SF_Param_CT{
    CT_StandardItemGroup* _grpCpy_grp;
    const CT_AbstractItemDrawableWithPointCloud* _itemCpy_cloud_in;
    CT_ResultGroup* _resCpy_res;
};

template <typename PointType>
struct SF_Param_Cloud: public SF_Param_CT{
    typename pcl::PointCloud<PointType>::Ptr _cloud_in;
};
template <typename PointType>
struct SF_Param_Filter: public SF_Param_Cloud<PointType>{
    int _size_output;
    std::vector<int> _output_indices;
};
template <typename PointType>
struct SF_Param_Statistical_Outlier_Filter : public SF_Param_Filter<PointType>{
    int _k;
    float _std_mult;
    int _iterations;
};
template <typename PointType>
struct SF_Param_Radius_Outlier_Filter : public SF_Param_Filter<PointType>{
    int _min_Pts;
    double _radius;


};

#endif // SF_ABSTRACT_PARAM_H
