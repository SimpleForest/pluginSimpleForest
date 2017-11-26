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
#ifndef SF_STEP_STATISTICAL_OUTLIER_REMOVAL_ADAPTER_H
#define SF_STEP_STATISTICAL_OUTLIER_REMOVAL_ADAPTER_H

#include <steps/param/sf_abstract_param.h>
#include <converters/CT_To_PCL/sf_converter_ct_to_pcl.h>
#include <pcl/cloud/filter/binary/statisticaloutlier/sf_statistical_outlier_filter.h>


#include <QDebug>

class SF_Step_Statistical_Outlier_Removal_Adapter
{
public:
    SF_Step_Statistical_Outlier_Removal_Adapter();

    virtual void operator()(SF_Param_Statistical_Outlier_Filter & params)
    {
        SF_Converter_CT_To_PCL<SF_Point> converter( params._itemCpy_cloud_in);
        converter.compute();
        params._cloud_in = converter.get_cloud_translated();
        SF_Statistical_Outlier_Filter<SF_Point> filter (params._cloud_in);
        filter.compute(params);
        params._output_indices = filter.get_indices();
    }
};

#endif // SF_STEP_STATISTICAL_OUTLIER_REMOVAL_ADAPTER_H
