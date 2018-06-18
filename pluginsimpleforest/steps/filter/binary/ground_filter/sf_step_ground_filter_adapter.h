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
#ifndef SF_STEP_GROUND_FILTER_ADAPTER_H
#define SF_STEP_GROUND_FILTER_ADAPTER_H

#include <pcl/cloud/filter/binary/ground/sf_ground_filter.h>
#include <QThreadPool>

#include "steps/param/sf_abstract_param.h"
#include "converters/CT_To_PCL/sf_converter_ct_to_pcl.h"
class SF_Step_Ground_Filter_Adapter {
public:

    std::shared_ptr<QMutex>  mMutex;

    SF_Step_Ground_Filter_Adapter(const SF_Step_Ground_Filter_Adapter &obj) {
        mMutex = obj.mMutex;
    }

    SF_Step_Ground_Filter_Adapter () {
        mMutex.reset(new QMutex);
    }

    ~SF_Step_Ground_Filter_Adapter () {
    }

    void operator()(SF_Param_Ground_Filter<SF_Point_N> & params) {
        SF_Converter_CT_To_PCL<SF_Point_N> converter;
        {
            QMutexLocker m1(&*mMutex);
            converter.set_itemCpy_cloud_in(params._itemCpy_cloud_in);
        }
        converter.compute();
        {
            QMutexLocker m1(&*mMutex);
            params._cloud_in = converter.get_cloud_translated();
        }
        params.log_import();
        SF_Ground_Filter<SF_Point_N> filter;
        {
            QMutexLocker m1(&*mMutex);
            filter.set_cloud_in(params._cloud_in);
            filter.set_params(params);
        }
        filter.compute();
        {
            QMutexLocker m1(&*mMutex);
            params._output_indices = filter.get_indices();
        }
        params.log_filter(filter.get_percentage());
    }
};

#endif // SF_STEP_GROUND_FILTER_ADAPTER_H
