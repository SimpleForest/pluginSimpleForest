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
#ifndef SF_RADIUS_OUTLIER_FILTER_ADAPTER_H
#define SF_RADIUS_OUTLIER_FILTER_ADAPTER_H

#include "steps/param/sf_abstract_param.h"
#include <converters/CT_To_PCL/sf_converterCTToPCL.h>
#include <pcl/cloud/filter/binary/radiusoutlier/sf_radius_outlier_filter.h>

class SF_Radius_Outlier_Filter_Adapter {

public:    

    std::shared_ptr<QMutex>  mMutex;

    SF_Radius_Outlier_Filter_Adapter(const SF_Radius_Outlier_Filter_Adapter &obj) {
        mMutex = obj.mMutex;
    }

    SF_Radius_Outlier_Filter_Adapter () {
        mMutex.reset(new QMutex);
    }

    ~SF_Radius_Outlier_Filter_Adapter () {
    }

    void operator()(SF_Param_Radius_Outlier_Filter<SF_PointNormal> & params) {
        Sf_ConverterCTToPCL<SF_PointNormal> converter;
        {
            QMutexLocker m1(&*mMutex);
            converter.setItemCpyCloudIn(params._itemCpyCloudIn);
        }
        converter.compute();
        {
            QMutexLocker m1(&*mMutex);
            params._cloud_in = converter.getCloudTranslated();
        }
        params.log_import();
        SF_Radius_Outlier_Filter<SF_PointNormal> filter;
        {
            QMutexLocker m1(&*mMutex);
            filter.setCloudIn(params._cloud_in);
        }
        filter.compute(params);
        {
            QMutexLocker m1(&*mMutex);
            params._output_indices = filter.getIndices();
        }
        params.log_filter(filter.get_percentage());
    }
};

#endif // SF_RADIUS_OUTLIER_FILTER_ADAPTER_H
