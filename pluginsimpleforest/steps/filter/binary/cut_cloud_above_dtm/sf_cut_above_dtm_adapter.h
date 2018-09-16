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

#ifndef SF_CUT_ABOVE_DTM_ADAPTER_H
#define SF_CUT_ABOVE_DTM_ADAPTER_H

#include <pcl/cloud/filter/binary/ground/sf_ground_filter.h>
#include <QThreadPool>

#include "steps/param/sf_abstract_param.h"
#include "converters/CT_To_PCL/sf_converter_ct_to_pcl.h"
#include "converters/CT_To_PCL/sf_converter_ct_to_pcl_dtm.h".h"


class SF_Step_Cut_Above_DTM_Adapter {
public:
    std::shared_ptr<QMutex>  mMutex;

    SF_Step_Cut_Above_DTM_Adapter(const SF_Step_Cut_Above_DTM_Adapter &obj) {
        mMutex = obj.mMutex;
    }

    SF_Step_Cut_Above_DTM_Adapter () {
        mMutex.reset(new QMutex);
    }

    ~SF_Step_Cut_Above_DTM_Adapter () {
    }

    void operator()(SF_Param_DTM_Height<pcl::PointXYZ> & params) {
        SF_Converter_CT_To_PCL<pcl::PointXYZ> converterCloud;
        {
            QMutexLocker m1(&*mMutex);
            converterCloud.setItemCpyCloudIn(params._itemCpyCloudIn);
        }
        converterCloud.compute();
        float _cutHeight;
        CT_Image2D<float> * dtmCT;
        {
            QMutexLocker m1(&*mMutex);
            params._cloud_in = converterCloud.get_cloud_translated();
            _cutHeight = params._cropHeight;
            dtmCT = params._dtmCT;
        }
        SF_Converter_CT_to_PCL_DTM dtmConverter(converterCloud.getCenterOfMass(), params._dtmCT);
        std::shared_ptr<SF_DTM_Model> dtmModel = dtmConverter.dtmPCL();
        std::vector<int> indices;
        for(size_t j = 0; j < converterCloud.get_cloud_translated()->points.size(); j++) {
            pcl::PointXYZ p = converterCloud.get_cloud_translated()->points[j];
            if(dtmModel->heightAbove(p) < _cutHeight) {
                indices.push_back(0);
            } else {
                indices.push_back(1);
            }
        }
        {
            QMutexLocker m1(&*mMutex);
            params._output_indices = indices;
        }
    }
};

#endif // SF_CUT_ABOVE_DTM_ADAPTER_H
