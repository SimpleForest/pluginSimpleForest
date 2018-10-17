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

#include <pcl/cloud/filter/binary/ground/sf_groundFilter.h>
#include <QThreadPool>

#include "steps/param/sf_paramAllSteps.h"
#include "converters/CT_To_PCL/sf_converterCTToPCL.h"
#include "converters/CT_To_PCL/sf_converterCTToPCLDTM.h".h"


class SF_StepCutCloudAboveDTMAdapter {
public:
    std::shared_ptr<QMutex>  mMutex;

    SF_StepCutCloudAboveDTMAdapter(const SF_StepCutCloudAboveDTMAdapter &obj) {
        mMutex = obj.mMutex;
    }

    SF_StepCutCloudAboveDTMAdapter () {
        mMutex.reset(new QMutex);
    }

    ~SF_StepCutCloudAboveDTMAdapter () {
    }

    void operator()(SF_ParamDTMHeight<pcl::PointXYZ> & params) {
        Sf_ConverterCTToPCL<pcl::PointXYZ> converterCloud;
        {
            QMutexLocker m1(&*mMutex);
            converterCloud.setItemCpyCloudInDeprecated(params._itemCpyCloudIn);
        }
        converterCloud.compute();
        float _cutHeight;
        CT_Image2D<float> * dtmCT;
        {
            QMutexLocker m1(&*mMutex);
            params._cloudIn = converterCloud.cloudTranslated();
            _cutHeight = params._sliceHeight;
            dtmCT = params._dtmCT;
        }
        SF_ConverterCTToPCLDTM dtmConverter(converterCloud.translation(), params._dtmCT);
        std::shared_ptr<SF_ModelDTM> dtmModel = dtmConverter.dtmPCL();
        std::vector<int> indices;
        for(size_t j = 0; j < converterCloud.cloudTranslated()->points.size(); j++) {
            pcl::PointXYZ p = converterCloud.cloudTranslated()->points[j];
            if(dtmModel->heightAbove(p) < _cutHeight) {
                indices.push_back(0);
            } else {
                indices.push_back(1);
            }
        }
        {
            QMutexLocker m1(&*mMutex);
            params._outputIndices = indices;
        }
    }
};

#endif // SF_CUT_ABOVE_DTM_ADAPTER_H
