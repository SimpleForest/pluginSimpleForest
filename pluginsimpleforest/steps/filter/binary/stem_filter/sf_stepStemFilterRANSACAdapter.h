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

#ifndef SF_STEP_STEM_FILTER_RANSAC_ADAPTER_H
#define SF_STEP_STEM_FILTER_RANSAC_ADAPTER_H

#include "pcl/cloud/filter/binary/stem/sf_stemRansacFilter.h"
#include <QThreadPool>
#include <iostream>

#include "converters/CT_To_PCL/sf_converterCTToPCL.h"
#include "steps/param/sf_paramAllSteps.h"

class SF_StepStemFilterRANSACAdapter {
public:
  std::shared_ptr<QMutex> mMutex;

  SF_StepStemFilterRANSACAdapter(const SF_StepStemFilterRANSACAdapter &obj) {
    mMutex = obj.mMutex;
  }

  SF_StepStemFilterRANSACAdapter() {
    QThreadPool::globalInstance()->setMaxThreadCount(1);
    mMutex.reset(new QMutex);
  }

  ~SF_StepStemFilterRANSACAdapter() {}

  void operator()(SF_ParamStemRansacFilter &params) {
    Sf_ConverterCTToPCL<pcl::PointXYZINormal> converter;
    {
      QMutexLocker m1(&*mMutex);
      converter.setItemCpyCloudInDeprecated(params._itemCpyCloudIn);
    }
    converter.compute();
    {
      QMutexLocker m1(&*mMutex);
      params._cloudIn = converter.cloudTranslated();
    }
    SF_StemRANSACFilter filter;
    {
      QMutexLocker m1(&*mMutex);
      filter.setCloudIn(params._cloudIn);
      filter.setParams(params);
    }
    filter.compute();
    {
      QMutexLocker m1(&*mMutex);
      params._outputIndices = filter.getIndices();
    }
  }
};
#endif // SF_STEP_STEM_FILTER_RANSAC_ADAPTER_H
