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
#ifndef SF_STATISTICAL_OUTLIER_FILTER_HPP
#define SF_STATISTICAL_OUTLIER_FILTER_HPP

#include <pcl/filters/statistical_outlier_removal.h>

#include "pcl/cloud/filter/binary/statisticaloutlier/sf_statisticalOutlierFilter.h"

template <typename PointType>
SF_StatisticalOutlierFilter<PointType>::SF_StatisticalOutlierFilter() {}

template <typename PointType>
void SF_StatisticalOutlierFilter<PointType>::compute(
    const SF_ParamStatisticalOutlierFilter<PointType> &params) {
  SF_StatisticalOutlierFilter<PointType>::_cloudOutFiltered.reset(
      new pcl::PointCloud<PointType>());
  SF_StatisticalOutlierFilter<PointType>::statisticalOutlierFilterIteratively(
      params);
  SF_StatisticalOutlierFilter<PointType>::createIndices();
}

template <typename PointType>
void SF_StatisticalOutlierFilter<PointType>::statisticalOutlierFilter(
    SF_ParamStatisticalOutlierFilter<PointType> stdParams,
    typename pcl::PointCloud<PointType>::Ptr cloud) {
  SF_StatisticalOutlierFilter<PointType>::_cloudOutFiltered.reset(
      new pcl::PointCloud<PointType>());
  pcl::StatisticalOutlierRemoval<PointType> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(stdParams._k);
  sor.setStddevMulThresh(stdParams._stdMult);
  sor.filter(*SF_StatisticalOutlierFilter<PointType>::_cloudOutFiltered);
}

template <typename PointType>
void SF_StatisticalOutlierFilter<PointType>::
    statisticalOutlierFilterIteratively(
        SF_ParamStatisticalOutlierFilter<PointType> stdParams) {
  typename pcl::PointCloud<PointType>::Ptr cloud(
      new pcl::PointCloud<PointType>());
  cloud = SF_StatisticalOutlierFilter<PointType>::_cloudIn;
  SF_StatisticalOutlierFilter<PointType>::iterate(stdParams, cloud);
}

template <typename PointType>
void SF_StatisticalOutlierFilter<PointType>::iterate(
    SF_ParamStatisticalOutlierFilter<PointType> params,
    typename pcl::PointCloud<PointType>::Ptr cloud) {
  int iterations = params._iterations;
  while (iterations > 0) {
    SF_StatisticalOutlierFilter<PointType>::statisticalOutlierFilter(params,
                                                                     cloud);
    if (cloud->points.size() ==
        SF_StatisticalOutlierFilter<PointType>::_cloudOutFiltered->points
            .size()) {
      break;
    }
    cloud = SF_StatisticalOutlierFilter<PointType>::_cloudOutFiltered;
    iterations--;
  }
}

#endif // SF_STATISTICAL_OUTLIER_FILTER_HPP
