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

#ifndef SF_RADIUS_OUTLIER_FILTER_HPP
#define SF_RADIUS_OUTLIER_FILTER_HPP

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <QDebug>

#include "pcl/cloud/filter/binary/radiusoutlier/sf_radiusOutlierFilter.h"
#include "converters/CT_To_PCL/sf_converterCTToPCL.h"
#include "ct_colorcloud/ct_colorcloudstdvector.h"
#include "ct_itemdrawable/ct_pointsattributescolor.h"

template <typename PointType>
SF_RadiusOutlierFilter<PointType>::SF_RadiusOutlierFilter() {
    Sf_AbstractBinaryFilter<PointType>::reset();
}

template <typename PointType>
void  SF_RadiusOutlierFilter<PointType>::compute(const SF_ParamRadiusOutlierFilter<PointType> &params) {
    if(SF_AbstractCloud<PointType>::_cloudIn->points.size() >= 1) {
        SF_RadiusOutlierFilter<PointType>::radiusOutlierFilter(params);
    } else {
        qDebug() << "SF_Radius_Outlier_Filter<PointType>::compute - not enough points.";
    }
    SF_RadiusOutlierFilter<PointType>::createIndices();
}
template<typename PointType>
CT_ColorCloudStdVector
*SF_RadiusOutlierFilter<PointType>::colors() const
{
    return _colors;
}


template <typename PointType>
void SF_RadiusOutlierFilter<PointType>::radiusOutlierFilter(SF_ParamRadiusOutlierFilter<PointType> stdParams)
{
    _colors = new CT_ColorCloudStdVector(SF_AbstractCloud<PointType>::_cloudIn->points.size());
    float maxNumber = 0;
    float minNumber = std::numeric_limits<float>::max();
    if(stdParams._radius>0.1f)
    {
        Sf_ConverterCTToPCL<PointType> converter;
        converter.setItemCpyCloudInDeprecated(stdParams._itemCpyCloudIn);
        typename pcl::PointCloud<PointType>::Ptr downscaledCloud (new pcl::PointCloud<PointType>);
        converter.downScale(stdParams._radius/5,downscaledCloud);
        typename pcl::PointCloud<PointType>::Ptr downscaledCloudNeighborsNumber  (new pcl::PointCloud<PointType>);
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud (downscaledCloud);
        for(size_t i = 0; i < downscaledCloud->points.size(); i++) {
            PointType p =  downscaledCloud->points[i];
            float numberneighbors = 0;
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if ( kdtree.radiusSearch (p, stdParams._radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
                for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
                    numberneighbors += downscaledCloud->points[ pointIdxRadiusSearch[j] ].intensity;
            }
            p.intensity = numberneighbors;
            if(numberneighbors > maxNumber)
            {
                maxNumber = numberneighbors;
            }
            if(numberneighbors < minNumber)
            {
                minNumber = numberneighbors;
            }
            downscaledCloudNeighborsNumber->push_back(p);
        }
        float range = maxNumber- minNumber;
        float sqrdRange = std::sqrt(std::sqrt(range));
        pcl::KdTreeFLANN<PointType> kdtree_nn;
        kdtree_nn.setInputCloud (downscaledCloudNeighborsNumber);
        for(size_t i = 0; i < SF_AbstractCloud<PointType>::_cloudIn->points.size(); i++) {
            CT_Color &col = _colors->colorAt(i);
            PointType p = SF_AbstractCloud<PointType>::_cloudIn->points[i];
            std::vector<int> pointIdxNNSearch;
            std::vector<float> pointNNSquaredDistance;
            if ( kdtree.nearestKSearch (p, 1, pointIdxNNSearch, pointNNSquaredDistance) > 0 ) {
                float neighbors = downscaledCloudNeighborsNumber->points[ pointIdxNNSearch[0] ].intensity;
                float perc = (range == 0) ? 0 : std::sqrt(std::sqrt(neighbors - minNumber))/sqrdRange;
                col.r() = (255 - perc*255);
                col.g() = (perc*255);
                col.b() = (0);
                if(neighbors>= stdParams._minPts) {
                    SF_AbstractFilterDeprecated<PointType>::_cloudOutFiltered->points.push_back(p);
                }
            }
        }
    } else {
        pcl::RadiusOutlierRemoval<PointType> outrem;
        outrem.setInputCloud (SF_AbstractCloud<PointType>::_cloudIn);
        outrem.setRadiusSearch(stdParams._radius);
        outrem.setMinNeighborsInRadius( stdParams._minPts);
        outrem.filter (*SF_AbstractFilterDeprecated<PointType>::_cloudOutFiltered);
    }
}

#endif // SF_RADIUS_OUTLIER_FILTER_HPP
