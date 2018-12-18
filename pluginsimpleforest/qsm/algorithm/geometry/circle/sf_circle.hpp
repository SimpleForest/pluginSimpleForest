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

#ifndef SF_CIRCLE_HPP
#define SF_CIRCLE_HPP

#include "sf_circle.h"

template<typename PointType>
SF_Circle::SF_Circle(pcl::PointCloud<PointType>::Ptr cloudIn, const std::vector<int> &indices, const SF_ParamSpherefollowingBasic &params):
    m_cloudIn(cloudIn), m_indices(indices), m_params(params)
{
    pcl::ModelCoefficients circleMedian   = cirlceMedian();
    pcl::ModelCoefficients circleSACModel = cirlceSACModel();
    chooseModel(circleMedian,circleSACModel);
}


pcl::ModelCoefficients SF_Circle::coeff() const
{
    return m_coeff;
}

pcl::ModelCoefficients SF_Circle::cirlceMedian()
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(m_cloudIn, m_indices, centroid);
    std::vector<float> distances(m_cloudIn->points.size());
    size_t index = 0;
    std::for_each(m_indices.begin(), m_indices.end(), [&distances, index, &centroid](int pointIndex){
        Eigen::Vector3f diff = m_cloudIn->points[pointIndex].getVector3fMap () - Eigen::Vector3f (centroid[0], centroid[1] , centroid[2]);
        distances[index++] = diff.norm();
    });
    pcl::ModelCoefficients circleMedian;
    circleMedian.values.push_back(centroid[0]);
    circleMedian.values.push_back(centroid[1]);
    circleMedian.values.push_back(centroid[2]);
    circleMedian.values.push_back(SF_Math::getMedian(distances));
    return circleMedian;
}

template<typename PointType>
pcl::ModelCoefficients SF_Circle::cirlceSACModel()
{    pcl::PointIndices::Ptr inliersCylinder (new pcl::PointIndices);
     pcl::ModelCoefficients coeff;
     pcl::SACSegmentationFromNormals<PointType, PointType> seg;
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_CIRCLE3D);
     seg.setNormalDistanceWeight (0.1);
     seg.setMethodType (m_params._sphereFollowingParams._fittingMethod);
     seg.setMaxIterations (std::min(m_params._sphereFollowingParams._RANSACIterations, m_cloudIn->points.size()*m_cloudIn->points.size() ) );
     seg.setDistanceThreshold (m_params._sphereFollowingParams._inlierDistance);
     seg.setInputCloud (m_cloudIn);
     seg.setInputNormals (m_cloudIn);
     seg.setIndices(m_indices);
     seg.segment ( *inliersCylinder, coeff);
     return coeff;
}

template<typename PointType>
void SF_Circle::chooseModel(const pcl::ModelCoefficients &circleMedian, const pcl::ModelCoefficients &circleSACModel)
{
    int index;
    if(circleSACModel.values.size()==4)
    {
        if(circleSACModel.values[3] < circleMedian.values[3]*m_params._sphereFollowingParams._optimizationParams)
        {
            m_coeff = circleSACModel;
        }
        else
        {
            m_coeff = circleMedian;
        }
    }
    else
    {
        m_coeff = circleMedian;
    }
}
#endif // SF_CIRCLE_HPP
