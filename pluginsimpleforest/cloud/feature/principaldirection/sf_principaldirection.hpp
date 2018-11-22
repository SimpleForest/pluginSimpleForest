/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_principaldirection.hpp is part of SimpleForest - a plugin for the
    Computree platform.

    SimpleForest is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SimpleForest is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with
    SimpleForest. If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#ifndef SF_PRINCIPALDIRECTION_HPP
#define SF_PRINCIPALDIRECTION_HPP

#include "sf_principaldirection.h"

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

template<typename PointType>
SF_PrincipalDirection::SF_PrincipalDirection()
{

}

template<typename PointType>
void SF_PrincipalDirection::compute()
{
    pcl::PointCloud<PointType>::Ptr cloud = m_cloudIn.first;

    // Compute the normals
    pcl::NormalEstimation<PointType, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud (cloud);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    normalEstimation.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
    normalEstimation.setRadiusSearch (m_params.m_normalRadius);
    normalEstimation.compute (*cloudWithNormals);

    // Setup the principal curvatures computation
    pcl::PrincipalCurvaturesEstimation<PointType, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
    principalCurvaturesEstimation.setInputCloud (cloud);
    principalCurvaturesEstimation.setInputNormals(cloudWithNormals);
    principalCurvaturesEstimation.setSearchMethod (tree);
    principalCurvaturesEstimation.setRadiusSearch(m_params.m_pdRadius);
    m_principalCurvatures.reset(new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principalCurvaturesEstimation.compute (*m_principalCurvatures);
}

template<typename PointType>
void SF_PrincipalDirection::compute(SF_ParameterSetPrincipalDirection &params)
{
    m_params = params;
    m_cloudIn = m_params.m_cloud;
    compute();
}






#endif // SF_PRINCIPALDIRECTION_HPP
