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
SF_PrincipalDirection<PointType>::SF_PrincipalDirection()
{}
template<typename PointType>
pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr
SF_PrincipalDirection<PointType>::principalCurvatures()
{
  return m_principalCurvatures;
}

template<typename PointType>
SF_ParameterSetPrincipalDirection<PointType>
SF_PrincipalDirection<PointType>::params() const
{
  return m_params;
}

template<typename PointType>
void
SF_PrincipalDirection<PointType>::compute()
{
  typename pcl::PointCloud<PointType>::Ptr cloud = m_params.m_cloud.first;
  m_principalCurvatures.reset(new pcl::PointCloud<pcl::PrincipalCurvatures>);
  // Compute the normals
  pcl::NormalEstimation<PointType, PointType> normalEstimation;
  normalEstimation.setInputCloud(cloud);
  typename pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  normalEstimation.setSearchMethod(tree);
  normalEstimation.setRadiusSearch(m_params.m_normalRadius);
  normalEstimation.compute(*cloud);

  // Setup the principal curvatures computation
  pcl::PrincipalCurvaturesEstimation<PointType, PointType, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
  principalCurvaturesEstimation.setInputCloud(cloud);
  principalCurvaturesEstimation.setInputNormals(cloud);
  principalCurvaturesEstimation.setSearchMethod(tree);
  principalCurvaturesEstimation.setRadiusSearch(m_params.m_pdRadius);
  m_principalCurvatures.reset(new pcl::PointCloud<pcl::PrincipalCurvatures>());
  principalCurvaturesEstimation.compute(*m_principalCurvatures);
  m_params.m_principalCurvatures = m_principalCurvatures;
}

template<typename PointType>
void
SF_PrincipalDirection<PointType>::setParams(SF_ParameterSetPrincipalDirection<PointType>& params)
{
  m_params = params;
  compute();
}

#endif // SF_PRINCIPALDIRECTION_HPP
