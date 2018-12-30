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

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <cmath>

template <typename PointType>
SF_Circle<PointType>::SF_Circle(typename pcl::PointCloud<PointType>::Ptr cloudIn,
                     const std::vector<int> &indices,
                     const SF_ParamSpherefollowingBasic<PointType> params,
                     size_t paramIndex)
    : m_cloudIn(cloudIn), m_indices(indices), m_params(params),
      m_paramIndex(paramIndex) {
  pcl::ModelCoefficients circleMedian = cirlceMedianWithIndices();
  pcl::ModelCoefficients circleSACModel = cirlceSACModelWithIndices();
  chooseModel(circleMedian, circleSACModel);
}

template <typename PointType>
SF_Circle<PointType>::SF_Circle(typename pcl::PointCloud<PointType>::Ptr cloudIn,
                     const SF_ParamSpherefollowingBasic<PointType> params,
                     size_t paramIndex)
    : m_cloudIn(cloudIn), m_indices {}, m_params(params),
      m_paramIndex(paramIndex) {
  pcl::ModelCoefficients circleMedian = circleMedianWithSubCloud();
  pcl::ModelCoefficients circleSACModel = cirlceSACModelWithSubCloud();
  chooseModel(circleMedian, circleSACModel);
}

template <typename PointType>
pcl::ModelCoefficients SF_Circle<PointType>::cirlceMedianWithIndices() {
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(m_cloudIn, m_indices, centroid);
  std::vector<float> distances(m_indices.size());
  size_t index = 0;
  typename pcl::PointCloud<PointType>::Ptr tmp = m_cloudIn;
  std::for_each(m_indices.begin(), m_indices.end(),
                [&distances, &index, &centroid,  tmp](int pointIndex) {
                  Eigen::Vector3f diff =
                      tmp->points[pointIndex].getVector3fMap() -
                      Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
                  distances[index++] = diff.norm();
                });
  pcl::ModelCoefficients circleMedian;
  circleMedian.values.push_back(centroid[0]);
  circleMedian.values.push_back(centroid[1]);
  circleMedian.values.push_back(centroid[2]);
  circleMedian.values.push_back(SF_Math<float>::getMedian(distances));
  return circleMedian;
}

template <typename PointType>
pcl::ModelCoefficients SF_Circle<PointType>::circleMedianWithSubCloud() {
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*m_cloudIn, centroid);
  std::vector<float> distances(m_cloudIn->points.size());
  size_t index = 0;
  std::for_each(m_cloudIn->points.begin(), m_cloudIn->points.end(),
                [&distances, &index, &centroid, this](PointType &point) {
                  Eigen::Vector3f diff =
                      point.getVector3fMap() -
                      Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
                  distances[index++] = diff.norm();
                });
  pcl::ModelCoefficients circleMedian;
  circleMedian.values.push_back(centroid[0]);
  circleMedian.values.push_back(centroid[1]);
  circleMedian.values.push_back(centroid[2]);
  circleMedian.values.push_back(SF_Math<float>::getMedian(distances));
  return circleMedian;
}

template <typename PointType>
pcl::ModelCoefficients SF_Circle<PointType>::cirlceSACModelWithSubCloud() {
  pcl::PointIndices::Ptr inliersCylinder(new pcl::PointIndices);
  pcl::ModelCoefficients coeff;
  pcl::SACSegmentationFromNormals<PointType, PointType> seg;
  setParam(seg);
  seg.segment(*inliersCylinder, coeff);
  return coeff;
}

template <typename PointType>
pcl::ModelCoefficients SF_Circle<PointType>::cirlceSACModelWithIndices() {
  pcl::PointIndices::Ptr inliersCylinder(new pcl::PointIndices);
  pcl::ModelCoefficients coeff;
  pcl::SACSegmentationFromNormals<PointType, PointType> seg;
  setParam(seg);
  seg.setIndices(m_indices);
  seg.segment(*inliersCylinder, coeff);
  return coeff;
}

template<typename PointType>
void SF_Circle<PointType>::setParam(pcl::SACSegmentationFromNormals<PointType, PointType> &seg) {
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setNormalDistanceWeight(0.8);
    seg.setMethodType(m_params._sphereFollowingParams._fittingMethod);
    int sparse = static_cast<int>(std::round(std::pow(m_cloudIn->points.size(), 1.5) ) );
    int sparseIterations = static_cast<int>(std::max(5,  sparse));
    seg.setMaxIterations(
        std::min(m_params._sphereFollowingParams._RANSACIterations,
                 sparseIterations));
    seg.setDistanceThreshold(m_params._sphereFollowingParams._inlierDistance);
    seg.setInputCloud(m_cloudIn);
    seg.setInputNormals(m_cloudIn);
}

template <typename PointType>
void SF_Circle<PointType>::chooseModel(const pcl::ModelCoefficients &circleMedian,
                            const pcl::ModelCoefficients &circleSACModel) {
  if (circleSACModel.values.size() == 7) {
    if ((circleSACModel.values[3] <
        circleMedian.values[3] *
            m_params._sphereFollowingParams.m_optimizationParams[m_paramIndex]._medianRadiusMultiplier) &&
            (circleSACModel.values[3] > circleMedian.values[3] * 0.5) ){
      m_coeff.values.clear();
      m_coeff.values.push_back(circleSACModel.values[0]);
      m_coeff.values.push_back(circleSACModel.values[1]);
      m_coeff.values.push_back(circleSACModel.values[2]);
      m_coeff.values.push_back(circleSACModel.values[3]);
    } else {
      m_coeff = circleMedian;
    }
  } else {
    m_coeff = circleMedian;
  }
}

template <typename PointType>
pcl::ModelCoefficients SF_Circle<PointType>::coeff() const { return m_coeff; }

#endif // SF_CIRCLE_HPP
