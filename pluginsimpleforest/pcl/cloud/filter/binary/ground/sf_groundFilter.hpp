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
#ifndef SF_STEM_FILTER_HPP
#define SF_STEM_FILTER_HPP

#include "ct_colorcloud/ct_colorcloudstdvector.h"
#include "ct_itemdrawable/ct_pointsattributescolor.h"
#include "pcl/cloud/feature/growth_direction/sf_growthDirection.h"
#include "pcl/cloud/filter/binary/ground/sf_groundFilter.h"

template <typename PointType>
CT_ColorCloudStdVector *SF_GroundFilter<PointType>::colors() const {
  return _colors;
}

template <typename PointType>
void SF_GroundFilter<PointType>::transferNormalAndFilter(
    const SF_ParamGroundFilter<PointType> &params,
    typename pcl::PointCloud<PointType>::Ptr cloudIn,
    typename pcl::PointCloud<PointType>::Ptr cloudWithGrowthDirection) {
  _colors = new CT_ColorCloudStdVector(cloudIn->points.size());
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(cloudWithGrowthDirection);
  for (size_t i = 0; i < cloudIn->points.size(); i++) {
    PointType p = cloudIn->points.at(i);
    CT_Color &col = _colors->colorAt(i);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (kdtree.nearestKSearch(p, 1, pointIdxRadiusSearch,
                              pointRadiusSquaredDistance) > 0) {
      PointType gd_point =
          cloudWithGrowthDirection->points[pointIdxRadiusSearch[0]];
      Eigen::Vector3f vec(gd_point.normal_x, gd_point.normal_y,
                          gd_point.normal_z);
      if (gd_point.normal_z < 0) {
        vec = Eigen::Vector3f(-gd_point.normal_x, -gd_point.normal_y,
                              -gd_point.normal_z);
      }
      Eigen::Vector3f vecNorm = vec.normalized();
      col.r() = (std::abs((vecNorm[0] * 126) + 127));
      col.g() = (std::abs((vecNorm[1] * 126) + 127));
      col.b() = (std::abs((vecNorm[2] * 250)));
      Eigen::Vector3f axis1;
      axis1[0] = params._x;
      axis1[1] = params._y;
      axis1[2] = params._z;
      Eigen::Vector3f axis2;
      axis2[0] = gd_point.normal_x;
      axis2[1] = gd_point.normal_y;
      axis2[2] = gd_point.normal_z;
      double deg = SF_Math<double>::getAngleBetweenDeg(axis1, axis2);
      if (deg < params._angle || deg > (180 - params._angle)) {
        SF_GroundFilter<PointType>::_cloudOutFiltered->points.push_back(p);
      } else {
        SF_GroundFilter<PointType>::_cloudOutFilteredNoise->points.push_back(p);
      }
    }
  }
}

template <typename PointType>
void SF_GroundFilter<PointType>::setParams(
    SF_ParamGroundFilter<PointType> &params) {
  _params = params;
}

template <typename PointType> void SF_GroundFilter<PointType>::compute() {
  SF_GroundFilter<PointType>::_cloudOutFilteredNoise.reset(
      new typename pcl::PointCloud<PointType>);
  SF_GroundFilter<PointType>::_cloudOutFiltered.reset(
      new typename pcl::PointCloud<PointType>);
  typename pcl::PointCloud<PointType>::Ptr downScaledCloud =
      SF_AbstractCloud<PointType>::downScale(_params._voxelSize);

  pcl::NormalEstimation<PointType, PointType> ne;
  ne.setInputCloud(downScaledCloud);
  typename pcl::search::KdTree<PointType>::Ptr tree(
      new pcl::search::KdTree<PointType>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(_params._radiusNormal);
  ne.compute(*downScaledCloud);

  transferNormalAndFilter(_params, SF_AbstractCloud<PointType>::_cloudIn,
                          downScaledCloud);
  SF_GroundFilter<PointType>::createIndices();
}

template <typename PointType> SF_GroundFilter<PointType>::SF_GroundFilter() {
  Sf_AbstractBinaryFilter<PointType>::reset();
}

#endif // SF_STEM_FILTER_HPP
