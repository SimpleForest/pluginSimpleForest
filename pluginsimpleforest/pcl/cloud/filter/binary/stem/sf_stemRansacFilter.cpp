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

#include "sf_stemRansacFilter.h"
#include "pcl/sf_math.h"
#include <cmath>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

SF_StemRANSACFilter::SF_StemRANSACFilter()
{
  Sf_AbstractBinaryFilter<pcl::PointXYZINormal>::reset();
}

void
SF_StemRANSACFilter::computeNormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
{
  pcl::NormalEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(_params._radiusNormal);
  ne.compute(*cloud);
}

void
SF_StemRANSACFilter::segment(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                             pcl::ModelCoefficients::Ptr coeffCylinder,
                             pcl::PointIndices::Ptr inliersCylinder)
{
  pcl::SACSegmentationFromNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> seg;
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setOptimizeCoefficients(false);
  seg.setMethodType(pcl::SAC_MLESAC);
  seg.setMaxIterations(_params._iterations);
  seg.setDistanceThreshold(_params._inlierDistance);
  seg.setRadiusLimits(0, 2);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud);
  seg.segment(*inliersCylinder, *coeffCylinder);
}

void
SF_StemRANSACFilter::addInliers(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudFiltered,
                                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                                pcl::ModelCoefficients::Ptr coeffCylinder,
                                pcl::ModelCoefficients::Ptr lastCoeffCylinder,
                                pcl::PointIndices::Ptr inliersCylinder)
{
  if (static_cast<int>(inliersCylinder->indices.size()) > std::max<int>((int)(std::ceil(1 / _params._voxelSize) / 2), 10)) {
    Eigen::Vector3f zAxis(_params._x, _params._y, _params._z);
    if (lastCoeffCylinder->values.size() == 7) {
      zAxis[0] = lastCoeffCylinder->values[3];
      zAxis[1] = lastCoeffCylinder->values[4];
      zAxis[2] = lastCoeffCylinder->values[5];
    }
    Eigen::Vector3f cylinderAxis(coeffCylinder->values[3], coeffCylinder->values[4], coeffCylinder->values[5]);
    float angle = SF_Math<float>::getAngleBetweenDegf(zAxis, cylinderAxis);
    if (!(angle > _params._angle && angle < (180 - _params._angle))) {
      *lastCoeffCylinder = *coeffCylinder;
      for (size_t j = 0; j < inliersCylinder->indices.size(); ++j)
        cloudFiltered->points.push_back(cloud->points[inliersCylinder->indices[j]]);
    }
  }
}

void
SF_StemRANSACFilter::filterIteratively(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScaledCloudFiltered)
{
  pcl::ModelCoefficients::Ptr lastcoeffCylinder(new pcl::ModelCoefficients);
  for (size_t i = 0; i < _clouds.size(); i++) {
    pcl::ModelCoefficients::Ptr coeffCylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliersCylinder(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = _clouds[i];
    //    if (lastcoeffCylinder->values.size() == 7) {
    //      pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudInlier(new pcl::PointCloud<pcl::PointXYZINormal>());
    //      Eigen::Vector3d x1(lastcoeffCylinder->values[0], lastcoeffCylinder->values[1], lastcoeffCylinder->values[2]);
    //      Eigen::Vector3d x2(lastcoeffCylinder->values[0] + lastcoeffCylinder->values[3],
    //                         lastcoeffCylinder->values[1] + lastcoeffCylinder->values[4],
    //                         lastcoeffCylinder->values[2] + lastcoeffCylinder->values[5]);
    //      for (size_t i = 0; i < cloud->points.size(); i++) {
    //        Eigen::Vector3d x0(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    //        Eigen::Vector3d a = x0 - x1;
    //        Eigen::Vector3d b = x0 - x2;
    //        Eigen::Vector3d c = x2 - x1;
    //        double dist = ((a.cross(b)).norm()) / (c.norm());
    //        float radius = std::max<float>((float)lastcoeffCylinder->values[6] + _params._inlierDistance,
    //                                       (float)(lastcoeffCylinder->values[6]) * 1.5);
    //        if (dist < radius) {
    //          cloudInlier->points.push_back(cloud->points[i]);
    //        }
    //      }
    //      cloud = cloudInlier;
    //    }
    segment(cloud, coeffCylinder, inliersCylinder);
    for (size_t j = 0; j < inliersCylinder->indices.size(); ++j)
      downScaledCloudFiltered->points.push_back(cloud->points[inliersCylinder->indices[j]]);
    //    addInliers(downScaledCloudFiltered, cloud, coeffCylinder, lastcoeffCylinder, inliersCylinder);
  }
}

void
SF_StemRANSACFilter::backScale(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScaledCloudFiltered)
{
  _cloudOutFilteredNoise.reset(new typename pcl::PointCloud<pcl::PointXYZINormal>);
  _cloudOutFiltered.reset(new typename pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
  kdtree.setInputCloud(downScaledCloudFiltered);
  for (size_t i = 0; i < _cloudIn->points.size(); i++) {
    pcl::PointXYZINormal point = _cloudIn->points[i];
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      if (std::sqrt(pointNKNSquaredDistance[0]) < _params._voxelSize * 2.5) {
        _cloudOutFiltered->points.push_back(point);
      } else {
        _cloudOutFilteredNoise->points.push_back(point);
      }
    } else {
      _cloudOutFilteredNoise->points.push_back(point);
    }
  }
}

void
SF_StemRANSACFilter::compute()
{
  std::cout << "FOO "
            << " voxelSize " << _params._voxelSize << " inlierDistance " << _params._inlierDistance << "iterations "
            << _params._iterations << std::endl;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScaledCloud = downScale(_params._voxelSize);
  computeNormals(downScaledCloud);
  initializeClouds(downScaledCloud);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScaledCloudFiltered(new SF_CloudNormal());
  filterIteratively(downScaledCloudFiltered);
  if (downScaledCloudFiltered->points.size() > 0) {
    backScale(downScaledCloudFiltered);
  }
  createIndices();
}

void
SF_StemRANSACFilter::initializeClouds(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
{
  _clouds.clear();
  if (_params._sliceClouds) {
    int size = static_cast<int>(cloud->points.size());
    float zMin = std::numeric_limits<float>::max();
    float zMax = std::numeric_limits<float>::lowest();
    for (int i = 0; i < size; i++) {
      pcl::PointXYZINormal point = cloud->points[i];
      if (point.z < zMin)
        zMin = point.z;
      if (point.z > zMax)
        zMax = point.z;
    }
    int sliceNumber = std::ceil(zMax - zMin);
    for (int i = 0; i < sliceNumber; i++) {
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr slice(new pcl::PointCloud<pcl::PointXYZINormal>());
      _clouds.push_back(slice);
    }
    for (int i = 0; i < size; i++) {
      pcl::PointXYZINormal point = cloud->points[i];
      int index = std::floor(point.z - zMin);
      _clouds[index]->points.push_back(point);
    }
  } else {
    _clouds.push_back(cloud);
  }
}

void
SF_StemRANSACFilter::setParams(const SF_ParamStemRansacFilter& params)
{
  _params = params;
}
