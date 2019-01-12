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

#include "sf_modelRaster.h"
#include "math/interpolation/sf_interpolation.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr
SF_ModelRaster::getCloud3D() const
{
  return _cloud3D;
}

void
SF_ModelRaster::setCloud3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D)
{
  _cloud3D = cloud3D;
  create2DFrom3D();
  createKDTree();
}

void
SF_ModelRaster::create2DFrom3D()
{
  _cloud2D.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < _cloud3D->points.size(); i++) {
    pcl::PointXYZ point3D = _cloud3D->points[i];
    point3D.z = 0;
    _cloud2D->points.push_back(point3D);
  }
  _cloud2D->width = _cloud2D->points.size();
  _cloud2D->height = 1;
  _cloud2D->is_dense = true;
}

void
SF_ModelRaster::createKDTree()
{
  _kdtree2D.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  _kdtree2D->setInputCloud(_cloud2D);
}

void
SF_ModelRaster::getNearestNeighborsHeightsAndDistances(const pcl::PointXYZ& point,
                                                       const int nn,
                                                       std::vector<float>& heightsOut,
                                                       std::vector<float>& distancesOut)
{
  heightsOut.clear();
  distancesOut.clear();
  std::vector<int> pointIdxNKNSearch(nn);
  std::vector<float> pointNKNSquaredDistance(nn);
  if (_kdtree2D->nearestKSearch(point, nn, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
      heightsOut.push_back(_cloud3D->points[pointIdxNKNSearch[i]].z);
      distancesOut.push_back(std::sqrt(pointNKNSquaredDistance[i]));
    }
  }
}

void
SF_ModelRaster::interpolateIDW(const int knn, std::shared_ptr<SF_ModelRaster> dst)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDst = dst->getCloud3D();
  for (size_t i = 0; i < cloudDst->points.size(); i++) {
    pcl::PointXYZ point = cloudDst->points[i];
    point.z = 0;
    std::vector<float> heightsOut;
    std::vector<float> distancesOut;
    getNearestNeighborsHeightsAndDistances(point, knn, heightsOut, distancesOut);
    float interpolated = SF_Interpolation::interpolateIDW(heightsOut, distancesOut);
    cloudDst->points[i].z = interpolated;
  }
  dst->setCloud3D(cloudDst);
}

void
SF_ModelRaster::interpolateMedian(const int knn, std::shared_ptr<SF_ModelRaster> dst)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDst = dst->getCloud3D();
  for (size_t i = 0; i < cloudDst->points.size(); i++) {
    pcl::PointXYZ point = cloudDst->points[i];
    point.z = 0;
    std::vector<float> heightsOut;
    std::vector<float> distancesOut;
    getNearestNeighborsHeightsAndDistances(point, knn, heightsOut, distancesOut);
    float interpolated = SF_Interpolation::interpolateMedian(heightsOut);
    cloudDst->points[i].z = interpolated;
  }
  dst->setCloud3D(cloudDst);
}

float
SF_ModelRaster::heightAt(float x, float y)
{
  pcl::PointXYZ point(x, y, 0);
  std::vector<float> heightsOut;
  std::vector<float> distancesOut;
  getNearestNeighborsHeightsAndDistances(point, 3, heightsOut, distancesOut);
  float interpolated = SF_Interpolation::interpolateIDW(heightsOut, distancesOut);
  return interpolated;
}

SF_ModelRaster::SF_ModelRaster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D)
{
  _cloud3D = cloud3D;
  create2DFrom3D();
  createKDTree();
}
