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

#ifndef SF_RASTER_MODEL_H
#define SF_RASTER_MODEL_H

#include <pcl/kdtree/kdtree_flann.h>

#include "pcl/sf_point.h"

class SF_ModelRaster
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud3D;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud2D;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr _kdtree2D;
  void create2DFrom3D();
  void createKDTree();
  void getNearestNeighborsHeightsAndDistances(const pcl::PointXYZ& point,
                                              const int nn,
                                              std::vector<float>& heightsOut,
                                              std::vector<float>& distancesOut);

public:
  SF_ModelRaster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D);
  void interpolateIDW(const int knn, std::shared_ptr<SF_ModelRaster> dst);
  void interpolateMedian(const int knn, std::shared_ptr<SF_ModelRaster> dst);
  float heightAt(float x, float y);

  pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud3D() const;
  void setCloud3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D);
};

#endif // SF_RASTER_MODEL_H
