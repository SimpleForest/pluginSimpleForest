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

#ifndef SF_STEM_RANSAC_FILTER_H
#define SF_STEM_RANSAC_FILTER_H

#include <pcl/cloud/filter/binary/sf_abstractBinaryFilter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class SF_StemRANSACFilter : public Sf_AbstractBinaryFilter<pcl::PointXYZINormal>
{
  SF_ParamStemRansacFilter _params;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> _clouds;
  void initializeClouds(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);

public:
  void setParams(const SF_ParamStemRansacFilter& params);
  SF_StemRANSACFilter();
  void compute();

private:
  void computeNormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);
  void segment(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
               pcl::ModelCoefficients::Ptr coeffCylinder,
               pcl::PointIndices::Ptr inliersCylinder);
  void addInliers(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudFiltered,
                  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                  pcl::ModelCoefficients::Ptr coeffCylinder,
                  pcl::ModelCoefficients::Ptr lastCoeffCylinder,
                  pcl::PointIndices::Ptr inliersCylinder);
  void filterIteratively(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScaledCloudFiltered);
  void backScale(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScaledCloudFiltered);
};

#endif // SF_STEM_RANSAC_FILTER_H
