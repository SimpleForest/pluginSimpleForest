/****************************************************************************

 Copyright (C) 2017-2019 Dr. Jan Hackenberg, free software developer
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

#ifndef SF_STEPSEGMENTTREECLOUDFROMQSMADAPTER_H
#define SF_STEPSEGMENTTREECLOUDFROMQSMADAPTER_H

#include <QThreadPool>
#include <converters/CT_To_PCL/sf_converterCTToPCL.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "qsm/algorithm/cloudQSM/sf_clustercloudbyqsm.h"
#include "steps/param/sf_paramAllSteps.h"
#include "steps/visualization/sf_colorfactory.h"

class SF_SegmentTreeCloudFromQSMAdapter
{
public:
  std::shared_ptr<QMutex> mMutex;

  SF_SegmentTreeCloudFromQSMAdapter(const SF_SegmentTreeCloudFromQSMAdapter& obj) { mMutex = obj.mMutex; }

  SF_SegmentTreeCloudFromQSMAdapter() { mMutex.reset(new QMutex); }

  ~SF_SegmentTreeCloudFromQSMAdapter() {}

  void operator()(SF_ParamSegmentTreeFromQSM<pcl::PointXYZINormal>& params)
  {
    Sf_ConverterCTToPCL<pcl::PointXYZINormal> converter;
    {
      QMutexLocker m1(&*mMutex);
      converter.setItemCpyCloudInDeprecated(params._itemCpyCloudIn);
    }
    converter.compute();
    SF_ClusterCloudByQSM<pcl::PointXYZINormal> clustering;
    Eigen::Vector3d translation = converter.translation();
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    {
      QMutexLocker m1(&*mMutex);
      cloud = converter.cloudTranslated();
      params._qsm->translate(-translation);
    }
    pcl::NormalEstimation<SF_PointNormal, SF_PointNormal> ne;
    {
      QMutexLocker m1(&*mMutex);
      ne.setInputCloud(cloud);
      pcl::search::KdTree<SF_PointNormal>::Ptr tree(new pcl::search::KdTree<SF_PointNormal>());
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(0.03f);
    }
    ne.compute(*cloud);
    {
      QMutexLocker m1(&*mMutex);
      params._cloudIn = cloud;
      clustering.setParams(params);
    }
    clustering.compute();
    {
      QMutexLocker m1(&*mMutex);
      params = clustering.params();
      params._qsm->translate(translation);
    }
  }
};

#endif // SF_STEPSEGMENTTREECLOUDFROMQSMADAPTER_H
