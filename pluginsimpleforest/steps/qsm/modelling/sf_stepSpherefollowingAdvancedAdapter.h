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

#ifndef SF_STEPSPHEREFOLLOWINGADVANCEDADAPTER_H
#define SF_STEPSPHEREFOLLOWINGADVANCEDADAPTER_H

#include <QThreadPool>
#include <converters/CT_To_PCL/sf_converterCTIDToPCLCloud.h>
#include <converters/CT_To_PCL/sf_converterCTToPCL.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "cloud/sf_transferfeature.h"
#include "qsm/algorithm/optimization/downHillSimplex/sf_downhillsimplex.h"
#include "qsm/algorithm/postprocessing/sf_qsmmedianfilter.h"
#include "qsm/algorithm/sf_QSMAlgorithm.h"
#include "qsm/algorithm/sf_QSMCylinder.h"
#include "qsm/algorithm/visualization/sf_visualizefitquality.h"
#include "qsm/build/sf_buildQSM.h"
#include "steps/param/sf_paramAllSteps.h"
#include "steps/visualization/sf_colorfactory.h"

class SF_SpherefollowingAdvancedAdapter
{
public:
  std::shared_ptr<QMutex> mMutex;

  SF_SpherefollowingAdvancedAdapter(const SF_SpherefollowingAdvancedAdapter& obj) { mMutex = obj.mMutex; }

  SF_SpherefollowingAdvancedAdapter() { mMutex.reset(new QMutex); }

  ~SF_SpherefollowingAdvancedAdapter() {}

  void operator()(SF_ParamSpherefollowingAdvanced<SF_PointNormal>& params)
  {
    Sf_ConverterCTToPCL<SF_PointNormal> converter;
    {
      QMutexLocker m1(&*mMutex);
      converter.setItemCpyCloudInDeprecated(params._itemCpyCloudIn);
    }
    SF_CloudNormal::Ptr cloudDownscaled(new SF_CloudNormal);
    SF_CloudNormal::Ptr cloud;
    converter.compute();
    {
      QMutexLocker m1(&*mMutex);
      params._translation = converter.translation();
      cloud = converter.cloudTranslated();
    }
    pcl::VoxelGrid<SF_PointNormal> sor;
    sor.setInputCloud(cloud);
    {
      QMutexLocker m1(&*mMutex);
      sor.setLeafSize(params._voxelSize, params._voxelSize, params._voxelSize);
    }
    sor.filter(*cloudDownscaled);

    SF_CloudNormal::Ptr largestCluster(new SF_CloudNormal());
    pcl::EuclideanClusterExtraction<SF_PointNormal> ec;
    {
      QMutexLocker m1(&*mMutex);
      pcl::search::KdTree<SF_PointNormal>::Ptr tree(new pcl::search::KdTree<SF_PointNormal>);
      tree->setInputCloud(cloudDownscaled);
      ec.setClusterTolerance(params._clusteringDistance);
      ec.setMinClusterSize(10);
      ec.setMaxClusterSize(std::numeric_limits<int>::max());
      ec.setSearchMethod(tree);
      ec.setInputCloud(largestCluster);
    }

    std::vector<pcl::PointIndices> clusterIndices;
    ec.extract(clusterIndices);
    if (clusterIndices.size() > 0) {
      for (std::vector<int>::const_iterator pit = clusterIndices[0].indices.begin(); pit != clusterIndices[0].indices.end(); ++pit)
        largestCluster->points.push_back(cloudDownscaled->points[*pit]);
    }

    Sf_ConverterCTIDToPCLCloud<SF_PointNormal> converterID;
    {
      QMutexLocker m1(&*mMutex);
      converterID.setCloudAndID(cloud, params._ctID);
    }
    converterID.compute();
    std::vector<SF_CloudNormal::Ptr> clusters;

    {
      QMutexLocker m1(&*mMutex);
      clusters = converterID.clusters();
      std::for_each(clusters.begin(), clusters.end(), [&cloud, &params](SF_CloudNormal::Ptr cluster) {
        SF_TransferFeature<SF_PointNormal> tf;
        tf.setInputClouds(cloud, cluster);
        tf.compute();
        pcl::NormalEstimation<SF_PointNormal, SF_PointNormal> ne;
        {
          ne.setInputCloud(cluster);
          pcl::search::KdTree<SF_PointNormal>::Ptr tree(new pcl::search::KdTree<SF_PointNormal>());
          ne.setSearchMethod(tree);
          ne.setRadiusSearch(params._voxelSize * 3);
        }
        ne.compute(*cluster);
      });
      params._clusters = clusters;
    }

    SF_DownHillSimplex sphereFollowing;
    {
      QMutexLocker m1(&*mMutex);
      sphereFollowing.setParams(params);
    }
    sphereFollowing.compute();
    {
      QMutexLocker m1(&*mMutex);
      params = sphereFollowing.params();
      params._cloudIn = largestCluster;

      SF_QSMMedianFilter med;
      med.compute(params._tree);
    }
    SF_VisualizeFitquality vfq;
    {
      QMutexLocker m1(&*mMutex);
      vfq.setCloud(cloud);
      vfq.setParams(params._distanceParams);
      vfq.setQsm(params._tree);
    }
    vfq.compute();
    {
      QMutexLocker m1(&*mMutex);
      params._colors = vfq.colors();
      params._tree->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME);
      params._tree->translate(Eigen::Vector3f(params._translation[0], params._translation[1], params._translation[2]));
    }
  }
};

#endif // SF_STEPSPHEREFOLLOWINGADVANCEDADAPTER_H