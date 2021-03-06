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

#ifndef SF_STEPSPHEREFOLLOWINGRECURSIVEADAPTER_H
#define SF_STEPSPHEREFOLLOWINGRECURSIVEADAPTER_H

#include <QThreadPool>
#include <converters/CT_To_PCL/sf_converterCTToPCL.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "qsm/algorithm/optimization/gridsearch/sf_spherefollowingrastersearch.h"
#include "qsm/algorithm/optimization/recursion/sf_spherefollowingRecursive.h"
#include "qsm/algorithm/sf_QSMAlgorithm.h"
#include "qsm/algorithm/sf_QSMCylinder.h"
#include "qsm/algorithm/visualization/sf_visualizefitquality.h"
#include "qsm/build/sf_buildQSM.h"
#include "steps/param/sf_paramAllSteps.h"
#include "steps/visualization/sf_colorfactory.h"

class SF_SpherefollowingRecursiveAdapter
{
public:
  std::shared_ptr<QMutex> mMutex;

  SF_SpherefollowingRecursiveAdapter(const SF_SpherefollowingRecursiveAdapter& obj) { mMutex = obj.mMutex; }

  SF_SpherefollowingRecursiveAdapter() { mMutex.reset(new QMutex); }

  ~SF_SpherefollowingRecursiveAdapter() {}

  void operator()(SF_ParamSpherefollowingRecursive<SF_PointNormal>& params)
  {
    Sf_ConverterCTToPCL<SF_PointNormal> converter;
    {
      QMutexLocker m1(&*mMutex);
      converter.setItemCpyCloudInDeprecated(params._itemCpyCloudIn);
    }
    SF_CloudNormal::Ptr cloud;
    SF_CloudNormal::Ptr cloudDownscaled(new SF_CloudNormal());
    std::shared_ptr<SF_ModelQSM> qsmCpy;
    converter.compute();
    {
      QMutexLocker m1(&*mMutex);
      params._translation = converter.translation();
      params._qsm->translate(-params._translation);
      qsmCpy = params._qsm;
      cloud = converter.cloudTranslated();
    }
    pcl::VoxelGrid<SF_PointNormal> sor;
    sor.setInputCloud(cloud);
    {
      QMutexLocker m1(&*mMutex);
      sor.setLeafSize(params._voxelSize, params._voxelSize, params._voxelSize);
    }
    sor.filter(*cloudDownscaled);
    pcl::NormalEstimation<SF_PointNormal, SF_PointNormal> ne;
    {
      QMutexLocker m1(&*mMutex);

      ne.setInputCloud(cloudDownscaled);
      pcl::search::KdTree<SF_PointNormal>::Ptr tree(new pcl::search::KdTree<SF_PointNormal>());
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(params._voxelSize * 3);
    }
    ne.compute(*cloudDownscaled);
    params._stepProgress->fireComputation();

    SF_SpherefollowingRecursive recursion;
    {
      QMutexLocker m1(&*mMutex);
      recursion.setParams(params);
      recursion.setCloud(cloudDownscaled);
      recursion.setQsm(params._qsm);
    }
    try {
      recursion.compute();
      {
        QMutexLocker m1(&*mMutex);
        params._qsm = recursion.getQsm();
        if (!params._qsm) {
          params._qsm = qsmCpy;
        }
        params._qsm->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME, 0.0001);
        params._qsm->translate(params._translation);
        params._qsm->setTranslation(Eigen::Vector3d(0, 0, 0));
      }
    } catch (...) {
      QMutexLocker m1(&*mMutex);
      params._qsm = qsmCpy;
      params._qsm->translate(params._translation);
    }
  }
};

#endif // SF_STEPSPHEREFOLLOWINGRECURSIVEADAPTER_H
