#ifndef SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H
#define SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H

#include "steps/param/sf_paramAllSteps.h"
#include <converters/CT_To_PCL/sf_converterCTToPCL.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <QThreadPool>

#include "qsm/algorithm/sf_QSMAlgorithm.h"
#include "qsm/algorithm/sf_QSMCylinder.h"
#include "qsm/algorithm/sf_buildQSM.h"
#include <pcl/ModelCoefficients.h>

#include "qsm/algorithm/spherefollowing/sf_spherefollowingrastersearch.h"

class SF_SpherefollowingRootAdapter {
public:
  std::shared_ptr<QMutex> mMutex;

  SF_SpherefollowingRootAdapter(const SF_SpherefollowingRootAdapter &obj) {
      QThreadPool::globalInstance()->setMaxThreadCount(1);
    mMutex = obj.mMutex;
  }

  SF_SpherefollowingRootAdapter() {
      QThreadPool::globalInstance()->setMaxThreadCount(1);
      mMutex.reset(new QMutex);
  }

  ~SF_SpherefollowingRootAdapter() {}

  void operator()(SF_ParamSpherefollowingBasic<SF_PointNormal> &params) {
    Sf_ConverterCTToPCL<SF_PointNormal> converter;
    {
      QMutexLocker m1(&*mMutex);
      converter.setItemCpyCloudInDeprecated(params._itemCpyCloudIn);
    }
    SF_CloudNormal::Ptr cloud;
    SF_CloudNormal::Ptr cloudDownscaled(new SF_CloudNormal());
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
    pcl::NormalEstimation<SF_PointNormal, SF_PointNormal> ne;
    {
      QMutexLocker m1(&*mMutex);

      ne.setInputCloud(cloudDownscaled);
      pcl::search::KdTree<SF_PointNormal>::Ptr tree(
          new pcl::search::KdTree<SF_PointNormal>());
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(params._voxelSize * 3);
    }
    ne.compute(*cloudDownscaled);

    SF_SphereFollowingRasterSearch sphereFollowing;
    {
      QMutexLocker m1(&*mMutex);
      std::cout << "foo1" << std::endl;
      sphereFollowing.setParams(params);
      sphereFollowing.setCloud(cloudDownscaled);
      std::cout << "foo2" << std::endl;
    }

    sphereFollowing.compute();
    {
      QMutexLocker m1(&*mMutex);
      params = sphereFollowing.getParamVec()[0];
      std::cout << "foo3 " << sphereFollowing.getParamVec().size() << " ; " << sphereFollowing.getParamVec()[0]._modelCloudError<< std::endl;
      std::cout << "foo3 " << sphereFollowing.getParamVec().size() << " ; " << sphereFollowing.getParamVec()[39]._modelCloudError<< std::endl;
      std::cout << "foo3 " << sphereFollowing.getParamVec().size() << " ; " << sphereFollowing.getParamVec()[79]._modelCloudError<< std::endl;
    }
  }
};

#endif // SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H
