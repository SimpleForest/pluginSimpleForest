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

#include "qsm/algorithm/spherefollowing/sf_spherefollowing.h"

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


    {
      QMutexLocker m1(&*mMutex);
      std::vector<SF_CloudNormal::Ptr> clusters;
      clusters.push_back(cloudDownscaled);
      SF_SphereFollowing sphereFollowing(params,clusters);
      std::cout << "FOOOOOFASDASDASDQWE 3" << std::endl;
      std::cout << " Foo Bar Error " << sphereFollowing.error() << std::endl;
      std::cout << " Foo Bar Error " << sphereFollowing.error() << std::endl;
      params._tree = sphereFollowing.getQSM();
      std::cout << " Foo Bar Errorasd " << params._tree->getBuildingBricks().size() << std::endl;
    }

    //        params.log_import();
    //        SF_Statistical_Outlier_Filter<SF_Point> filter;
    //        {
    //            QMutexLocker m1(&*mMutex);
    //            filter.set_cloud_in(params._cloud_in);
    //        }
    //        filter.compute(params);
    //        {
    //            QMutexLocker m1(&*mMutex);
    //            params._output_indices = filter.get_indices();
    //        }
    //        params.log_filter(filter.get_percentage());
  }
};

#endif // SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H
