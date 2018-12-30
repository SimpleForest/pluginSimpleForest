#ifndef SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H
#define SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H

#include "steps/param/sf_paramAllSteps.h"
#include <QThreadPool>
#include <converters/CT_To_PCL/sf_converterCTToPCL.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

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
    SF_CloudNormal::Ptr largestCluster(new SF_CloudNormal());
    pcl::EuclideanClusterExtraction<SF_PointNormal> ec;
    {
      QMutexLocker m1(&*mMutex);
      pcl::search::KdTree<SF_PointNormal>::Ptr tree(
          new pcl::search::KdTree<SF_PointNormal>);
      tree->setInputCloud(cloudDownscaled);
      ec.setClusterTolerance(params._clusteringDistance);
      ec.setMinClusterSize(10);
      ec.setMaxClusterSize(std::numeric_limits<int>::max());
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloudDownscaled);
    }

    std::vector<pcl::PointIndices> clusterIndices;
    ec.extract(clusterIndices);
    if (clusterIndices.size() > 0) {
      for (std::vector<int>::const_iterator pit =
               clusterIndices[0].indices.begin();
           pit != clusterIndices[0].indices.end(); ++pit)
        largestCluster->points.push_back(cloudDownscaled->points[*pit]);
    }

    SF_SphereFollowingRasterSearch sphereFollowing;
    {
      QMutexLocker m1(&*mMutex);
      sphereFollowing.setParams(params);
      sphereFollowing.setCloud(largestCluster);
    }

    sphereFollowing.compute();
    {
      QMutexLocker m1(&*mMutex);
      params = sphereFollowing.getParamVec()[0];
      std::cout << "foo3 " << sphereFollowing.getParamVec().size() << " ; "
                << sphereFollowing.getParamVec()[0]._modelCloudError
                << std::endl;
      std::cout << "foo3 " << sphereFollowing.getParamVec().size() << " ; "
                << sphereFollowing.getParamVec()[39]._modelCloudError
                << std::endl;
      std::cout << "foo3 " << sphereFollowing.getParamVec().size() << " ; "
                << sphereFollowing.getParamVec()[79]._modelCloudError
                << std::endl;
    }
  }
};

#endif // SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H
