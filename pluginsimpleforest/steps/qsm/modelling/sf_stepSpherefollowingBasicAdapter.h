#ifndef SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H
#define SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H

#include <QThreadPool>
#include <converters/CT_To_PCL/sf_converterCTToPCL.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "qsm/algorithm/optimization/downHillSimplex/sf_downhillsimplex.h"
#include "qsm/algorithm/optimization/gridsearch/sf_spherefollowingrastersearch.h"
#include "qsm/algorithm/postprocessing/sf_qsmmedianfilter.h"
#include "qsm/algorithm/sf_QSMAlgorithm.h"
#include "qsm/algorithm/sf_QSMCylinder.h"
#include "qsm/algorithm/visualization/sf_visualizefitquality.h"
#include "qsm/build/sf_buildQSM.h"
#include "steps/param/sf_paramAllSteps.h"
#include "steps/visualization/sf_colorfactory.h"

class SF_SpherefollowingRootAdapter
{
public:
  std::shared_ptr<QMutex> mMutex;

  SF_SpherefollowingRootAdapter(const SF_SpherefollowingRootAdapter& obj)
  {
    //        QThreadPool::globalInstance()->setMaxThreadCount(1);
    mMutex = obj.mMutex;
  }

  SF_SpherefollowingRootAdapter()
  {
    //        QThreadPool::globalInstance()->setMaxThreadCount(1);
    mMutex.reset(new QMutex);
  }

  ~SF_SpherefollowingRootAdapter() {}

  void operator()(SF_ParamSpherefollowingBasic<SF_PointNormal>& params)
  {
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
      pcl::search::KdTree<SF_PointNormal>::Ptr tree(new pcl::search::KdTree<SF_PointNormal>());
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(params._voxelSize * 3);
    }
    ne.compute(*cloudDownscaled);
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
      ec.setInputCloud(cloudDownscaled);
    }

    std::vector<pcl::PointIndices> clusterIndices;
    ec.extract(clusterIndices);
    if (clusterIndices.size() > 0) {
      for (std::vector<int>::const_iterator pit = clusterIndices[0].indices.begin(); pit != clusterIndices[0].indices.end(); ++pit)
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
      params._tree->translate(Eigen::Vector3f(params._translation[0], params._translation[1], params._translation[2]));
    }
  }
};

#endif // SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H
