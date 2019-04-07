#include "sf_spherefollowingrastersearch.h"

SF_SphereFollowingRasterSearch::SF_SphereFollowingRasterSearch() {}

void
SF_SphereFollowingRasterSearch::compute()
{
  SF_Descriptor<SF_PointNormal> desc(m_cloud);
  desc.setParameters(2);
  desc.computeFeatures();
  m_params._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistance = 3 * (desc.mean() + 2 * desc.sd());
  m_params._sphereFollowingParams.m_optimizationParams[0]._epsilonSphere = 3 * (desc.mean() + 2 * desc.sd());
  std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal>> paramVec = paramVector();
  std::for_each(paramVec.begin(), paramVec.end(), [this](SF_ParamSpherefollowingBasic<SF_PointNormal>& params) {
    SF_SphereFollowing sphereFollowing;
    std::vector<SF_CloudNormal::Ptr> clusters;
    clusters.push_back(m_cloud);
    sphereFollowing.setParams(params);
    sphereFollowing.setClusters(clusters);
    try {
      sphereFollowing.compute();
      params._tree = sphereFollowing.getQSM();
      params._modelCloudError = sphereFollowing.error();
    } catch (...) {
      params._tree = sphereFollowing.getQSM();
      params._modelCloudError = std::numeric_limits<float>::max();
    }
  });
  std::sort(paramVec.begin(),
            paramVec.end(),
            [](SF_ParamSpherefollowingBasic<SF_PointNormal> params1, SF_ParamSpherefollowingBasic<SF_PointNormal> params2) {
              return params1._modelCloudError < params2._modelCloudError;
            });
  m_paramVec = paramVec;
}

void
SF_SphereFollowingRasterSearch::setParams(const SF_ParamSpherefollowingBasic<SF_PointNormal>& params)
{
  m_params = params;
}

void
SF_SphereFollowingRasterSearch::setCloud(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud)
{
  m_cloud = cloud;
}

std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal>>
SF_SphereFollowingRasterSearch::getParamVec() const
{
  return m_paramVec;
}

std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal>>
SF_SphereFollowingRasterSearch::paramVector()
{
  std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal>> paramVec;
  auto sphereRadiusMultiplierMultipliers = m_params._sphereFollowingParams.m_optimizationParams[0]._sphereRadiusMultiplierMultiplier;
  for (auto factorSphereRadius : sphereRadiusMultiplierMultipliers) {
    SF_ParamSpherefollowingBasic<SF_PointNormal> paramsA = m_params;
    paramsA._sphereFollowingParams.m_optimizationParams[0]._sphereRadiusMultiplier *= factorSphereRadius;
    auto euclideanClusterMultipliers = m_params._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistanceMultiplier;
    for (auto factorEuclideanCluster : euclideanClusterMultipliers) {
      SF_ParamSpherefollowingBasic<SF_PointNormal> paramsB = paramsA;
      paramsB._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistance *= factorEuclideanCluster;
      auto epsilonSphereMultipliers = m_params._sphereFollowingParams.m_optimizationParams[0]._epsilonSphereMultiplier;
      for (auto factorEpsilonSphere : epsilonSphereMultipliers) {
        SF_ParamSpherefollowingBasic<SF_PointNormal> paramsC = paramsB;
        paramsC._sphereFollowingParams.m_optimizationParams[0]._epsilonSphere *= factorEpsilonSphere;
        paramVec.push_back(paramsC);
      }
    }
  }
  return paramVec;
}
