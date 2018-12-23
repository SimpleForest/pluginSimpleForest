#include "sf_spherefollowingrastersearch.h"

SF_SphereFollowingRasterSearch::SF_SphereFollowingRasterSearch()
{

}

void SF_SphereFollowingRasterSearch::compute()
{
    SF_Descriptor<SF_PointNormal> desc(m_cloud);
    desc.setParameters(2);
    desc.computeFeatures();
    m_params._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistance = 2*(desc.mean() + 2*desc.sd());
    m_params._sphereFollowingParams.m_optimizationParams[0]._epsilonSphere = (desc.mean() + 2*desc.sd());
    std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal> > paramVec = paramVector();
    size_t index = 0;
    std::for_each(paramVec.begin(), paramVec.end(), [this, &index](SF_ParamSpherefollowingBasic<SF_PointNormal> &params){
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
    std::sort(paramVec.begin(), paramVec.end(), [](SF_ParamSpherefollowingBasic<SF_PointNormal> params1, SF_ParamSpherefollowingBasic<SF_PointNormal> params2) {
        return params1._modelCloudError < params2._modelCloudError;
    });
    m_paramVec = paramVec;
}

void SF_SphereFollowingRasterSearch::setParams(const SF_ParamSpherefollowingBasic<SF_PointNormal> &params)
{
    m_params = params;
}

void SF_SphereFollowingRasterSearch::setCloud(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud)
{
    m_cloud = cloud;
}

std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal> > SF_SphereFollowingRasterSearch::getParamVec() const
{
    return m_paramVec;
}


std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal> > SF_SphereFollowingRasterSearch::paramVector()
{
    std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal> > paramVec;
    for(size_t a = 0; a < 5; a++)
    {
        SF_ParamSpherefollowingBasic<SF_PointNormal> paramsA = m_params;
        switch (a) {
        case 0:
            paramsA._sphereFollowingParams.m_optimizationParams[0]._sphereRadiusMultiplier = 1.75;
            break;
        case 1:
            paramsA._sphereFollowingParams.m_optimizationParams[0]._sphereRadiusMultiplier = 2.25;
            break;
        case 2:
            paramsA._sphereFollowingParams.m_optimizationParams[0]._sphereRadiusMultiplier = 2.75;
            break;
        case 3:
            paramsA._sphereFollowingParams.m_optimizationParams[0]._sphereRadiusMultiplier = 3.25;
            break;
        case 4:
            paramsA._sphereFollowingParams.m_optimizationParams[0]._sphereRadiusMultiplier = 4;
            break;
        default:
            break;
        }
        for(size_t b = 0; b < 4; b++)
        {
            SF_ParamSpherefollowingBasic<SF_PointNormal> paramsB = paramsA;
            switch (b) {
            case 0:
                paramsB._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistance *=0.5;
                break;
            case 1:
                paramsB._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistance *=1;
                break;
            case 2:
                paramsB._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistance *=4;
                break;
            case 3:
                paramsB._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistance *=8;
                break;
            default:
                break;
            }
            for(size_t c = 0; c < 4 ; c++)
            {
                SF_ParamSpherefollowingBasic<SF_PointNormal> paramsC = paramsB;
                switch (c) {
                case 0:
                    paramsC._sphereFollowingParams.m_optimizationParams[0]._epsilonSphere *= 0.5;
                    break;
                case 1:
                    paramsC._sphereFollowingParams.m_optimizationParams[0]._epsilonSphere *= 1;
                    break;
                case 2:
                    paramsC._sphereFollowingParams.m_optimizationParams[0]._epsilonSphere *= 2;
                    break;
                case 3:
                    paramsC._sphereFollowingParams.m_optimizationParams[0]._epsilonSphere *= 3;
                    break;
                default:
                    break;
                }
                paramVec.push_back(paramsC);
            }

        }
    }
    return paramVec;
}
