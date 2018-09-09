#ifndef SF_SPHEREFOLLOWING_PARAMETERS_H
#define SF_SPHEREFOLLOWING_PARAMETERS_H

#include <vector>
#include <algorithm>
#include <pcl/segmentation/sac_segmentation.h>

struct SfSphereFollowingParametersCluster {
    float _euclideanDistance          = 0.03;
    float _sphereRadiusMultiplier     = 2;
    float _epsilonSphere              = 0.035;
    float _heightStartSphere          = 0.05;
    int   _minPtsCircle               = 3;
    float _minRadius                  = 0.07;
    int   _ransacIterations           = 100;
    int   _fittingMethod              = pcl::SAC_RANSAC;
    SfSphereFollowingParametersCluster(float euclideanDistance,
                                       float sphereRadiusMultiplier,
                                       float epsilonSphere,
                                       int minPtsCircle):
        _euclideanDistance(euclideanDistance),_sphereRadiusMultiplier(sphereRadiusMultiplier), _epsilonSphere(epsilonSphere), _minPtsCircle(std::max(3,minPtsCircle)) {
        _heightStartSphere = 2*epsilonSphere;
    }
};

struct SfSphereFollowingParameters {
    std::vector<SfSphereFollowingParametersCluster> _paramVec;
    float _voxelSize                  = 0.01;
    float _ransacCircleInlierDistance = 0.03;
    SfSphereFollowingParameters() {}
};

#endif // SF_SPHEREFOLLOWING_PARAMETERS_H
