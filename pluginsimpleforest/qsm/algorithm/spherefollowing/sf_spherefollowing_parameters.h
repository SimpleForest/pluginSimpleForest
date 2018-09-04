#ifndef SF_SPHEREFOLLOWING_PARAMETERS_H
#define SF_SPHEREFOLLOWING_PARAMETERS_H

#include <vector>

struct SF_Sphere_Following_Parameters_Cluster {
    float _euclideanDistance = 0.03;
    float _sphereRadiusMultiplier = 2;
    float _minRadius = 0.07;
    float _sphereEpsilon = 0.035;
    float _ransacCircleInlierDistance = 0.03;
    int _minPtsCircle = 3;
    float _heightStartSphere = 0.05;
    float _voxelSize = 0.01;
    SF_Sphere_Following_Parameters_Cluster() {}
};

struct SF_Sphere_Following_Parameters {
    std::vector<SF_Sphere_Following_Parameters_Cluster> _paramVec;
    SF_Sphere_Following_Parameters() {}
};

#endif // SF_SPHEREFOLLOWING_PARAMETERS_H
