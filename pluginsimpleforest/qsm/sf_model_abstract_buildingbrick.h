#ifndef SF_MODEL_ABSTRACT_BUILDINGBRICK_H
#define SF_MODEL_ABSTRACT_BUILDINGBRICK_H

#include<Eigen/Core>

class SF_Model_Abstract_Buildingbrick
{
public:
    SF_Model_Abstract_Buildingbrick();

    virtual float get_length() = 0;

    virtual float get_volume() = 0;

    virtual Eigen::Vector3f get_principle_direction() = 0;
};

#endif // SF_MODEL_ABSTRACT_BUILDINGBRICK_H
