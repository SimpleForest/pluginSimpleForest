/****************************************************************************

 Copyright (C) 2017-2018 Jan Hackenberg, free software developer
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

#ifndef SF_MODEL_CYLINDER_BUILDINGBRICK_H
#define SF_MODEL_CYLINDER_BUILDINGBRICK_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include "sf_model_abstract_buildingbrick.h"


class SF_Model_Cylinder_Buildingbrick: public SF_Model_Abstract_Buildingbrick {
    float _radius;
protected:
    virtual float getRadius();
    virtual float getVolume();
    virtual float getLength();
    virtual Eigen::Vector3f getCenter();
    virtual Eigen::Vector3f getPrincipleDirection();
public:
    SF_Model_Cylinder_Buildingbrick(pcl::ModelCoefficients::Ptr circleA, pcl::ModelCoefficients::Ptr circleB);
    virtual std::string toString();
    virtual std::string toHeaderString();
};

#endif // SF_MODEL_CYLINDER_BUILDINGBRICK_H
