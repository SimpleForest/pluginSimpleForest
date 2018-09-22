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

#ifndef SF_MODEL_ABSTRACT_BUILDINGBRICK_H
#define SF_MODEL_ABSTRACT_BUILDINGBRICK_H

#include<Eigen/Core>
#include<memory.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/point_types.h>

enum FittingType {UNKNOWN, SPHEREFOLLOWING};

class SF_ModelAbstractSegment;

class Sf_ModelAbstractBuildingbrick
{
protected:
    size_t _ID;
    size_t _indexVector;
    std::weak_ptr<SF_ModelAbstractSegment> _segment;
    Eigen::Vector3f _start;
    Eigen::Vector3f _end;
    FittingType _fittingType;
    virtual float getDistanceToAxis(const Eigen::Vector3f& point) = 0;
    virtual Eigen::Vector3f getProjectionOnAxis(const Eigen::Vector3f& point) = 0;
    virtual float getBoundingSphereRadius();

public:
    Sf_ModelAbstractBuildingbrick();
    std::shared_ptr<Sf_ModelAbstractBuildingbrick> getParent();
    std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick> > getChildren();

    float getGrowthLength();
    float getGrowthVolume();
    virtual float getLength() = 0;
    virtual float getVolume() = 0;
    virtual float getRadius() = 0;
    virtual float getDistance(const Eigen::Vector3f& point) = 0;
    float getDistance(const pcl::PointXYZ &point);
    float getDistance(const pcl::PointXYZINormal &point);
    virtual Eigen::Vector3f getCenter() = 0;
    virtual Eigen::Vector3f getAxis() = 0;
    virtual std::string toString() = 0;
    virtual std::string toHeaderString() = 0;

    size_t getIndex() const;
    void setIndex(const size_t &index);
    size_t getID() const;
    void setID(const size_t &ID);
    Eigen::Vector3f getStart() const;
    Eigen::Vector3f getEnd() const;
    std::shared_ptr<SF_ModelAbstractSegment> getSegment();
    void setSegment(std::shared_ptr<SF_ModelAbstractSegment> segment);
};

#endif // SF_MODEL_ABSTRACT_BUILDINGBRICK_H
