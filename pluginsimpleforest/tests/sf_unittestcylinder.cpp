/****************************************************************************

 Copyright (C) 2017-2019 Dr. Jan Hackenberg, free software developer
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

#include "tests/sf_unittestcylinder.h"

void
SF_UnitTestCylinder::cylinderConstructor()
{
    pcl::ModelCoefficients::Ptr circleA (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr circleB (new pcl::ModelCoefficients);
    circleA->values.push_back(-1);
    circleA->values.push_back(0);
    circleA->values.push_back(2);
    circleA->values.push_back(4);

    circleB->values.push_back(1);
    circleB->values.push_back(0);
    circleB->values.push_back(2);
    circleB->values.push_back(1);
    Sf_ModelCylinderBuildingbrick cylinder{circleA, circleB};
    auto start = cylinder.getStart();
    auto end = cylinder.getEnd();
    auto radius = cylinder.getRadius();

    // assure only second cylinder radius is applied
    QVERIFY(radius == 1);
    QVERIFY(start == Eigen::Vector3f(-1,0,2));
    QVERIFY(end == Eigen::Vector3f(1,0,2));
}

void
SF_UnitTestCylinder::lengthAndVolume()
{
    pcl::ModelCoefficients::Ptr circleA (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr circleB (new pcl::ModelCoefficients);
    circleA->values.push_back(-1);
    circleA->values.push_back(0);
    circleA->values.push_back(2);
    circleA->values.push_back(4);

    circleB->values.push_back(1);
    circleB->values.push_back(0);
    circleB->values.push_back(2);
    circleB->values.push_back(1);
    Sf_ModelCylinderBuildingbrick cylinder{circleA, circleB};
    auto length = cylinder.getLength();
    auto volume = cylinder.getVolume();

    QVERIFY(length == 2);
    // radius is 1, so is squared radius
    QVERIFY(volume == 2*SF_Math<float>::_PI);
}

void
SF_UnitTestCylinder::distance()
{
    pcl::ModelCoefficients::Ptr circleA (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr circleB (new pcl::ModelCoefficients);
    circleA->values.push_back(-1);
    circleA->values.push_back(0);
    circleA->values.push_back(2);
    circleA->values.push_back(4);

    circleB->values.push_back(1);
    circleB->values.push_back(0);
    circleB->values.push_back(2);
    circleB->values.push_back(1);
    Sf_ModelCylinderBuildingbrick cylinder{circleA, circleB};
    {
        Eigen::Vector3f point (0,0,0);
        auto distance = cylinder.getDistance(point);
        QVERIFY(distance == 1);
    }
    {
        Eigen::Vector3f point (-2,0,0);
        auto distance = cylinder.getDistance(point);
        QVERIFY(distance == std::sqrt(2.0f));
    }
    {
        Eigen::Vector3f point (2,0,0);
        auto distance = cylinder.getDistance(point);
        QVERIFY(distance == std::sqrt(2.0f));
    }
}

void
SF_UnitTestCylinder::distanceToAxis()
{
    pcl::ModelCoefficients::Ptr circleA (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr circleB (new pcl::ModelCoefficients);
    circleA->values.push_back(-1);
    circleA->values.push_back(0);
    circleA->values.push_back(2);
    circleA->values.push_back(4);

    circleB->values.push_back(1);
    circleB->values.push_back(0);
    circleB->values.push_back(2);
    circleB->values.push_back(1);
    Sf_ModelCylinderBuildingbrick cylinder{circleA, circleB};
    Eigen::Vector3f point (0,0,0);
    auto distance = cylinder.getDistanceToAxis(point);
    QVERIFY(distance == 2);
}

void SF_UnitTestCylinder::distanceToInfinitHull()
{
    pcl::ModelCoefficients::Ptr circleA (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr circleB (new pcl::ModelCoefficients);
    circleA->values.push_back(-1);
    circleA->values.push_back(0);
    circleA->values.push_back(2);
    circleA->values.push_back(4);

    circleB->values.push_back(1);
    circleB->values.push_back(0);
    circleB->values.push_back(2);
    circleB->values.push_back(1);
    Sf_ModelCylinderBuildingbrick cylinder{circleA, circleB};
    Eigen::Vector3f point (0,0,0);
    auto distance = cylinder.getDistanceToInfinitHull(point);
    QVERIFY(distance == 1);
}

void SF_UnitTestCylinder::projectionOnAxis()
{
    pcl::ModelCoefficients::Ptr circleA (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr circleB (new pcl::ModelCoefficients);
    circleA->values.push_back(-1);
    circleA->values.push_back(0);
    circleA->values.push_back(2);
    circleA->values.push_back(4);

    circleB->values.push_back(1);
    circleB->values.push_back(0);
    circleB->values.push_back(2);
    circleB->values.push_back(1);
    Sf_ModelCylinderBuildingbrick cylinder{circleA, circleB};
    Eigen::Vector3f point (0,0,0);
    auto projection = cylinder.getProjectionOnAxis(point);
    QVERIFY(projection == Eigen::Vector3f(0,0,2));
    cylinder.getProjectedDistanceToSegment(point);
}

void SF_UnitTestCylinder::projectedDistanceToSegment()
{
    pcl::ModelCoefficients::Ptr circleA (new pcl::ModelCoefficients);
     pcl::ModelCoefficients::Ptr circleB (new pcl::ModelCoefficients);
     circleA->values.push_back(-1);
     circleA->values.push_back(0);
     circleA->values.push_back(2);
     circleA->values.push_back(4);

     circleB->values.push_back(1);
     circleB->values.push_back(0);
     circleB->values.push_back(2);
     circleB->values.push_back(1);
     Sf_ModelCylinderBuildingbrick cylinder{circleA, circleB};
     {
         Eigen::Vector3f point (0,0,0);
         auto distance = cylinder.getProjectedDistanceToSegment(point);
         QVERIFY(distance == 0); ;
     }
     {
         Eigen::Vector3f point (-10,0,0);
         auto distance = cylinder.getProjectedDistanceToSegment(point);
         QVERIFY(distance == 9); ;
     }
     {
         Eigen::Vector3f point (10,0,0);
         auto distance = cylinder.getProjectedDistanceToSegment(point);
         QVERIFY(distance == 9); ;
     }

}
