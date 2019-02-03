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

#include "sf_qsmfactory.h"

SF_QSMFactory::SF_QSMFactory() {}

pcl::ModelCoefficients::Ptr
SF_QSMFactory::circle(float x, float y, float z, float rad)
{
  pcl::ModelCoefficients::Ptr circle(new pcl::ModelCoefficients);
  circle->values.push_back(x);
  circle->values.push_back(y);
  circle->values.push_back(z);
  circle->values.push_back(rad);
  return circle;
}

void
SF_QSMFactory::largestGrowthvolumeSecond(std::shared_ptr<SF_ModelQSM> qsm)
{
  std::shared_ptr<SF_ModelAbstractSegment> root(new SF_ModelAbstractSegment(qsm));
  pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 0, 1);
  pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 1, 1);
  std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
  root->addBuildingBrick(cylinder);
  qsm->setRootSegment(root);
  {
    std::shared_ptr<SF_ModelAbstractSegment> childSmallGrowthVolumeLargeRadius(new SF_ModelAbstractSegment(qsm));
    pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 0, 2);
    pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 0.01, 2);
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
    childSmallGrowthVolumeLargeRadius->addBuildingBrick(cylinder);
    root->addChild(childSmallGrowthVolumeLargeRadius);
  }
  {
    std::shared_ptr<SF_ModelAbstractSegment> childLargeGrowthVolumeSmallRadius(new SF_ModelAbstractSegment(qsm));
    pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 0, 0.5);
    pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 100, 0.5);
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
    childLargeGrowthVolumeSmallRadius->addBuildingBrick(cylinder);
    root->addChild(childLargeGrowthVolumeSmallRadius);
  }
}

void
SF_QSMFactory::largestGrowthlengthSecond(std::shared_ptr<SF_ModelQSM> qsm)
{
  std::shared_ptr<SF_ModelAbstractSegment> root(new SF_ModelAbstractSegment(qsm));
  pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 0, 1);
  pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 1, 1);
  std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
  root->addBuildingBrick(cylinder);
  qsm->setRootSegment(root);
  {
    std::shared_ptr<SF_ModelAbstractSegment> childSmallGrowthLengthZeroRadius(new SF_ModelAbstractSegment(qsm));
    pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 0, 0);
    pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 0.01, 0);
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
    childSmallGrowthLengthZeroRadius->addBuildingBrick(cylinder);
    root->addChild(childSmallGrowthLengthZeroRadius);
  }
  {
    std::shared_ptr<SF_ModelAbstractSegment> childLargeGrowthLengthZeroRadius(new SF_ModelAbstractSegment(qsm));
    pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 0, 0);
    pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 100, 0);
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
    childLargeGrowthLengthZeroRadius->addBuildingBrick(cylinder);
    root->addChild(childLargeGrowthLengthZeroRadius);
  }
}

void
SF_QSMFactory::largestRadiusSecond(std::shared_ptr<SF_ModelQSM> qsm)
{
  std::shared_ptr<SF_ModelAbstractSegment> root(new SF_ModelAbstractSegment(qsm));
  pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 0, 1);
  pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 1, 1);
  std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
  root->addBuildingBrick(cylinder);
  qsm->setRootSegment(root);
  {
    std::shared_ptr<SF_ModelAbstractSegment> childSmallRadius(new SF_ModelAbstractSegment(qsm));
    pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 0, 1);
    pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 0, 1);
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
    childSmallRadius->addBuildingBrick(cylinder);
    root->addChild(childSmallRadius);
  }
  {
    std::shared_ptr<SF_ModelAbstractSegment> childLargeRadius(new SF_ModelAbstractSegment(qsm));
    pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 0, 2);
    pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 0, 2);
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
    childLargeRadius->addBuildingBrick(cylinder);
    root->addChild(childLargeRadius);
  }
}

void
SF_QSMFactory::largestAngleSecond(std::shared_ptr<SF_ModelQSM> qsm)
{
  std::shared_ptr<SF_ModelAbstractSegment> root(new SF_ModelAbstractSegment(qsm));
  pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 0, 1);
  pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 1, 1);
  std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
  root->addBuildingBrick(cylinder);
  qsm->setRootSegment(root);
  {
    std::shared_ptr<SF_ModelAbstractSegment> childSmallAngle(new SF_ModelAbstractSegment(qsm));
    pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 1, 1);
    pcl::ModelCoefficients::Ptr circleb = circle(0, 1, 1, 1);
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
    childSmallAngle->addBuildingBrick(cylinder);
    root->addChild(childSmallAngle);
  }
  {
    std::shared_ptr<SF_ModelAbstractSegment> childLargeAngle(new SF_ModelAbstractSegment(qsm));
    pcl::ModelCoefficients::Ptr circlea = circle(0, 0, 1, 1);
    pcl::ModelCoefficients::Ptr circleb = circle(0, 0, 2, 1);
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(circlea, circleb));
    childLargeAngle->addBuildingBrick(cylinder);
    root->addChild(childLargeAngle);
  }
}

std::shared_ptr<SF_ModelQSM>
SF_QSMFactory::qsm(SORTTYPE type)
{
  std::shared_ptr<SF_ModelQSM> qsm(new SF_ModelQSM(1));
  switch (type) {
    case SORTTYPE::LARGEST_GROWTHVOLUME_SECOND:
      largestGrowthvolumeSecond(qsm);
      break;
    case SORTTYPE::LARGEST_GROWTHLENGTH_SECOND:
      largestGrowthlengthSecond(qsm);
      break;
    case SORTTYPE::LARGEST_RADIUS_SECOND:
      largestRadiusSecond(qsm);
      break;
    case SORTTYPE::LARGEST_ANGLE_SECOND:
      largestAngleSecond(qsm);
      break;
    default:
      throw("SF_QSMFactory: Unkown SORTTYPE.");
      break;
  }
  return qsm;
}
