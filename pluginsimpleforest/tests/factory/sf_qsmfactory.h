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

#ifndef SF_QSMFACTORY_H
#define SF_QSMFACTORY_H

#include "qsm/model/sf_modelCylinderBuildingbrick.h"
#include "qsm/model/sf_modelQSM.h"

enum SORTTYPE
{
  LARGEST_GROWTHVOLUME_SECOND,
  LARGEST_GROWTHLENGTH_SECOND,
  LARGEST_RADIUS_SECOND,
  LARGEST_ANGLE_SECOND
};
class SF_QSMFactory
{
  SF_QSMFactory();
  static pcl::ModelCoefficients::Ptr circle(float x = 0.0f, float y = 0.0f, float z = 0.0f, float rad = 1.0f);
  static void largestGrowthvolumeSecond(std::shared_ptr<SF_ModelQSM> qsm);
  static void largestGrowthlengthSecond(std::shared_ptr<SF_ModelQSM> qsm);
  static void largestRadiusSecond(std::shared_ptr<SF_ModelQSM> qsm);
  static void largestAngleSecond(std::shared_ptr<SF_ModelQSM> qsm);

public:
  static std::shared_ptr<SF_ModelQSM> qsm(SORTTYPE type);
};

#endif // SF_QSMFACTORY_H
