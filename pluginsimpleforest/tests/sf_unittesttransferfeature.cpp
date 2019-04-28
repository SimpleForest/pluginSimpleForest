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

#include "sf_unittesttransferfeature.h"

SF_CloudNormal::Ptr
SF_UnitTestTransferFeature::createSmallCloud()
{
  SF_CloudNormal::Ptr cloud(new SF_CloudNormal());
  {
    SF_PointNormal point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    point.intensity = 1;
    point.normal_x = 0;
    point.normal_y = 0;
    point.normal_z = 1;
    cloud->push_back(point);
  }
  {
    SF_PointNormal point;
    point.x = 1;
    point.y = 1;
    point.z = 1;
    point.intensity = 2;
    point.normal_x = 0;
    point.normal_y = 0;
    point.normal_z = 1;
    cloud->push_back(point);
  }
  {
    SF_PointNormal point;
    point.x = 2;
    point.y = 2;
    point.z = 2;
    point.intensity = 3;
    point.normal_x = 0;
    point.normal_y = 0;
    point.normal_z = 1;
    cloud->push_back(point);
  }
  return cloud;
}

SF_CloudNormal::Ptr
SF_UnitTestTransferFeature::createLargeCloud()
{
  SF_CloudNormal::Ptr cloud(new SF_CloudNormal());
  {
    SF_PointNormal point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    point.intensity = 0;
    point.normal_x = 0;
    point.normal_y = 1;
    point.normal_z = 1;
    cloud->push_back(point);
  }
  {
    SF_PointNormal point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    point.intensity = 0;
    point.normal_x = 0;
    point.normal_y = 1;
    point.normal_z = 1;
    cloud->push_back(point);
  }
  {
    SF_PointNormal point;
    point.x = 1;
    point.y = 1;
    point.z = 1;
    point.intensity = 0;
    point.normal_x = 0;
    point.normal_y = 1;
    point.normal_z = 1;
    cloud->push_back(point);
  }
  {
    SF_PointNormal point;
    point.x = 1;
    point.y = 1;
    point.z = 1;
    point.intensity = 0;
    point.normal_x = 0;
    point.normal_y = 1;
    point.normal_z = 1;
    cloud->push_back(point);
  }
  {
    SF_PointNormal point;
    point.x = 2;
    point.y = 2;
    point.z = 2;
    point.intensity = 0;
    point.normal_x = 0;
    point.normal_y = 1;
    point.normal_z = 1;
    cloud->push_back(point);
  }
  {
    SF_PointNormal point;
    point.x = 2;
    point.y = 2;
    point.z = 2;
    point.intensity = 0;
    point.normal_x = 0;
    point.normal_y = 1;
    point.normal_z = 1;
    cloud->push_back(point);
  }
  return cloud;
}

void
SF_UnitTestTransferFeature::transferUpscalingWorks()
{
  SF_CloudNormal::Ptr source = createSmallCloud();
  SF_CloudNormal::Ptr target = createLargeCloud();

  SF_TransferFeature<SF_PointNormal> tf;
  tf.setInputClouds(source, target);
  tf.compute();
  std::for_each(target->points.begin(), target->points.end(), [](const SF_PointNormal& point) {
    QVERIFY(point.normal_x == 0);
    QVERIFY(point.normal_y == 0);
    QVERIFY(point.normal_z == 1);
  });
  QVERIFY(target->points[0].intensity == 1);
  QVERIFY(target->points[1].intensity == 1);
  QVERIFY(target->points[2].intensity == 2);
  QVERIFY(target->points[3].intensity == 2);
  QVERIFY(target->points[4].intensity == 3);
  QVERIFY(target->points[5].intensity == 3);
}

void
SF_UnitTestTransferFeature::transferDownscalingWorks()
{
  SF_CloudNormal::Ptr source = createLargeCloud();
  SF_CloudNormal::Ptr target = createSmallCloud();

  SF_TransferFeature<SF_PointNormal> tf;
  tf.setInputClouds(source, target);
  tf.compute();
  std::for_each(target->points.begin(), target->points.end(), [](const SF_PointNormal& point) {
    QVERIFY(point.normal_x == 0);
    QVERIFY(point.normal_y == 1);
    QVERIFY(point.normal_z == 1);
  });
  QVERIFY(target->points[0].intensity == 0);
  QVERIFY(target->points[1].intensity == 0);
  QVERIFY(target->points[2].intensity == 0);
}
