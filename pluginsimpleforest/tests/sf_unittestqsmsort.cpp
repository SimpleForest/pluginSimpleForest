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

#ifndef SF_UNIT_TEST_QSM_SORT_H
#define SF_UNIT_TEST_QSM_SORT_H

#include "tests/factory/sf_qsmfactory.h"
#include <QtTest/QtTest>

class SF_UnitTestQSMSort : public QObject
{
  Q_OBJECT
private slots:
  void growthVolumeSort();
  void growthLengthSort();
  void angleSort();
  void radiusSort();
};

void
SF_UnitTestQSMSort::growthVolumeSort()
{
  SORTTYPE type = SORTTYPE::LARGEST_GROWTHVOLUME_SECOND;
  std::shared_ptr<SF_ModelQSM> growthVolumeQSM = SF_QSMFactory::qsm(type);
  QVERIFY(growthVolumeQSM->getRootSegment()->getChildren()[0]->getGrowthVolume() <
          growthVolumeQSM->getRootSegment()->getChildren()[1]->getGrowthVolume());
  growthVolumeQSM->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME);
  QVERIFY(growthVolumeQSM->getRootSegment()->getChildren()[0]->getGrowthVolume() >
          growthVolumeQSM->getRootSegment()->getChildren()[1]->getGrowthVolume());
}

void
SF_UnitTestQSMSort::growthLengthSort()
{
  SORTTYPE type = SORTTYPE::LARGEST_GROWTHLENGTH_SECOND;
  std::shared_ptr<SF_ModelQSM> growthVolumeQSM = SF_QSMFactory::qsm(type);
  QVERIFY(growthVolumeQSM->getRootSegment()->getChildren()[0]->getGrowthLength() <
          growthVolumeQSM->getRootSegment()->getChildren()[1]->getGrowthLength());
  growthVolumeQSM->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_LENGTH);
  QVERIFY(growthVolumeQSM->getRootSegment()->getChildren()[0]->getGrowthLength() >
          growthVolumeQSM->getRootSegment()->getChildren()[1]->getGrowthLength());
}

void
SF_UnitTestQSMSort::angleSort()
{
  SORTTYPE type = SORTTYPE::LARGEST_ANGLE_SECOND;
  std::shared_ptr<SF_ModelQSM> growthVolumeQSM = SF_QSMFactory::qsm(type);
  QVERIFY(growthVolumeQSM->getRootSegment()->getChildren()[0]->getEnd()[2] <
          growthVolumeQSM->getRootSegment()->getChildren()[1]->getEnd()[2]);
  growthVolumeQSM->sort(SF_ModelAbstractSegment::SF_SORTTYPE::ANGLE);
  QVERIFY(growthVolumeQSM->getRootSegment()->getChildren()[0]->getEnd()[2] >
          growthVolumeQSM->getRootSegment()->getChildren()[1]->getEnd()[2]);
}

void
SF_UnitTestQSMSort::radiusSort()
{
  SORTTYPE type = SORTTYPE::LARGEST_RADIUS_SECOND;
  std::shared_ptr<SF_ModelQSM> growthVolumeQSM = SF_QSMFactory::qsm(type);
  QVERIFY(growthVolumeQSM->getRootSegment()->getChildren()[0]->getRadius() <
          growthVolumeQSM->getRootSegment()->getChildren()[1]->getRadius());
  growthVolumeQSM->sort(SF_ModelAbstractSegment::SF_SORTTYPE::RADIUS);
  QVERIFY(growthVolumeQSM->getRootSegment()->getChildren()[0]->getRadius() >
          growthVolumeQSM->getRootSegment()->getChildren()[1]->getRadius());
}

QTEST_MAIN(SF_UnitTestQSMSort)

#include "sf_unittestqsmsort.moc"

#endif // SF_UNIT_TEST_QSM_SORT_H
