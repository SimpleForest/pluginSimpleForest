/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_testclustertransfer.h is part of SimpleForest - a plugin for the
    Computree platform.

    SimpleForest is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SimpleForest is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with
    SimpleForest. If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#ifndef SF_TESTCLUSTERTRANSFER_CPP
#define SF_TESTCLUSTERTRANSFER_CPP

#include "sf_testclustertransfer.h"

void TestClusterTransfer::initTestCase() {
  qDebug("called before everything else");
}
void TestClusterTransfer::myFirstTest() { QVERIFY(1 == 1); }
void TestClusterTransfer::mySecondTest() { QVERIFY(1 != 2); }
void TestClusterTransfer::cleanupTestCase() {
  qDebug("called after myFirstTest and mySecondTest");
}

#endif // SF_TESTCLUSTERTRANSFER_H
