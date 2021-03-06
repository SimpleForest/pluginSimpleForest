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

#ifndef SF_QSMMEDIANFILTER_H
#define SF_QSMMEDIANFILTER_H

#include "qsm/algorithm/sf_QSMCylinder.h"
#include "qsm/model/sf_modelQSM.h"

class SF_QSMMedianFilter
{
  float m_percentage = 0.2f;
  std::shared_ptr<SF_ModelQSM> m_qsm;
  int m_filterSize = 3;

public:
  SF_QSMMedianFilter();
  void compute(std::shared_ptr<SF_ModelQSM> qsm);
  void setPercentage(float percentage);
};

#endif // SF_QSMMEDIANFILTER_H
