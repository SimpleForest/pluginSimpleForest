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

#include "sf_qsmmedianfilter.h"

#include "pcl/sf_math.h"

SF_QSMMedianFilter::SF_QSMMedianFilter() {}

void
SF_QSMMedianFilter::compute(std::shared_ptr<SF_ModelQSM> qsm)
{
  m_qsm = qsm;
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments = m_qsm->getSegments();
  std::for_each(segments.begin(), segments.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> segment) {
    if (!segment->isRoot()) {
      std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks = segment->getBuildingBricks();
      std::vector<float> radii;
      std::for_each(bricks.begin(), bricks.end(), [&radii, this](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) {
        radii.push_back(brick->getRadius());
      });
      float median = SF_Math<float>::getMedian(radii);
      std::for_each(bricks.begin(), bricks.end(), [&radii, median, this](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) {
        float radius = brick->getRadius();
        if (radius < median * (1 - M_DEVIATIONPERCENTAGE)) {
          radius = median;
        }
        if (radius > median * (1 + M_DEVIATIONPERCENTAGE)) {
          radius = median;
        }
        brick->setRadius(radius);
      });
    }
  });
}
