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

void
SF_QSMMedianFilter::setPercentage(float percentage)
{
  m_percentage = percentage;
}

SF_QSMMedianFilter::SF_QSMMedianFilter() {}

void
SF_QSMMedianFilter::compute(std::shared_ptr<SF_ModelQSM> qsm)
{
  m_qsm = qsm;
  if (!qsm) {
    return;
  }

  auto bricks = m_qsm->getBuildingBricks();
  for (auto brick : bricks) {
    std::vector<double> radii;
    radii.push_back(brick->getRadius());
    auto current = brick;
    for (int i = 0; i < m_filterSize; i++) {
      auto parent = brick->getParent();
      if (parent && parent->getFittingType() != FittingType::DIJKSTRALIGHT && parent->getFittingType() != FittingType::CONNECTQSM) {
        current = parent;
      }
      radii.push_back(current->getRadius());
    }
    current = brick;
    for (int i = 0; i < m_filterSize; i++) {
      if (!brick->getChildren().empty()) {
        auto child = brick->getChildren().front();
        if (child && child->getFittingType() != FittingType::DIJKSTRALIGHT && child->getFittingType() != FittingType::CONNECTQSM) {
          current = child;
        }
      }

      radii.push_back(current->getRadius());
    }
    if (radii.size() == 2 * m_filterSize + 1) {
      auto median = SF_Math<double>::getMedian(radii);
      float radius = brick->getRadius();
      FittingType type = brick->getFittingType();
      if (radius < median * (1.0 - m_percentage)) {
        radius = median;
        type = FittingType::MEDIAN;
      }
      if (radius > median * (1.0 + m_percentage)) {
        radius = median;
        type = FittingType::MEDIAN;
      }
      brick->setRadius(radius, type);
    }
  }
}
