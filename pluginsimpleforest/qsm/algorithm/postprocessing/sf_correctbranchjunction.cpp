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

#include "sf_correctbranchjunction.h"

SF_CorrectBranchJunction::SF_CorrectBranchJunction()
{
  std::cout << "TODO: SF_CorrectBranchJunction make line cylinder intersection." << std::endl;
}

void
SF_CorrectBranchJunction::setParams(const SF_ParamAllometricCorrectionNeighboring& params)
{
  m_params = params;
}

SF_ParamAllometricCorrectionNeighboring
SF_CorrectBranchJunction::params() const
{
  return m_params;
}

void
SF_CorrectBranchJunction::compute()
{
  if (!m_params._qsm) {
    return;
  }
  auto translation = m_params._qsm->translateToOrigin();
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments = m_params._qsm->getSegments();
  std::for_each(
    segments.begin(), segments.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> segment) { correctSegment(segment); });
  m_params._qsm->translate(-translation);
}

void
SF_CorrectBranchJunction::correctSegment(std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> children = segment->getChildren();
  std::for_each(children.begin(), children.end(), [this](std::shared_ptr<SF_ModelAbstractSegment> child) {
    size_t numberBricks = child->getBuildingBricks().size();
    if (numberBricks <= 1) {
      return;
    }
    std::shared_ptr<Sf_ModelAbstractBuildingbrick> first = child->getBuildingBricks()[0];
    std::shared_ptr<Sf_ModelAbstractBuildingbrick> second = child->getBuildingBricks()[1];
    auto axisSecond = second->getAxis().normalized();
    auto length = first->getLength();
    auto axisNew = -(axisSecond)*length;
    auto newStartFirst = first->getEnd() + axisNew;
    first->setStartEndRadius(newStartFirst, first->getEnd(), first->getRadius(), first->getFittingType());
  });
}
