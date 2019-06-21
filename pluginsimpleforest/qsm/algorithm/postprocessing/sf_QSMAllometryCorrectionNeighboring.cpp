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

#include "sf_QSMAllometryCorrectionNeighboring.h"

void
SF_QSMAllometryCorrectionNeighboring::setParams(const SF_ParamAllometricCorrectionNeighboring& params)
{
  m_params = params;
}

SF_ParamAllometricCorrectionNeighboring
SF_QSMAllometryCorrectionNeighboring::params() const
{
  return m_params;
}

void
SF_QSMAllometryCorrectionNeighboring::compute()
{
  if (!m_params._qsm) {
    return;
  }
  if (m_params.m_useGrowthLength) {
  } else {
  }

  auto translation = m_params._qsm->translateToOrigin();
  auto cylinder = startBrick();
  correct(cylinder);
  m_params._qsm->translate(-translation);
}

void
SF_QSMAllometryCorrectionNeighboring::correct(std::shared_ptr<Sf_ModelAbstractBuildingbrick> parent,
                                              std::shared_ptr<Sf_ModelAbstractBuildingbrick> child)
{
  float xNew;
  if (!isInPipeModelRelation(parent, child, xNew)) {
    if (xNew > m_params._minRadius) {
      if (m_params.m_useGrowthLength) {
        child->setRadius(xNew, FittingType::ALLOMETRICGROWTHLENGTH);
      } else {
        child->setRadius(xNew, FittingType::ALLOMETRICGROWTHVOLUME);
      }
    } else {
      child->setRadius(m_params._minRadius, FittingType::MINRADIUS);
    }
  }
}

void
SF_QSMAllometryCorrectionNeighboring::correct(std::shared_ptr<Sf_ModelAbstractBuildingbrick> cylinder)
{
  auto parent = cylinder->getParent();
  if (parent) {
    correct(parent, cylinder);
  }
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> children = cylinder->getChildren();
  std::for_each(
    children.begin(), children.end(), [this, &cylinder](std::shared_ptr<Sf_ModelAbstractBuildingbrick> child) { correct(child); });
}

std::shared_ptr<Sf_ModelAbstractBuildingbrick>
SF_QSMAllometryCorrectionNeighboring::startBrick()
{
  auto root = m_params._qsm->getRootSegment();
  auto cylinder = root->getBuildingBricks().back();
  if (root->getBuildingBricks().size() > 5) {
    cylinder = root->getBuildingBricks()[5];
  }
  if (root->getBuildingBricks().size() > 10) {
    cylinder = root->getBuildingBricks()[10];
  }
  return cylinder;
}

bool
SF_QSMAllometryCorrectionNeighboring::isInPipeModelRelation(std::shared_ptr<Sf_ModelAbstractBuildingbrick> parent,
                                                            std::shared_ptr<Sf_ModelAbstractBuildingbrick> child,
                                                            float& xNew)
{
  float xParent = parent->getRadius();
  float xChild = child->getRadius();
  float yParent, yChild;

  if (m_params.m_useGrowthLength) {
    yParent = parent->getGrowthLength();
    yChild = child->getGrowthLength();
  } else {
    yParent = parent->getGrowthVolume();
    yChild = child->getGrowthVolume();
  }
  xNew = xParent * (std::pow((yChild / yParent), m_params.m_power));
  float minX = xNew - m_params._range * xNew;
  float maxX = xNew + m_params._range * xNew;
  return (xChild > minX && xChild < maxX) ? true : false;
}

SF_QSMAllometryCorrectionNeighboring::SF_QSMAllometryCorrectionNeighboring() {}
