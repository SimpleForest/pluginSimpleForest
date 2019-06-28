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

#ifndef SF_QSMALLOMETRICCORRECTIONPARAMETERESTIMATION_H
#define SF_QSMALLOMETRICCORRECTIONPARAMETERESTIMATION_H

#include "qsm/algorithm/sf_QSMCylinder.h"
#include "qsm/model/sf_modelQSM.h"
#include "steps/param/sf_paramAllSteps.h"

class SF_QSMAllometricCorrectionParameterEstimation
{
  SF_ParamAllometricCorrectionNeighboring m_params;

public:
  SF_QSMAllometricCorrectionParameterEstimation();
  void setParams(const SF_ParamAllometricCorrectionNeighboring& params);
  SF_ParamAllometricCorrectionNeighboring params() const;
  void compute();
  bool isUnCorrectedRadiusFit(std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick);

  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> unCorrectedBuildingBricks();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> removeStem(
    std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks);
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> chooseBestBricks(
    std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks);
};

#endif // SF_QSMALLOMETRICCORRECTIONPARAMETERESTIMATION_H
