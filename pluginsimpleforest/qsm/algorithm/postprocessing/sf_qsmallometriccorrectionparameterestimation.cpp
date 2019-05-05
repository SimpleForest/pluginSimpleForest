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

#include "sf_qsmallometriccorrectionparameterestimation.h"

#include "math/fit/power/sf_fitgnpower.h"

void
SF_QSMAllometricCorrectionParameterEstimation::setParams(const SF_ParamAllometricCorrectionNeighboring& params)
{
  m_params = params;
}

SF_ParamAllometricCorrectionNeighboring
SF_QSMAllometricCorrectionParameterEstimation::params() const
{
  return m_params;
}

SF_QSMAllometricCorrectionParameterEstimation::SF_QSMAllometricCorrectionParameterEstimation() {}

bool
SF_QSMAllometricCorrectionParameterEstimation::isUnCorrectedRadiusFit(std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick)
{
  return (buildingBrick->getFittingType() == SPHEREFOLLOWING || buildingBrick->getFittingType() == CYLINDERCORRECTION ||
          buildingBrick->getFittingType() == TRUNCATEDCONECORRECTION);
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_QSMAllometricCorrectionParameterEstimation::chooseBestBricks(std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks)
{
  std::vector<float> y;
  if (m_params.m_useGrowthLength) {
    std::transform(bricks.begin(), bricks.end(), std::back_inserter(y), [](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) {
      return brick->getGrowthLength();
    });

  } else {
    std::transform(bricks.begin(), bricks.end(), std::back_inserter(y), [](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) {
      return brick->getGrowthVolume();
    });
  }
  std::sort(y.begin(), y.end());
  float min = SF_Math<float>::getQuantile(y, (1 - m_params.m_quantile));
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bestBricks;
  std::for_each(bricks.begin(), bricks.end(), [&min, &bestBricks, this](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) {
    if (m_params.m_useGrowthLength) {
      float growthLength = brick->getGrowthLength();
      if (growthLength > min) {
        bestBricks.push_back(brick);
      }
    } else {
      float growthVolume = brick->getGrowthVolume();
      if (growthVolume > min) {
        bestBricks.push_back(brick);
      }
    }
  });
  return bestBricks;
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_QSMAllometricCorrectionParameterEstimation::unCorrectedBuildingBricks()
{
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments = m_params._qsm->getSegments();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingbricks;
  std::for_each(segments.begin(), segments.end(), [this, &buildingbricks](std::shared_ptr<SF_ModelAbstractSegment> segment) {
    if (!segment->isRoot()) {
      std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> segmentsBuildingBricks = segment->getBuildingBricks();
      if (m_params.m_useGrowthLength) {
        if (segmentsBuildingBricks.size() > 0) {
          int median = segmentsBuildingBricks.size() / 2;
          std::shared_ptr<Sf_ModelAbstractBuildingbrick> medianBrick = segmentsBuildingBricks[median];
          if (isUnCorrectedRadiusFit(medianBrick)) {
            buildingbricks.push_back(medianBrick);
          }
        }
      } else {
        std::for_each(segmentsBuildingBricks.begin(),
                      segmentsBuildingBricks.end(),
                      [this, &buildingbricks](std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick) {
                        if (isUnCorrectedRadiusFit(buildingBrick)) {
                          buildingbricks.push_back(buildingBrick);
                        }
                      });
      }
    }
  });
  return buildingbricks;
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_QSMAllometricCorrectionParameterEstimation::removeStem(std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks)
{
  auto crownStart = m_params._qsm->crownStartBrick(m_params._crownStartFraction);
  float height = crownStart->getCenter()[2];

  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> noStemBricks;
  std::for_each(bricks.begin(), bricks.end(), [&height, &noStemBricks](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) {
    auto cylinderHeight = brick->getCenter()[2];
    if (cylinderHeight > height || brick->getSegment()->getBranchOrder() != 0) {
      noStemBricks.push_back(brick);
    }
  });
  return noStemBricks;
}

void
SF_QSMAllometricCorrectionParameterEstimation::compute()
{
  std::vector<float> y;
  std::vector<float> x;

  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> uncorrectedBricks = unCorrectedBuildingBricks();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> noStemBricks = removeStem(uncorrectedBricks);
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bestBricks = chooseBestBricks(noStemBricks);
  std::transform(bestBricks.begin(),
                 bestBricks.end(),
                 std::back_inserter(y),
                 [](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) { return brick->getRadius(); });
  if (m_params.m_useGrowthLength) {
    std::transform(bestBricks.begin(),
                   bestBricks.end(),
                   std::back_inserter(x),
                   [](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) { return brick->getGrowthLength(); });
  } else {
    std::transform(bestBricks.begin(),
                   bestBricks.end(),
                   std::back_inserter(x),
                   [](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) { return brick->getGrowthVolume(); });
  }
  SF_FitGNPower<float> powerFit;
  powerFit.setX(x);
  powerFit.setY(y);
  powerFit.setMinPts(m_params.m_minPts);
  powerFit.setInlierDistance(m_params.m_inlierDistance);
  powerFit.setRansacIterations(m_params.m_ransacIterations);
  powerFit.setGaussNewtonIterations(m_params.m_gaussNewtonIterations);
  powerFit.setFitWithIntercept(m_params.m_withIntercept);
  try {
    powerFit.compute();
    m_params.m_power = powerFit.a();
    if (m_params.m_useGrowthLength) {
      m_params._qsm->setAGrowthLength(powerFit.a());
      m_params._qsm->setBGrowthLength(powerFit.b());
      m_params._qsm->setCGrowthLength(powerFit.c());
    } else {
      m_params._qsm->setAGrowthVolume(powerFit.a());
      m_params._qsm->setBGrowthVolume(powerFit.b());
      m_params._qsm->setCGrowthVolume(powerFit.c());
    }
  } catch (const std::exception& e) {
    std::string eStr = std::string(e.what());
    eStr.append("Could not fit allometric parameters.");
    throw(eStr);
  } catch (...) {
    throw("Could not fit allometric parameters.");
  }
}
