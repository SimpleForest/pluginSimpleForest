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
  return ((buildingBrick->getFittingType() == SPHEREFOLLOWING || buildingBrick->getFittingType() == CYLINDERCORRECTION ||
           buildingBrick->getFittingType() == TRUNCATEDCONECORRECTION));
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_QSMAllometricCorrectionParameterEstimation::chooseBestBricks(std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks)
{
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bestBricks;
  float minRadius = std::numeric_limits<float>::max();
  float maxRadius = 0;
  float minY = std::numeric_limits<float>::max();
  float maxY = 0;
  float radiusAtMaxY = 0;
  for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick : bricks) {
    float y;
    if (m_params.m_useGrowthLength) {
      y = brick->getGrowthLength();
    } else {
      y = brick->getGrowthVolume();
    }
    float radius = brick->getRadius();
    if (radius < minRadius) {
      minRadius = radius;
    }
    if (radius > maxRadius) {
      maxRadius = radius;
    }

    if (y < minY) {
      minY = y;
    }

    if (y > maxY) {
      maxY = y;
      radiusAtMaxY = radius;
    }
  }
  for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick : bricks) {
    float y;
    if (m_params.m_useGrowthLength) {
      y = brick->getGrowthLength();
    } else {
      y = brick->getGrowthVolume();
    }
    float radius = brick->getRadius();

    if (radius > radiusAtMaxY)
      continue;
    if (y < maxY * 0.01)
      continue;
    if (y > maxY * 0.33)
      continue;
    bestBricks.push_back(brick);
  }

  //  std::vector<float> y;
  //  if (m_params.m_useGrowthLength) {
  //    std::transform(bricks.begin(), bricks.end(), std::back_inserter(y), [](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) {
  //      return brick->getGrowthLength();
  //    });

  //  } else {
  //    std::transform(bricks.begin(), bricks.end(), std::back_inserter(y), [](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) {
  //      return brick->getGrowthVolume();
  //    });
  //  }
  //  std::sort(y.begin(), y.end());
  //  float min = SF_Math<float>::getQuantile(y, (1 - m_params.m_quantile));
  //  std::for_each(bricks.begin(), bricks.end(), [&min, &bestBricks, this](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) {
  //    if (m_params.m_useGrowthLength) {
  //      float growthLength = brick->getGrowthLength();
  //      if (growthLength > min) {
  //        bestBricks.push_back(brick);
  //      }
  //    } else {
  //      float growthVolume = brick->getGrowthVolume();
  //      if (growthVolume > min) {
  //        bestBricks.push_back(brick);
  //      }
  //    }
  //  });
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
        if (segmentsBuildingBricks.size() > 2) {
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
  std::cout << "SF_QSMAllometricCorrectionParameterEstimation unCorrectedBuildingBricks SegmentSize " << segments.size() << std::endl;
  std::cout << "SF_QSMAllometricCorrectionParameterEstimation unCorrectedBuildingBricks buildingbricks Size " << buildingbricks.size()
            << std::endl;
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
  std::cout << "SF_QSMAllometricCorrectionParameterEstimation removeStem bricks Size " << bricks.size() << std::endl;
  std::cout << "SF_QSMAllometricCorrectionParameterEstimation removeStem noStemBricks Size " << noStemBricks.size() << std::endl;
  return noStemBricks;
}

void
SF_QSMAllometricCorrectionParameterEstimation::compute()
{
  std::vector<float> y;
  std::vector<float> x;

  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> uncorrectedBricks = unCorrectedBuildingBricks();
  if (uncorrectedBricks.size() < 5) {
    std::cout << "fooo 1" << std::endl;
    return;
  }
  //  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> noStemBricks = removeStem(uncorrectedBricks);
  //  if (noStemBricks.size() < 5) {
  //    return;
  //  }
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bestBricks = chooseBestBricks(uncorrectedBricks);
  if (bestBricks.size() < 5) {
    std::cout << "fooo 2" << std::endl;
    return;
  }
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
  std::cout << "growthvolume, radius" << std::endl;
  for (int i = 0; i < x.size(); i++) {
    std::cout << x[i] << ", " << y[i] << std::endl;
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
    if (powerFit.b() > 0.2 && powerFit.b() < 0.8) {
      m_params.m_power = powerFit.a();
      if (m_params.m_useGrowthLength) {
        std::cout << "POWER GRWOTHJLENGTH" << std::endl;
        m_params._qsm->setA(powerFit.a());
        m_params._qsm->setB(powerFit.b());
        m_params._qsm->setC(powerFit.c());

      } else {
        std::cout << "POWER setAGrowthVolume" << std::endl;
        m_params._qsm->setAGrowthVolume(powerFit.a());
        m_params._qsm->setBGrowthVolume(powerFit.b());
        m_params._qsm->setCGrowthVolume(powerFit.c());
      }
    } else {
      m_params.m_power = 1 / 2.49;
      m_params._qsm->setA(0.01);
      m_params._qsm->setB(1 / 2.49);
      m_params._qsm->setC(0);
    }
    double a = m_params._qsm->getAGrowthLength();
    double b = m_params._qsm->getBGrowthLength();
    double c = m_params._qsm->getCGrowthLength();
    std::cout << "ab q2 c " << a << " ; " << b << " ; " << c << std::endl;
    double minR = 0.005;
    double minGrowthLength = std::pow((minR - c) / a, 1 / b);
    std::cout << "minGrowthLength " << minGrowthLength << " ; " << c << std::endl;

  } catch (const std::exception& e) {
    std::string eStr = std::string(e.what());
    eStr.append("Could not fit allometric parameters.");
    std::cout << eStr << std::endl;
    throw(eStr);
  } catch (...) {
    throw("Could not fit allometric parameters.");
  }
}
