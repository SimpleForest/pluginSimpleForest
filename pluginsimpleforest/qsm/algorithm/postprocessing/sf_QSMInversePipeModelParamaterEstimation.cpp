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

#include "sf_QSMInversePipeModelParamaterEstimation.h"
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

#include "sf_QSMInversePipeModelParamaterEstimation.h"
#include "math/fit/line/sf_fitline.h"
#include "math/fit/line/sf_fitransacline.h"

void
SF_QSMInversePipeModelParamaterEstimation::setParams(const SF_ParamInversePipeModelCorrection &params)
{
  m_params = params;
}

SF_ParamInversePipeModelCorrection
SF_QSMInversePipeModelParamaterEstimation::params() const
{
  return m_params;
}

SF_QSMInversePipeModelParamaterEstimation::SF_QSMInversePipeModelParamaterEstimation() {}

bool
SF_QSMInversePipeModelParamaterEstimation::isUnCorrectedRadiusFit(std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick)
{
  return ((buildingBrick->getFittingType() == SPHEREFOLLOWING || buildingBrick->getFittingType() == CYLINDERCORRECTION ||
           buildingBrick->getFittingType() == TRUNCATEDCONECORRECTION));
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_QSMInversePipeModelParamaterEstimation::chooseBestBricks(std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks)
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
  return bestBricks;
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_QSMInversePipeModelParamaterEstimation::unCorrectedBuildingBricks()
{
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> segments = m_params._qsm->getSegments();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingbricks;
  std::for_each(segments.begin(), segments.end(), [this, &buildingbricks](std::shared_ptr<SF_ModelAbstractSegment> segment) {
    if (!segment->isRoot()) {
      std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> segmentsBuildingBricks = segment->getBuildingBricks();
      if (segmentsBuildingBricks.size() > 2) {
        int median = segmentsBuildingBricks.size() / 2;
        std::shared_ptr<Sf_ModelAbstractBuildingbrick> medianBrick = segmentsBuildingBricks[median];
        if (isUnCorrectedRadiusFit(medianBrick)) {
          buildingbricks.push_back(medianBrick);
        }
      }
      }
    });
  return buildingbricks;
}

std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>
SF_QSMInversePipeModelParamaterEstimation::removeStem(std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks)
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
SF_QSMInversePipeModelParamaterEstimation::compute()
{
  std::vector<float> y;
  std::vector<float> x;

  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> uncorrectedBricks = unCorrectedBuildingBricks();
  if (uncorrectedBricks.size() < 5) {
    return;
  }
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> removedStem = removeStem(uncorrectedBricks);
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bestBricks = chooseBestBricks(removedStem);
  if (bestBricks.size() < 5) {
    return;
  }
  std::transform(bestBricks.begin(),
                 bestBricks.end(),
                 std::back_inserter(y),
                 [](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) { return brick->getRadius(); });
  std::transform(bestBricks.begin(),
                 bestBricks.end(),
                 std::back_inserter(x),
                 [](std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick) { return brick->getSegment()->getReversePipeBranchOrder(); });
SF_FitRansacLine<float> lineFitRANSAC;
lineFitRANSAC.setX(x);
lineFitRANSAC.setY(y);
lineFitRANSAC.setInlierDistance(m_params.m_inlierDistance);
lineFitRANSAC.setIterations(m_params.m_ransacIterations);
lineFitRANSAC.setMinPts(m_params.m_minPts);
try {
    lineFitRANSAC.compute();
    auto inliers = lineFitRANSAC.inliers();
    SF_FitLine<float> lineFit;
    lineFit.setX(inliers.first);
    lineFit.setY(inliers.second);
    lineFit.setMinPts(m_params.m_minPts);
    lineFit.compute();
    m_equation = lineFit.equation();
  } catch (const std::exception& e) {
    std::string eStr = std::string(e.what());
    eStr.append("Could not fit reverse pipemodel parameters.");
    std::cout << eStr << std::endl;
    throw(eStr);
  } catch (...) {
    throw("Could not fit reverse pipemodel parameters.");
  }
}
