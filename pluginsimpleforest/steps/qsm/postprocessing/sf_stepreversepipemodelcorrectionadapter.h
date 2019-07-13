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

#ifndef SF_STEPREVERSEPIPEMODELCORRECTIONADAPTER_H
#define SF_STEPREVERSEPIPEMODELCORRECTIONADAPTER_H

#include <QThreadPool>

#include "qsm/algorithm/postprocessing/sf_QSMInversePipeModelParamaterEstimation.h"
#include "steps/param/sf_paramAllSteps.h"

class SF_QSMReversePipeModelCorrectionAdapter
{
public:
  std::shared_ptr<QMutex> mMutex;

  SF_QSMReversePipeModelCorrectionAdapter(const SF_QSMReversePipeModelCorrectionAdapter& obj) { mMutex = obj.mMutex; }

  SF_QSMReversePipeModelCorrectionAdapter() { mMutex.reset(new QMutex); }

  ~SF_QSMReversePipeModelCorrectionAdapter() {}

  void operator()(SF_ParamReversePipeModelCorrection& params)
  {
    SF_ParamReversePipeModelCorrection paramsCpy;
    {
      QMutexLocker m1(&*mMutex);
      paramsCpy = params;
    }
    {
      QMutexLocker m1(&*mMutex);
      SF_QSMReversePipeModelParamaterEstimation paramEst;
      paramEst.setParams(paramsCpy);
      try {
        paramEst.compute();
        auto equation = paramEst.equation();
        if (equation.first == 0) {
          std::cout << "SF_QSMReversePipeModelParamaterEstimation 0" << std::endl;
          return;
        }
        auto bricks = params._qsm->getBuildingBricks();
        for (auto brick : bricks) {
          if (brick->getSegment()->isRoot()) {
            continue;
          }
          auto radius = brick->getRadius();
          auto reverseBranchOrder = brick->getSegment()->getReversePipeBranchOrder();
          auto predictedRadius = equation.first * reverseBranchOrder + equation.second;
          if (std::abs(predictedRadius - radius) > paramsCpy.m_inlierDistance) {
            radius = std::max(radius, 0.005);
            brick->setRadius(radius, FittingType::REVERSEPIPEMODEL);
          }
        }
      } catch (...) {
        std::cout << "BAR SF_QSMReversePipeModelParamaterEstimation failed" << std::endl;
      }
    }
    {
      QMutexLocker m1(&*mMutex);
      params = paramsCpy;
    }
  }
};

#endif // SF_STEPREVERSEPIPEMODELCORRECTIONADAPTER_H
