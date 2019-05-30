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

#ifndef SF_STEPQSMALLOMETRICCHECKADAPTER_H
#define SF_STEPQSMALLOMETRICCHECKADAPTER_H

#include <QThreadPool>

#include "qsm/algorithm/postprocessing/sf_QSMAllometryCorrectionNeighboring.h"
#include "qsm/algorithm/postprocessing/sf_qsmallometriccorrectionparameterestimation.h"
#include "steps/param/sf_paramAllSteps.h"

class SF_QSMAllometricCheckAdapter
{
public:
  std::shared_ptr<QMutex> mMutex;

  SF_QSMAllometricCheckAdapter(const SF_QSMAllometricCheckAdapter& obj) { mMutex = obj.mMutex; }

  SF_QSMAllometricCheckAdapter() { mMutex.reset(new QMutex); }

  ~SF_QSMAllometricCheckAdapter() {}

  void operator()(SF_ParamAllometricCorrectionNeighboring& params)
  {
    SF_ParamAllometricCorrectionNeighboring paramsCpy;
    {
      QMutexLocker m1(&*mMutex);
      paramsCpy = params;
    }
    if (paramsCpy.m_estimateParams) {
      QMutexLocker m1(&*mMutex);
      SF_QSMAllometricCorrectionParameterEstimation paramEst;
      paramEst.setParams(paramsCpy);
      try {
        paramEst.compute();
      } catch (std::exception& exp) {
        if (paramsCpy.m_useGrowthLength) {
          paramsCpy.m_useGrowthLength = false;
          paramEst.setParams(paramsCpy);
          try {
            paramEst.compute();
            paramsCpy = paramEst.params();
          } catch (...) {
          }
        }
      } catch (...) {
        if (paramsCpy.m_useGrowthLength) {
          paramsCpy.m_useGrowthLength = false;
          paramEst.setParams(paramsCpy);
          try {
            paramEst.compute();
            paramsCpy = paramEst.params();
          } catch (...) {
          }
        }
      }
      if (paramsCpy.m_useGrowthLength) {
        paramsCpy.m_power = paramsCpy._qsm->getBGrowthLength();
        std::cout << "paramsCpy.m_power " << paramsCpy.m_power << std::endl;
        std::cout << "paramsCpy._qsm->a() " << paramsCpy._qsm->getAGrowthLength() << std::endl;
        std::cout << "paramsCpy._qsm->b() " << paramsCpy._qsm->getBGrowthLength() << std::endl;
        std::cout << "paramsCpy._qsm->c() " << paramsCpy._qsm->getCGrowthLength() << std::endl;
      } else {
        paramsCpy.m_power = paramsCpy._qsm->getBGrowthVolume();
        std::cout << "paramsCpy.m_power " << paramsCpy.m_power << std::endl;
        std::cout << "paramsCpy._qsm->a() " << paramsCpy._qsm->getAGrowthVolume() << std::endl;
        std::cout << "paramsCpy._qsm->b() " << paramsCpy._qsm->getBGrowthVolume() << std::endl;
        std::cout << "paramsCpy._qsm->c() " << paramsCpy._qsm->getCGrowthVolume() << std::endl;
      }
    }

    SF_QSMAllometryCorrectionNeighboring ac;
    {
      QMutexLocker m1(&*mMutex);
      ac.setParams(paramsCpy);
      paramsCpy._qsm->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME);
    }
    if(paramsCpy.m_power > 0.25 && paramsCpy.m_power < 0.75)
    {
        ac.compute();
    }
    {
      QMutexLocker m1(&*mMutex);
      params = paramsCpy;
    }
  }
};

#endif // SF_STEPQSMALLOMETRICCHECKADAPTER_H
