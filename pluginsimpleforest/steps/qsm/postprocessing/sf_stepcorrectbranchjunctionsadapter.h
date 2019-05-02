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

#ifndef SF_STEPCORRECTBRANCHJUNCTIONSADAPTER_H
#define SF_STEPCORRECTBRANCHJUNCTIONSADAPTER_H

#include <QThreadPool>

#include "qsm/algorithm/postprocessing/sf_correctbranchjunction.h"
#include "steps/param/sf_paramAllSteps.h"

class SF_StepCorrectBranchJunctionsAdapter
{
public:
  std::shared_ptr<QMutex> mMutex;

  SF_StepCorrectBranchJunctionsAdapter(const SF_StepCorrectBranchJunctionsAdapter& obj) { mMutex = obj.mMutex; }

  SF_StepCorrectBranchJunctionsAdapter() { mMutex.reset(new QMutex); }

  ~SF_StepCorrectBranchJunctionsAdapter() {}

  void operator()(SF_ParamAllometricCorrectionNeighboring& params)
  {
    SF_CorrectBranchJunction ac;
    {
      QMutexLocker m1(&*mMutex);
      ac.setParams(params);
    }
    ac.compute();
  }
};

#endif // SF_STEPCORRECTBRANCHJUNCTIONSADAPTER_H
