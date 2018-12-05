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

#ifndef SF_ADAPTERPRINCIPALDIRECTION_H
#define SF_ADAPTERPRINCIPALDIRECTION_H

#include <QThreadPool>

#include "cloud/feature/principaldirection/sf_principaldirection.h"
#include "cloud/filter/binary/voxelgriddownscale/sf_voxelgriddownscale.h"
#include "parameter/sf_parameterSetPrincipalDirection.h"

class SF_AdapterPrincipalDirection {
public:
  std::shared_ptr<QMutex> mMutex;

  SF_AdapterPrincipalDirection(const SF_AdapterPrincipalDirection &obj) {
    mMutex = obj.mMutex;
  }

  SF_AdapterPrincipalDirection() { mMutex.reset(new QMutex); }

  ~SF_AdapterPrincipalDirection() {}

  void operator()(SF_ParameterSetPrincipalDirection<SF_PointNormal> &params) {

    SF_VoxelGridDownscale<SF_PointNormal> downscale;
    {
      QMutexLocker m1(&*mMutex);
      downscale.setParams(params.m_paramVoxelGridDownscaling);
    }
    downscale.compute();
    SF_PrincipalDirection<SF_PointNormal> pd;
    {
      QMutexLocker m1(&*mMutex);
      params.m_cloud = downscale.clusterOut().first;
      pd.setParams(params);
    }
    pd.compute();
    {
      QMutexLocker m1(&*mMutex);
      params.m_principalCurvatures = pd.principalCurvatures();
    }
  }
};
#endif // SF_ADAPTERPRINCIPALDIRECTION_H
