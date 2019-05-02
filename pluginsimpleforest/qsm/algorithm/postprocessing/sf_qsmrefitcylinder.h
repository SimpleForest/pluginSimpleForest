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

#ifndef SF_QSMREFITCYLINDER_H
#define SF_QSMREFITCYLINDER_H

#include "qsm/model/sf_modelQSM.h"
#include "steps/param/sf_paramAllSteps.h"
#include <pcl/kdtree/kdtree_flann.h>

class SF_QSMRefitCylinder
{
  SF_ParamRefitCylinders m_params;
  pcl::KdTreeFLANN<SF_PointNormal>::Ptr m_kdtreeQSM;
  SF_CloudNormal::Ptr m_cloud;
  std::vector<SF_CloudNormal::Ptr> m_cylinderClusters;

  void initialize();

public:
  SF_QSMRefitCylinder();
  void compute();
  void setParams(const SF_ParamRefitCylinders& params);
  void setCloud(const SF_CloudNormal::Ptr& cloud);
};

#endif // SF_QSMREFITCYLINDER_H
