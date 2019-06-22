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

#ifndef SF_SPHEREFOLLOWINGRASTERSEARCH_H
#define SF_SPHEREFOLLOWINGRASTERSEARCH_H

#include "pcl/cloud/feature/descriptor/sf_descriptor.h"
#include "qsm/algorithm/spherefollowing/sf_spherefollowing.h"

class SF_SphereFollowingRasterSearch
{
public:
  SF_SphereFollowingRasterSearch();
  void compute();
  std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal>> paramVector();
  void setParams(const SF_ParamSpherefollowingBasic<SF_PointNormal>& params);
  void setCloud(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud);
  std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal>> getParamVec() const;
  void setFire(bool fire);

private:
  bool m_fire = true;
  SF_ParamSpherefollowingBasic<SF_PointNormal> m_params;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr m_cloud;
  std::vector<SF_ParamSpherefollowingBasic<SF_PointNormal>> m_paramVec;
};

#endif // SF_SPHEREFOLLOWINGRASTERSEARCH_H
