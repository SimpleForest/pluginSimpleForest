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

#ifndef SF_SPHERE_FOLLOWING_H
#define SF_SPHERE_FOLLOWING_H

#include "qsm/algorithm/detection/sf_idetection.h"
#include "sf_spherefollowing_parameters.h"

class SF_SphereFollowing : public SF_IDetection {
  std::shared_ptr<SF_ModelQSM> _qsm;
  SF_SphereFollowingParameters _params;
  std::vector<SF_SphereFollowingOptimizationParameters> _optimParams;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr _cloud;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> _clusters;

public:
  SF_SphereFollowing(
      SF_SphereFollowingParameters params,
      std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clusters);
  const virtual std::shared_ptr<SF_ModelQSM> getQSM() override;
  virtual void compute() override {}
  virtual void error() override {}

private:
  void initializeCloud();
};

#endif // SF_SPHERE_FOLLOWING_H
