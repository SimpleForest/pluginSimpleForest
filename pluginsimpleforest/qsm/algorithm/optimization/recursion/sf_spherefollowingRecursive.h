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

#ifndef SF_SPHEREFOLLOWINGRECURSIVE_H
#define SF_SPHEREFOLLOWINGRECURSIVE_H

#include "qsm/algorithm/optimization/gridsearch/sf_spherefollowingrastersearch.h"
#include "qsm/model/sf_modelCylinderBuildingbrick.h"

#include <pcl/kdtree/kdtree_flann.h>

class SF_SpherefollowingRecursive
{
  SF_ParamSpherefollowingRecursive<SF_PointNormal> m_params;
  SF_CloudNormal::Ptr m_cloud;
  std::shared_ptr<SF_ModelQSM> m_qsm;
  std::shared_ptr<Sf_ModelCylinderBuildingbrick> m_axis;
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> m_bricks;
  pcl::KdTreeFLANN<SF_PointNormal>::Ptr _kdtreeQSM;
  SF_CloudNormal::Ptr extractUnfittedPoints();
  std::vector<SF_CloudNormal::Ptr> clusters(SF_CloudNormal::Ptr cloud);
  std::vector<SF_CloudNormal::Ptr> sortClusters(const std::vector<SF_CloudNormal::Ptr>& clusters);
  void processClusters(const std::vector<SF_CloudNormal::Ptr>& clusters);
  void connectQSM(std::shared_ptr<SF_ModelQSM> childQSM);
  void getMinMax(size_t& min, size_t& max, SF_CloudNormal::Ptr cluster);
  std::shared_ptr<Sf_ModelCylinderBuildingbrick> stemAxis();
  Eigen::Vector3f cloudVector(SF_CloudNormal::Ptr& cloud, const size_t minIndex, const size_t maxIndex);
  Eigen::Vector3d translateCloud(SF_CloudNormal::Ptr& cloud, const size_t index);
  void initializeKdTree();

public:
  SF_SpherefollowingRecursive();
  void compute();
  void setParams(const SF_ParamSpherefollowingRecursive<SF_PointNormal>& params);
  void setCloud(const SF_CloudNormal::Ptr& cloud);
  void setQsm(const std::shared_ptr<SF_ModelQSM>& qsm);
  std::shared_ptr<SF_ModelQSM> getQsm() const;
};

#endif // SF_SPHEREFOLLOWINGRECURSIVE_H
