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

#ifndef SF_BUILD_QSM_H
#define SF_BUILD_QSM_H

#include <pcl/octree/octree.h>

#include "pcl/sf_point.h"
#include "qsm/algorithm/sf_QSMAlgorithm.h"
#include "qsm/algorithm/sf_QSMCylinder.h"
#include "qsm/model/sf_modelQSM.h"

class SF_BuildQSM
{
  const float _RESOLUTION = 0.02f;
  const float _MINEQUALDISTANCE = 0.0001f;
  const float _MINLENGTH = 0.01f;
  std::shared_ptr<SF_ModelQSM> _tree;
  pcl::octree::OctreePointCloudSearch<SF_Point>::Ptr _octree;
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> _buildingBricksPtr;

public:
  SF_BuildQSM(const std::vector<SF_QSMDetectionCylinder>& cylinders, int index);
  std::shared_ptr<SF_ModelQSM> getTree() const;

private:
  void buildTree(std::shared_ptr<SF_ModelAbstractSegment> segment);
  void initializeOctree();
  void initializeCylinderBuildingBricks(const std::vector<SF_QSMDetectionCylinder>& cylinders);
  void initializeTree(int index);
  void addBuildingBrickToSegment(const std::vector<int>& pointIdxNKNSearch, std::shared_ptr<SF_ModelAbstractSegment> segment);
  void addBuildingBricksToChildSegments(const std::vector<int>& pointIdxNKNSearch, std::shared_ptr<SF_ModelAbstractSegment> segment);
  void addChildBuildingbricks(std::shared_ptr<SF_ModelAbstractSegment> segment, std::vector<int> pointIdxNKNSearch);
  SF_Cloud::Ptr m_cloud;
};

#endif // SF_BUILD_QSM_H
