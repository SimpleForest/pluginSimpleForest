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
#include "sf_qsm_algorithm.h"
#include "sf_qsm_cylinder.h"
#include "qsm/sf_model_tree.h"

class SF_Build_QSM {
    const float _RESOLUTION = 0.02f;
    const float _MINEQUALDISTANCE = 0.0001f;
    std::shared_ptr<SF_Model_Tree> _tree;
    pcl::octree::OctreePointCloudSearch<SF_Point>::Ptr _octree;
    std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > _buildingBricks;

public:
    SF_Build_QSM(const std::vector<SF_QSMDetectionCylinder> &cylinders, int index);
    std::shared_ptr<SF_Model_Tree> getTree() const;

private:
    void buildTree(std::shared_ptr<SF_Model_Abstract_Segment> segment);
    void initializeOctree();
    void initializeCylinderBuildingBricks(const std::vector<SF_QSMDetectionCylinder> &cylinders);
    void initializeTree(int index);
    void addBuildingBrickToSegment(const std::vector<int> &pointIdxNKNSearch, std::shared_ptr<SF_Model_Abstract_Segment> segment);
    void addBuildingBricksToChildSegments(const std::vector<int> &pointIdxNKNSearch, std::shared_ptr<SF_Model_Abstract_Segment> segment);
    void addChildBuildingbricks(std::shared_ptr<SF_Model_Abstract_Segment> segment, std::vector<int> pointIdxNKNSearch);
};

#endif // SF_BUILD_QSM_H
