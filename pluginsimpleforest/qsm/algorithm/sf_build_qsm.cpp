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

#include "sf_build_qsm.h"
#include "qsm/sf_model_cylinder_buildingbrick.h"

void SF_Build_QSM::initializeOctree() {
    _octree.reset(new pcl::octree::OctreePointCloudSearch<SF_Point>(_RESOLUTION));
    SF_Cloud::Ptr cloud(new SF_Cloud);
    for(size_t i = 1; i < _buildingBricks.size(); i++) {
        std::shared_ptr<SF_Model_Abstract_Buildingbrick> buildingBrick = _buildingBricks[i];
        SF_Point point(buildingBrick->getStart()[0], buildingBrick->getStart()[1], buildingBrick->getStart()[2]);
        cloud->push_back(point);
    }
    _octree->setInputCloud(cloud);
    _octree->addPointsFromInputCloud();
}

void SF_Build_QSM::initializeCylinderBuildingBricks(const std::vector<Cylinder> &cylinders) {
    _buildingBricks.clear();
    for(size_t i = 0; i < cylinders.size(); i++) {
        Cylinder cyl = cylinders[i];
        std::shared_ptr<SF_Model_Cylinder_Buildingbrick> cylinder(new SF_Model_Cylinder_Buildingbrick(cyl._circleA,cyl._circleB));
        _buildingBricks.push_back(cylinder);
    }
}

void SF_Build_QSM::initializeTree(int index) {
    _tree.reset(new SF_Model_Tree(index));
    std::shared_ptr<SF_Model_Abstract_Segment> rootSegment(new SF_Model_Abstract_Segment(_tree));
    rootSegment->addBuildingBrick(_buildingBricks[0]);
    _tree->setRootSegment(rootSegment);
}

std::shared_ptr<SF_Model_Tree> SF_Build_QSM::getTree() const {
    return _tree;
}

SF_Build_QSM::SF_Build_QSM(const std::vector<Cylinder> &cylinders, int index) {
    initializeCylinderBuildingBricks(cylinders);
    initializeTree(index);
    initializeOctree();
    buildTree(_tree->getRootSegment());
}

void SF_Build_QSM::addBuildingBrickToSegment(const std::vector<int> &pointIdxNKNSearch, std::shared_ptr<SF_Model_Abstract_Segment> segment) {
    std::shared_ptr<SF_Model_Abstract_Buildingbrick> buildingBrick = _buildingBricks[pointIdxNKNSearch[0] + 1 ];
    if(buildingBrick->getLength() > _MINEQUALDISTANCE) {
        segment->addBuildingBrick(buildingBrick);
        buildTree(segment);
    }
}

void SF_Build_QSM::addChildBuildingbricks(std::shared_ptr<SF_Model_Abstract_Segment> segment, std::vector<int> pointIdxNKNSearch) {
    if(pointIdxNKNSearch.size()==1) {
        addBuildingBrickToSegment(pointIdxNKNSearch, segment);
    } else if(pointIdxNKNSearch.size()>1) {
        addBuildingBricksToChildSegments(pointIdxNKNSearch, segment);
    }
}

void SF_Build_QSM::addBuildingBricksToChildSegments(const std::vector<int> &pointIdxNKNSearch, std::shared_ptr<SF_Model_Abstract_Segment> segment) {
    for(size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
        std::shared_ptr<SF_Model_Abstract_Buildingbrick> buildingBrick = _buildingBricks[pointIdxNKNSearch[i] + 1 ];
        if(buildingBrick->getLength() > _MINEQUALDISTANCE) {
            std::shared_ptr<SF_Model_Abstract_Segment> segmentChild(new SF_Model_Abstract_Segment(_tree));
            segmentChild->addBuildingBrick(buildingBrick);
            segment->addChild(segmentChild);
            buildTree(segmentChild);
        }
    }
}

void SF_Build_QSM::buildTree(std::shared_ptr<SF_Model_Abstract_Segment> segment) {
    std::shared_ptr<SF_Model_Abstract_Buildingbrick> buildingBrick = segment->getBuildingBricks().back();
    SF_Point end (buildingBrick->getEnd()[0], buildingBrick->getEnd()[1], buildingBrick->getEnd()[2]);
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    if (_octree->radiusSearch( end, _MINEQUALDISTANCE, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        _octree->removeLeaf(end.x,end.y,end.z);
        addChildBuildingbricks(segment, pointIdxNKNSearch);
    }
}
