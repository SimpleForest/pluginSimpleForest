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

#include "sf_model_tree.h"

std::shared_ptr<SF_Model_Abstract_Segment> SF_Model_Tree::getRootSegment() const {
    return _rootSegment;
}

void SF_Model_Tree::setRootSegment(const std::shared_ptr<SF_Model_Abstract_Segment> &rootSegment) {
    _rootSegment = rootSegment;
}

SF_Model_Tree::SF_Model_Tree(int ID):_ID(ID), _species("unknownSpecies") {

}

std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > SF_Model_Tree::getSegments() {
    std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > segments;
    if(_rootSegment!=nullptr) {
         segments = getSegments(_rootSegment);
    }
    return segments;
}

std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > SF_Model_Tree::getSegments(std::shared_ptr<SF_Model_Abstract_Segment> segment) {
    std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > segments;
    segments.push_back(segment);
    for(size_t i = 0; i < segments->getChildSegments().size(); i++) {
        std::shared_ptr<SF_Model_Abstract_Segment> child = segments->getChildSegments()[i];
        std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > childSegments = getSegments(child);
        segments.insert( segments.end(), childSegments.begin(), childSegments.end() );
    }
    return segments;
}

std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > SF_Model_Tree::getBuildingBricks() {
    std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > segments = getSegments();
    std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > buildingBricks;
    for(size_t i = 0; i < segments.size(); i++) {
        std::shared_ptr<SF_Model_Abstract_Segment> segment = segments[i];
        std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > segmentBuildingBricks = segment->getBuildingBricks();
        for(size_t j = 0; j < segmentBuildingBricks.size(); j++) {
            std::shared_ptr<SF_Model_Abstract_Buildingbrick> buildingBrick = segmentBuildingBricks[j];
            buildingBricks.push_back(buildingBrick);
        }
    }
    return buildingBricks;
}
