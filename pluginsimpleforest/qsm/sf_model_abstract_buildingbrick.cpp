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

#include "sf_model_abstract_buildingbrick.h"

float SF_Model_Abstract_Buildingbrick::getGrowthLength() {
    float  growthLength = getLength();
    std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> >  children = getChildren();
    for(size_t i = 0; i < children.size(); i++) {
        std::shared_ptr<SF_Model_Abstract_Buildingbrick> child = children.at(i);
        growthLength += child->getGrowthLength();
    }
    return growthLength;
}

float SF_Model_Abstract_Buildingbrick::getGrowthVolume() {
    float  growthVolume = getVolume();
    std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > children = getChildren();
    for(size_t i = 0; i < children.size(); i++) {
        std::shared_ptr<SF_Model_Abstract_Buildingbrick> child = children.at(i);
        growthVolume += child->getGrowthVolume();
    }
    return growthVolume;
}

size_t SF_Model_Abstract_Buildingbrick::getIndex() const {
    return _indexVector;
}

void SF_Model_Abstract_Buildingbrick::setIndex(const size_t &index) {
    _indexVector = index;
}

size_t SF_Model_Abstract_Buildingbrick::getID() const {
    return _ID;
}

std::shared_ptr<SF_Model_Abstract_Segment> SF_Model_Abstract_Buildingbrick::getSegment() {
    return _segment.lock();
}

void SF_Model_Abstract_Buildingbrick::setSegment(std::shared_ptr<SF_Model_Abstract_Segment> segment) {
    _segment = segment;
}

void SF_Model_Abstract_Buildingbrick::setID(const size_t &ID) {
    _ID = ID;
}

Eigen::Vector3f SF_Model_Abstract_Buildingbrick::getStart() const {
    return _start;
}

Eigen::Vector3f SF_Model_Abstract_Buildingbrick::getEnd() const {
    return _end;
}

std::shared_ptr<SF_Model_Abstract_Buildingbrick> SF_Model_Abstract_Buildingbrick::getParent() {
    std::shared_ptr<SF_Model_Abstract_Buildingbrick> parent = segment->getParentBuildingBrick(_indexVector);
    return parent;
}

std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > SF_Model_Abstract_Buildingbrick::getChildren() {
    std::shared_ptr<SF_Model_Abstract_Segment> segment = getSegment();
    return segment->getChildBuildingBricks(_indexVector);
}

SF_Model_Abstract_Buildingbrick::SF_Model_Abstract_Buildingbrick() {

}
