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

#include "sf_model_abstract_segment.h"
#include "pcl/sf_math.h"

void SF_Model_Abstract_Segment::setParent(const std::weak_ptr<SF_Model_Abstract_Segment> &parent) {
    _parent = parent;
}

std::shared_ptr<SF_Model_Abstract_Segment> SF_Model_Abstract_Segment::getParent() {
    return _parent.lock();
}

std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > SF_Model_Abstract_Segment::getBuildingBricks() const {
    return _buildingBricks;
}

std::string SF_Model_Abstract_Segment::toString() {
    std::string str(_ID);
    str.append(", ");
    std::shared_ptr<SF_Model_Abstract_Segment> parent = getParent();
    if(parent == nullptr) {
        str.append("-1");
    } else {
        str.append(parent->getID());
    }
    str.append(", ");
    str.append(getStart()[0]);
    str.append(", ");
    str.append(getStart()[1]);
    str.append(", ");
    str.append(getStart()[2]);
    str.append(", ");
    str.append(getEnd()[0]);
    str.append(", ");
    str.append(getEnd()[1]);
    str.append(", ");
    str.append(getEnd()[2]);
    str.append(", ");
    str.append(getRadius());
    str.append(", ");
    str.append(getVolume());
    str.append(", ");
    str.append(_buildingBricks[_buildingBricks.size()/2]->getGrowthVolume());
    str.append(", ");
    str.append(getLength());
    str.append(", ");
    str.append(_buildingBricks[_buildingBricks.size()/2]->getGrowthLength());
    str.append(std::endl);
    return str;
}

std::string SF_Model_Abstract_Segment::toHeaderString() {
    std::string str ("segmentID, parentSegmentID, segmentStartX, segmentStartY, segmentStartZ, segmentEndX, segmentEndY, segmentEndZ, segmentRadius, segmentVolume, segmentGrowthVolume, segmentLength, segmentGrowthLength");
    str.append(std::endl);
    return str;
}

Eigen::Vector3f SF_Model_Abstract_Segment::getStart() const {
    Eigen::Vector3f start;
    if(!_buildingBricks.empty()) {
        start = _buildingBricks[0]->getStart();
    }
    return start;
}

Eigen::Vector3f SF_Model_Abstract_Segment::getEnd() const {
    Eigen::Vector3f end;
    if(!_buildingBricks.empty()) {
        end = _buildingBricks[_buildingBricks.size()-1]->getEnd();
    }
    return end;
}

float SF_Model_Abstract_Segment::getRadius() const {
    std::vector<float> radii;
    for(size_t i = 0; i < _buildingBricks.size(); i++) {
        radii.push_back(_buildingBricks[i]->getRadius());
    }
    return SF_Math<float>::getMedian(radii);
}

float SF_Model_Abstract_Segment::getVolume() const {
    float volume = 0;
    for(size_t i = 0; i < _buildingBricks.size(); i++) {
        volume += _buildingBricks[i]->getVolume();
    }
    return volume;
}

float SF_Model_Abstract_Segment::getLength() const {
    float length = 0;
    for(size_t i = 0; i < _buildingBricks.size(); i++) {
        length += _buildingBricks[i]->getLength();
    }
    return length;
}

int SF_Model_Abstract_Segment::getID() const {
    return _ID;
}

void SF_Model_Abstract_Segment::setID(int ID) {
    _ID = ID;
}

std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > SF_Model_Abstract_Segment::getChildSegments() const
{
    return _childSegments;
}

int SF_Model_Abstract_Segment::getParentID() {
    std::shared_ptr<SF_Model_Abstract_Segment> parent = getParent();
    if(parent == nullptr) {
        return -1;
    } else {
        return parent->getID();
    }
}

void SF_Model_Abstract_Segment::addChild(std::shared_ptr<SF_Model_Abstract_Segment> child) {
    child->setParent(shared_from_this());
    _childSegments.push_back(child);
}

void SF_Model_Abstract_Segment::addBuildingBrick(std::shared_ptr<SF_Model_Abstract_Buildingbrick> buildingBrick) {
    buildingBrick->setIndex(_buildingBricks.size());
    _buildingBricks.push_back(buildingBrick);
}

std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > SF_Model_Abstract_Segment::getChildBuildingBricks(const size_t index) {
    std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > children;
    if(index < _buildingBricks.size()-1 ) {
        children.push_back(_buildingBricks.at(index + 1));
    } else if(index == _buildingBricks.size()-1 ) {
        for(size_t i = 0; i < _childSegments.size(); i ++) {
            std::shared_ptr<SF_Model_Abstract_Buildingbrick> child = _childSegments.at(i).getBuildingBricks()[0];
            children.push_back(child);
        }
    }
    return children;
}

std::shared_ptr<SF_Model_Abstract_Buildingbrick>  SF_Model_Abstract_Segment::getParentBuildingBrick(const size_t index) {
    std::shared_ptr<SF_Model_Abstract_Buildingbrick> parent;
    if(index == 0) {
        std::shared_ptr<SF_Model_Abstract_Segment> parentSegment = getParent();
        if(parentSegment!=nullptr) {
            parent = parentSegment->getBuildingBricks()[parentSegment->getBuildingBricks().size()-1];
        }
    } else {
        parent = _buildingBricks[index-1];
    }
    return parent;
}

SF_Model_Abstract_Segment::SF_Model_Abstract_Segment() {

}
