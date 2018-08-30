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

#ifndef SF_MODEL_ABSTRACT_SEGMENT_H
#define SF_MODEL_ABSTRACT_SEGMENT_H

#include "sf_model_abstract_buildingbrick.h"
#include "vector"

class SF_Model_Abstract_Segment: std::enable_shared_from_this<SF_Model_Abstract_Segment> {
    std::weak_ptr<SF_Model_Abstract_Segment> _parent;
    std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > _buildingBricks;
    std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > _childSegments;
protected:
    int _ID;
    int getParentID();
    std::shared_ptr<SF_Model_Abstract_Segment> getFirstChild();
public:
    void addChild(std::shared_ptr<SF_Model_Abstract_Segment> child);
    void addBuildingBrick(std::shared_ptr<SF_Model_Abstract_Buildingbrick> buildingBrick);
    std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > getChildBuildingBricks(const size_t index);
    std::shared_ptr<SF_Model_Abstract_Buildingbrick> getParentBuildingBrick(const size_t index);
    SF_Model_Abstract_Segment();    
    std::shared_ptr<SF_Model_Abstract_Segment> getParent();
    void setParent(const std::weak_ptr<SF_Model_Abstract_Segment> &parent);
    std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > getBuildingBricks() const;
    virtual std::string toString();
    virtual std::string toHeaderString();
    Eigen::Vector3f getStart() const;
    Eigen::Vector3f getEnd() const;
    float getRadius() const;
    float getVolume() const;
    float getLength() const;
    int getID() const;
    void setID(int ID);
    std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > getChildSegments() const;
};

#endif // SF_MODEL_ABSTRACT_SEGMENT_H
