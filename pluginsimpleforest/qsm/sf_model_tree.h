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

#ifndef SF_MODEL_TREE_H
#define SF_MODEL_TREE_H

#include "sf_model_abstract_segment.h"

class SF_Model_Tree {
    std::string _species;
    int _ID;
    std::shared_ptr<SF_Model_Abstract_Segment> _rootSegment;

public:
    SF_Model_Tree(int ID);
    virtual std::string toString();
    virtual std::string toHeaderString();

    std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > getSegments();
    std::vector<std::shared_ptr<SF_Model_Abstract_Segment> > getSegments(std::shared_ptr<SF_Model_Abstract_Segment> segment);
    std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > getBuildingBricks();
    std::shared_ptr<SF_Model_Abstract_Segment> getRootSegment() const;
    void setRootSegment(const std::shared_ptr<SF_Model_Abstract_Segment> &rootSegment);
};

#endif // SF_MODEL_TREE_H
