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

#include "sf_modelAbstractSegment.h"

class SF_ModelQSM
{
  int _ID;
  std::string _species;
  std::shared_ptr<SF_ModelAbstractSegment> _rootSegment;

public:
  SF_ModelQSM(const int ID);
  virtual std::string toString();
  virtual std::string toHeaderString();

  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> getSegments();
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> getSegments(std::shared_ptr<SF_ModelAbstractSegment> segment);
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> getBuildingBricks();
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> getLeaveSegments();
  std::shared_ptr<SF_ModelAbstractSegment> getRootSegment() const;
  void setRootSegment(const std::shared_ptr<SF_ModelAbstractSegment>& rootSegment);
};

#endif // SF_MODEL_TREE_H
