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

#include "sf_modelAbstractBuildingbrick.h"
#include "vector"

class SF_ModelQSM;

class SF_ModelAbstractSegment : public std::enable_shared_from_this<SF_ModelAbstractSegment>
{
  std::weak_ptr<SF_ModelAbstractSegment> m_parent;
  std::weak_ptr<SF_ModelQSM> m_tree;
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> m_buildingBricks;
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> m_children;

protected:
  int m_ID;
  int m_branchOrder;
  int m_reverseBranchOrder;
  int m_reverseSummedBranchOrder;
  int m_reversePipeBranchOrder;
  int m_branchID;
  int getParentID();
  std::shared_ptr<SF_ModelAbstractSegment> getFirstChild();

public:
  enum class SF_SORTTYPE {GROWTH_LENGTH, GROWTH_VOLUME, RADIUS};
  SF_ModelAbstractSegment(std::shared_ptr<SF_ModelQSM> tree);
  void addChild(std::shared_ptr<SF_ModelAbstractSegment> child);
  void addBuildingBrick(std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick);
  virtual std::string toString();
  virtual std::string toHeaderString();
  Eigen::Vector3f getStart() const;
  Eigen::Vector3f getEnd() const;
  Eigen::Vector3f getAxis() const;
  void remove();
  float getRadius() const;
  float getVolume() const;
  float getGrowthVolume() const;
  float getGrowthLength() const;
  float getLength() const;
  bool isRoot();
  void computeBranchOrder(int branchOrder = 0);
  void computeReverseBranchOrder(int branchOrder = 1);
  void computeReverseSummedBranchOrder(int branchOrder = 1);
  void computeReversePipeBranchOrder(int branchOrder = 1);
  void initializeOrder();
  std::shared_ptr<SF_ModelAbstractSegment> getParent();
  void setParent(const std::weak_ptr<SF_ModelAbstractSegment>& parent);
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> getBuildingBricks() const;
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> getChildren() const;
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> getSiblings();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> getChildBuildingBricks(const size_t index);
  std::shared_ptr<Sf_ModelAbstractBuildingbrick> getParentBuildingBrick(const size_t index);
  std::shared_ptr<SF_ModelQSM> getTree() const;
  void setChildSegments(const std::vector<std::shared_ptr<SF_ModelAbstractSegment>> childSegments);
  void setBuildingBricks(const std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>>& buildingBricks);
  void sort(const SF_SORTTYPE sortType);
  int getBranchOrder() const;
  void setBranchOrder(int branchOrder);
  int getReverseBranchOrder() const;
  void setReverseBranchOrder(int reverseBranchOrder);
  int getReversePipeBranchOrder() const;
  void setReversePipeBranchOrder(int reversePipeBranchOrder);
  int getID() const;
  void setID(int ID);
  int getReverseSummedBranchOrder() const;
  void setReverseSummedBranchOrder(int reverseSummedBranchOrder);
};

#endif // SF_MODEL_ABSTRACT_SEGMENT_H
