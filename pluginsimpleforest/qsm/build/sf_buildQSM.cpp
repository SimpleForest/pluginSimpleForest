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

#include "sf_buildQSM.h"

#include "qsm/model/sf_modelCylinderBuildingbrick.h"

#include <memory>
#include <pcl/filters/voxel_grid.h>

void
SF_BuildQSM::initializeOctree()
{
  m_octree.reset(new pcl::octree::OctreePointCloudSearch<SF_Point>(_RESOLUTION));
  m_cloud.reset(new SF_Cloud);
  for (size_t i = 1; i < m_buildingBricks.size(); i++) {
    std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick = m_buildingBricks[i];
    const auto start = buildingBrick->getStart();
    SF_Point point(start[0], start[1], start[2]);
    m_cloud->push_back(std::move(point));
  }
  m_octree->setInputCloud(m_cloud);
  m_octree->addPointsFromInputCloud();
}

void
SF_BuildQSM::initializeCylinderBuildingBricks(const std::vector<SF_QSMDetectionCylinder>& cylinders)
{
  m_buildingBricks.clear();
  for (size_t i = 0; i < cylinders.size(); i++) {
    SF_QSMDetectionCylinder cyl = cylinders[i];
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(cyl._circleA, cyl._circleB));
    m_buildingBricks.push_back(cylinder);
  }
}

void
SF_BuildQSM::initializeTree(int index)
{
  m_tree.reset(new SF_ModelQSM(index));
  std::shared_ptr<SF_ModelAbstractSegment> rootSegment(new SF_ModelAbstractSegment(m_tree));
  rootSegment->addBuildingBrick(m_buildingBricks[0]);
  m_tree->setRootSegment(rootSegment);
}

std::shared_ptr<SF_ModelQSM>
SF_BuildQSM::getTree() const
{
  return m_tree;
}

SF_BuildQSM::SF_BuildQSM(const std::vector<SF_QSMDetectionCylinder>& cylinders, int index)
{
  initializeCylinderBuildingBricks(cylinders);
  initializeTree(index);
  initializeOctree();
  buildTree(m_tree->getRootSegment());
}

void
SF_BuildQSM::addBuildingBrickToSegment(const std::vector<int>& pointIdxNKNSearch, std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick = m_buildingBricks[pointIdxNKNSearch[0] + 1];
  if (buildingBrick->getLength() > _MINLENGTH) {
    segment->addBuildingBrick(buildingBrick);
    buildTree(segment);
  }
}

void
SF_BuildQSM::addChildBuildingbricks(std::shared_ptr<SF_ModelAbstractSegment> segment, std::vector<int> pointIdxNKNSearch)
{
  if (pointIdxNKNSearch.size() == 1) {
    addBuildingBrickToSegment(pointIdxNKNSearch, segment);
  } else if (pointIdxNKNSearch.size() > 1) {
    addBuildingBricksToChildSegments(pointIdxNKNSearch, segment);
  }
}

void
SF_BuildQSM::addBuildingBricksToChildSegments(const std::vector<int>& pointIdxNKNSearch,
                                              std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
    std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick = m_buildingBricks[pointIdxNKNSearch[i] + 1]; // TODO why + 1
    if (buildingBrick->getLength() > _MINLENGTH) {
      std::shared_ptr<SF_ModelAbstractSegment> segmentChild(new SF_ModelAbstractSegment(m_tree));
      segmentChild->addBuildingBrick(buildingBrick);
      segment->addChild(segmentChild);
      buildTree(segmentChild);
    }
  }
}

void
SF_BuildQSM::buildTree(std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick = segment->getBuildingBricks().back();
  const auto endVec3f = buildingBrick->getEnd();
  SF_Point end(endVec3f[0], endVec3f[1], endVec3f[2]);
  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;
  if (m_octree->radiusSearch(end, _MINEQUALDISTANCE, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    m_octree->removeLeaf(end.x, end.y, end.z);
    for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
      m_octree->removeLeaf(
        m_cloud->points[pointIdxNKNSearch[i]].x, m_cloud->points[pointIdxNKNSearch[i]].y, m_cloud->points[pointIdxNKNSearch[i]].z);
    }
    addChildBuildingbricks(segment, pointIdxNKNSearch);
  }
}
