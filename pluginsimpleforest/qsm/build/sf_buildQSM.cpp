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
  _octree.reset(new pcl::octree::OctreePointCloudSearch<SF_Point>(_RESOLUTION));
  m_cloud.reset(new SF_Cloud);
  for (size_t i = 1; i < _buildingBricksPtr.size(); i++) {
    std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick = _buildingBricksPtr[i];
    SF_Point point(buildingBrick->getStart()[0], buildingBrick->getStart()[1], buildingBrick->getStart()[2]);
    m_cloud->push_back(point);
  }

  _octree->setInputCloud(m_cloud);
  _octree->addPointsFromInputCloud();
}

void
SF_BuildQSM::initializeCylinderBuildingBricks(const std::vector<SF_QSMDetectionCylinder>& cylinders)
{
  _buildingBricksPtr.clear();
  for (size_t i = 0; i < cylinders.size(); i++) {
    SF_QSMDetectionCylinder cyl = cylinders[i];
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> cylinder(new Sf_ModelCylinderBuildingbrick(cyl._circleA, cyl._circleB));
    _buildingBricksPtr.push_back(cylinder);
  }
}

void
SF_BuildQSM::initializeTree(int index)
{
  _tree.reset(new SF_ModelQSM(index));
  std::shared_ptr<SF_ModelAbstractSegment> rootSegment(new SF_ModelAbstractSegment(_tree));
  rootSegment->addBuildingBrick(_buildingBricksPtr[0]);
  _tree->setRootSegment(rootSegment);
}

std::shared_ptr<SF_ModelQSM>
SF_BuildQSM::getTree() const
{
  return _tree;
}

SF_BuildQSM::SF_BuildQSM(const std::vector<SF_QSMDetectionCylinder>& cylinders, int index)
{
  initializeCylinderBuildingBricks(cylinders);
  initializeTree(index);
  initializeOctree();
  buildTree(_tree->getRootSegment());
}

void
SF_BuildQSM::addBuildingBrickToSegment(const std::vector<int>& pointIdxNKNSearch, std::shared_ptr<SF_ModelAbstractSegment> segment)
{
  std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick = _buildingBricksPtr[pointIdxNKNSearch[0] + 1];
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
    std::shared_ptr<Sf_ModelAbstractBuildingbrick> buildingBrick = _buildingBricksPtr[pointIdxNKNSearch[i] + 1];
    if (buildingBrick->getLength() > _MINLENGTH) {
      std::shared_ptr<SF_ModelAbstractSegment> segmentChild(new SF_ModelAbstractSegment(_tree));
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
  SF_Point end(buildingBrick->getEnd()[0], buildingBrick->getEnd()[1], buildingBrick->getEnd()[2]);
  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;
  if (_octree->radiusSearch(end, _MINEQUALDISTANCE, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    _octree->removeLeaf(end.x, end.y, end.z);
    for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
      _octree->removeLeaf(
        m_cloud->points[pointIdxNKNSearch[i]].x, m_cloud->points[pointIdxNKNSearch[i]].y, m_cloud->points[pointIdxNKNSearch[i]].z);
    }
    addChildBuildingbricks(segment, pointIdxNKNSearch);
  }
}
