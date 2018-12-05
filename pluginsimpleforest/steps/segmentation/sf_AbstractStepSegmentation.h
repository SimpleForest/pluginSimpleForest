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

#ifndef SF_SEGMENTATION_STEP_H
#define SF_SEGMENTATION_STEP_H

#include "steps/filter/multiple/sf_abstractFilterMultipleStep.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

class SF_AbstractStepSegmentation : public SF_AbstractFilterMultipleStep {
  Q_OBJECT
public:
  SF_AbstractStepSegmentation(CT_StepInitializeData &dataInit);

protected:
  Eigen::Vector3d _centerOfMass;
  bool _first = true;
  void downscale(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL, float voxelSize,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLDownscaled);
  void initializeIndexVec(CT_ResultGroupIterator &resultGrpIterator2,
                          std::vector<CT_PointCloudIndexVector *> &indexVec);
  void initializeIndexVec(size_t size,
                          std::vector<CT_PointCloudIndexVector *> &indexVec);
  void createPCLCloud(const QString &clusterGrpStr, const QString &clusterStr,
                      CT_ResultGroup *outResult,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL,
                      std::vector<size_t> &indices, float factor);
  void fillIndexVec(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL,
                    std::vector<CT_PointCloudIndexVector *> &indexVec,
                    std::vector<size_t> &indices,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLDownscaled,
                    float maxRange);
  int getClusterNumber(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLDownscaled);
};

#endif // SF_SEGMENTATION_STEP_H
