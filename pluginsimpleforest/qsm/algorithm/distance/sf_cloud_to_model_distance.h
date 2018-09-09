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

#ifndef SF_CLOUD_TO_MODEL_DISTANCE_H
#define SF_CLOUD_TO_MODEL_DISTANCE_H

#include <pcl/kdtree/kdtree_flann.h>
#include "qsm/sf_model_tree.h"

enum SFCLoudToModelDistanceMethod {ZEROMOMENTUMORDER,
                                   FIRSTMOMENTUMORDERMSAC,
                                   FIRSTMOMENTUMORDER,
                                   SECONDMOMENTUMORDERMSAC,
                                   SECONDMOMENTUMORDER};

class SfCloudToModelDistance {
    const SFCLoudToModelDistanceMethod _METHOD;
    const float _INLIERDISTANCE;
    float _k;
    float _averageDistance;
    std::shared_ptr<SF_Model_Tree> _tree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr _kdtreeQSM;

    const float getDistance(const pcl::PointXYZ &point, std::shared_ptr<SF_Model_Abstract_Buildingbrick> buildingBrick);
    const std::vector<float> getCloudToModelDistances();
    const float getNumberInliers(const std::vector<float> &distances);
    const float adaptDistanceToMethod(float distance);
    void initializeKdTree();
    void compute();

public:
    SfCloudToModelDistance(std::shared_ptr<SF_Model_Tree> tree,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           SFCLoudToModelDistanceMethod &method,
                           float inlierDistance,
                           int k);
};

#endif // SF_CLOUD_TO_MODEL_DISTANCE_H
