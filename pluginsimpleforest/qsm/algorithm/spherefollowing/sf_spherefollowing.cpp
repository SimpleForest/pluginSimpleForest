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

#include "sf_spherefollowing.h"

#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>

void SF_SphereFollowing::initializeCloud() {
//  _cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
//  for (size_t i = 0; i < _clusters.size(); i++) {
//    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster = _clusters[i];
//    for (size_t j = 0; j < cluster->points.size(); j++) {
//      pcl::PointXYZINormal point = cluster->points[j];
//      point.intensity = i;
//      _cloud->push_back(point);
//    }
//  }
}

SF_SphereFollowing::SF_SphereFollowing(
    SF_SphereFollowingParameters params,
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clusters){
//    : _params(params), _clusters(clusters)
//  _optimParams = _params._optimizationParams;
//  if (_optimParams.size() != clusters.size()) {
//    throw std::runtime_error(
//        "SF_SphereFollowing critical error with unformatted input parameters.");
//  }
//  initializeCloud();
}

const std::shared_ptr<SF_ModelQSM> SF_SphereFollowing::getQSM() { return _qsm; }

void SF_SphereFollowing::initialize()
{
//    _octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZINormal>);
//    _octree->setInputCloud(_cloud);
//    _octree->addPointsFromInputCloud();
//    float _min = std::numeric_limits<float>::max();
//    std::for_each(_cloud->points.begin(), _cloud->points.end(), [&_min](const pcl::PointXYZINormal &point){
//        if(point.z < _min) _min = point.z;
//    } );
//    float _max = _min + _params._heightInitializationSlice;
//    pcl::PointCloud<pcl::PointXYZINormal>::Ptr lowestSlice (new pcl::PointCloud<pcl::PointXYZINormal>);
//    pcl::ConditionAnd<pcl::PointXYZINormal>::Ptr rangeCond (new
//      pcl::ConditionAnd<pcl::PointXYZINormal> ());
//    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr (new
//      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, _min)));
//    rangeCond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr (new
//      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, _max)));
//    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
//    condrem.setCondition (rangeCond);
//    condrem.setInputCloud (_cloud);
//    condrem.setKeepOrganized(true);
//    condrem.filter (*lowestSlice);



}
