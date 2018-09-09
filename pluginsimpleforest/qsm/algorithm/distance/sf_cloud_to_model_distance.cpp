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

#include "sf_cloud_to_model_distance.h"
#include "pcl/sf_math.h"

void SfCloudToModelDistance::initializeKdTree() {
     _kdtreeQSM.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
     pcl::PointCloud<pcl::PointXYZ>::Ptr centerCloud(new pcl::PointCloud<pcl::PointXYZ>() );
     std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > buildingBricks = _tree->getBuildingBricks();
     for(size_t i = 0; i < buildingBricks.size(); i++) {
         Eigen::Vector3f pointEigen = buildingBricks[i].getCenter();
         pcl::PointXYZ   point(pointEigen[0], pointEigen[1], pointEigen[2]);
         centerCloud->push_back(point);
     }
     _kdtreeQSM->setInputCloud(centerCloud);
}

const float SfCloudToModelDistance::adaptDistanceToMethod(float distance) {
    switch (_METHOD) {
    case SFCLoudToModelDistanceMethod::ZEROMOMENTUMORDER:
        if(distance < _INLIERDISTANCE) {
            distance = 1;
        } else {
            distance = 0;
        }
        break;
    case SFCLoudToModelDistanceMethod::FIRSTMOMENTUMORDER:
        distance = std::abs(distance);
        break;
    case SFCLoudToModelDistanceMethod::FIRSTMOMENTUMORDERMSAC:
        distance = std::abs(distance);
        distance = std::max(_INLIERDISTANCE, distance);
        break;
    case SFCLoudToModelDistanceMethod::SECONDMOMENTUMORDER:
        distance = distance*distance;
        break;
    case SFCLoudToModelDistanceMethod::SECONDMOMENTUMORDERMSAC:
        distance = std::abs(distance);
        distance = std::max(_INLIERDISTANCE, distance);
        distance = distance*distance;
        break;
    default:
        break;
    }
    return distance;
}

const float SfCloudToModelDistance::getDistance(const pcl::PointXYZ &point, std::shared_ptr<SF_Model_Abstract_Buildingbrick> buildingBrick) {
    float distance = buildingBrick->getDistance(point);
    distance = adaptDistanceToMethod(distance);
    return distance;
}

void SfCloudToModelDistance::compute() {
    std::vector<float> distances = getCloudToModelDistances();
    switch (_METHOD) {
    case SFCLoudToModelDistanceMethod::ZEROMOMENTUMORDER:
        _averageDistance = getNumberInliers(distances);
        break;
    case SFCLoudToModelDistanceMethod::FIRSTMOMENTUMORDER:
        _averageDistance = SF_Math<float>::getMean(distances);
        break;
    case SFCLoudToModelDistanceMethod::FIRSTMOMENTUMORDERMSAC:
        _averageDistance = SF_Math<float>::getMean(distances);
        break;
    case SFCLoudToModelDistanceMethod::SECONDMOMENTUMORDER:
        _averageDistance = std::sqrt(SF_Math<float>::getMean(distances));
        break;
    case SFCLoudToModelDistanceMethod::SECONDMOMENTUMORDERMSAC:
        _averageDistance = std::sqrt(SF_Math<float>::getMean(distances));
        break;
    default:
        break;
    }
}

const std::vector<float> SfCloudToModelDistance::getCloudToModelDistances() {
    std::vector<float> distances;
    std::vector<std::shared_ptr<SF_Model_Abstract_Buildingbrick> > buildingBricks =  getBuildingBricks();
    for(size_t i = 0; i < _cloud->points.size(); i++) {
        pcl::PointXYZ point = _cloud->points[i];
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( _kdtreeQSM->nearestKSearch(point, _k, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
            float minDistance = std::numeric_limits<float>::max();
            for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j) {
                std::shared_ptr<SF_Model_Abstract_Buildingbrick>  neighboringBrick = buildingBricks[ pointIdxRadiusSearch[j] ];
                float distance = getDistance(point, neighboringBrick);
                if(distance < minDistance) minDistance = distance;
            }
            distances.push_back(minDistance);
        } else {                    .
            distances.push_back(adaptDistanceToMethod(_INLIERDISTANCE));
        }
    }
}

const float SfCloudToModelDistance::getNumberInliers(const std::vector<float> &distances) {
    float sum = 0;
    for(size_t i = 0; i < distances.size(); i++) {
        sum+= distances[i];
    }
    return sum;
}

SfCloudToModelDistance::SfCloudToModelDistance(std::shared_ptr<SF_Model_Tree> tree,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                               SFCLoudToModelDistanceMethod &method,
                                               float inlierDistance,
                                               int k):
    _tree(tree), _cloud(cloud), _METHOD(method), _INLIERDISTANCE(inlierDistance), _k(k) {
    _k = 5;
    _averageDistance = std::numeric_limits<float>::max();
    _INLIERDISTANCE = 0.05;
    initializeKdTree();
    compute();
}
