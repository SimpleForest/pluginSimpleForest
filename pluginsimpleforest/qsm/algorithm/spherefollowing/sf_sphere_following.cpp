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

#include "sf_sphere_following.h"


void SF_SphereFollowing::initializeCloud() {
    _cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    for(size_t i = 0 ; i < _clusters.size(); i++) {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster = _clusters[i];
        for(size_t j = 0; j < cluster->points.size(); j++) {
            pcl::PointXYZINormal point = cluster->points[j];
            point.intensity = i;
            _cloud->push_back(point);
        }
    }
}

SF_SphereFollowing::SF_SphereFollowing(SF_SphereFollowingParameters params,
                                       std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clusters):
_params(params),
_clusters(clusters){
    _optimParams = _params._optimizationParams;
    if(_optimParams.size() != clusters.size()) {
        throw std::runtime_error("SF_SphereFollowing critical error with unformatted input parameters.");
    }
    initializeCloud();
}

const std::shared_ptr<SF_ModelQSM> SF_SphereFollowing::getQSM() {
    return _qsm;
}
