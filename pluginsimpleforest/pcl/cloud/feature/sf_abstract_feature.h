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
#ifndef SF_ABSTRACT_FEATURE_H
#define SF_ABSTRACT_FEATURE_H
#include "pcl/cloud/sf_abstract_cloud.h"
template <typename PointType, typename FeatureType>
class SF_Abstract_Feature: public  SF_Abstract_Cloud<PointType> {
    pcl::PointCloud<FeatureType> _features_out;

public:
    SF_Abstract_Feature(typename pcl::PointCloud<PointType>::Ptr cloud_in, typename pcl::PointCloud<FeatureType>::Ptr features_out);
    virtual void compute_features() = 0;
    pcl::PointCloud<FeatureType> get_features() const {
        return _features_out;
    }
};
#include "sf_abstract_feature.hpp"
#endif // SF_ABSTRACT_FEATURE_H
