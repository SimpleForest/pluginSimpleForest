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
#ifndef SF_NORMAL_H
#define SF_NORMAL_H


#include "pcl/cloud/feature/sf_abstract_feature.h"

template <typename PointType, typename FeatureType>
class SF_Normal: public  SF_Abstract_Feature<PointType, FeatureType> {
private:
    int _k = 5;
    bool _use_range = false;
    float _range = 0.03f;
public:
    SF_Normal::SF_Normal(typename pcl::PointCloud<PointType>::Ptr cloud_in, typename pcl::PointCloud<FeatureType>::Ptr features_out);
    virtual void compute_features();
    virtual void compute_features_range();
    virtual void compute_features_knn();
    void set_parameters(float range, bool use_range = true);
    void set_parameters(int k, bool use_range = false);
}


#include "sf_normal.hpp"
#endif // SF_NORMAL_H
