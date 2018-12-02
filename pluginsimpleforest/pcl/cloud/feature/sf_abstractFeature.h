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

#include "pcl/cloud/sf_abstractCloud.h"

template <typename PointType, typename FeatureType>
class SF_AbstractFeature:
        public  SF_AbstractCloud<PointType> {
protected:
    typename pcl::PointCloud<FeatureType>::Ptr _featuresOut;

public:
    SF_AbstractFeature(typename pcl::PointCloud<PointType>::Ptr cloudIn,
                       typename pcl::PointCloud<FeatureType>::Ptr featuresOut);
    virtual void computeFeatures() = 0;
    typename pcl::PointCloud<FeatureType>::Ptr get_features() const {
        return _featuresOut;
    }
};

#include "sf_abstractFeature.hpp"

#endif // SF_ABSTRACT_FEATURE_H
