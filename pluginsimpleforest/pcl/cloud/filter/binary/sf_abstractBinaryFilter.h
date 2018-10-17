/****************************************************************************

 Copyright (C) 2017-2017 Jan Hackenberg, free software developer
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

#ifndef SF_BINARY_FILTER_H
#define SF_BINARY_FILTER_H

#include "pcl/cloud/filter/sf_abstractFilter.h"
#include <pcl/cloud/sf_abstractCloud.h>

template <typename PointType>
class Sf_AbstractBinaryFilter: public  SF_AbstractFilterDeprecated<PointType> {

protected:
    typename pcl::PointCloud<PointType>::Ptr _cloudOutFilteredNoise;
    virtual void reset();
    virtual void createIndex(PointType point,
                      float sqrd_distance);

public:
    Sf_AbstractBinaryFilter();
    typename pcl::PointCloud<PointType>::Ptr getCloudOutFilteredNoise() const;
    double getPercentage();
};

#include "sf_abstractBinaryFilter.hpp"

#endif // SF_BINARY_FILTER_H


