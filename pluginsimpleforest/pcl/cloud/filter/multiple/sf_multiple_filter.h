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

#ifndef SF_MULTIPLE_FILTER_H
#define SF_MULTIPLE_FILTER_H

#include "pcl/cloud/filter/sf_abstract_filter.h"
#include <pcl/cloud/sf_abstract_cloud.h>

template <typename PointType>
class Sf_Multiple_Filter: public  SF_Abstract_Filter<PointType> {

private:
    typename pcl::PointCloud<PointType>::Ptr  _downScaledCloud;
    typename std::vector<pcl::PointCloud<PointType>::Ptr> _clouds;

protected:
    virtual void reset();
    virtual void create_indices();

public:
    Sf_Multiple_Filter();
};

#include "sf_multiple_filter.hpp"

#endif // SF_MULTIPLE_FILTER_H