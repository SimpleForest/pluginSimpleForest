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

#ifndef SF_EUCLIDEAN_CLUSTERING_H
#define SF_EUCLIDEAN_CLUSTERING_H

#include "pcl/cloud/filter/multiple/sf_multiple_filter.h"

template <typename PointType>
class Sf_Euclidean_Clustering: public  Sf_Multiple_Filter<PointType> {
    SF_Param_Euclidean_Clustering _params;
    extractClusterIndices(std::vector<pcl::PointIndices> & clusterIndices);
    extractClouds(const std::vector<pcl::PointIndices> & clusterIndices);
    convertAndDownScale();
protected:

public:
    Sf_Euclidean_Clustering();
    compute(SF_Param_Euclidean_Clustering& params);
};

#include "sf_euclidean_clustering.hpp"

#endif // SF_EUCLIDEAN_CLUSTERING_H
