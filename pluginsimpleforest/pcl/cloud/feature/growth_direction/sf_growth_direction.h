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
#ifndef SF_GROWTH_DIRECTION_H
#define SF_GROWTH_DIRECTION_H

#include "pcl/cloud/feature/sf_abstract_feature.h"
#include "../pca/sf_pca.h"

//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
template <typename PointType, typename FeatureType>
class SF_Growth_Direction: public  SF_Abstract_Feature<PointType, FeatureType> {
private:

    float _range_normal = 0.1f;
    float _range_gd = 0.3f;
    typename pcl::KdTree<PointType>::Ptr _kd_tree;
    static constexpr float MAX_LAMBDA3 = 0.9;
    void add_normals(std::vector<PCA_Values>& values);
    void add_growth_direction(std::vector<PCA_Values>& values);
    virtual void create_indices(){; }
    virtual void create_index(PointType point,
                         float sqrd_distance) {
        std::cout << "FOOO SF_Growth_Direction add_growth_direction 4ax " << " ; " << _range_gd << std::endl;; }
    virtual void reset(){_features_out.reset(new pcl::PointCloud<FeatureType>);}


public:
    SF_Growth_Direction(typename pcl::PointCloud<PointType>::Ptr cloud_in, typename pcl::PointCloud<FeatureType>::Ptr features_out);
    void compute_features();
    std::vector<PCA_Values> compute_normal_pca();
    void set_parameters(float range_normal, float range_gd);
};
#include "sf_growth_direction.hpp"
#endif // SF_GROWTH_DIRECTION_H
