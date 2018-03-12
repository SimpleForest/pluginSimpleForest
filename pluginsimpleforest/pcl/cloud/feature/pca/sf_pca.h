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
#ifndef SF_PCA_H
#define SF_PCA_H


//#include "pcl/cloud/feature/sf_abstract_feature.h"

//template <typename PointType>
//class SF_PCA: public  SF_Abstract_Feature<PointType> {
//private:
//    float _range;
//    bool _use_range;
//    int _k;
//    void extract_neighbors(pcl::search::KdTree<PointType>::Ptr kd_tree, PointType p, pcl::PointCloud<PointType>::Ptr neighborhood);
//    void extract_neighbors_by_index(std::vector<int> pointIndex, pcl::PointCloud<PointType>::Ptr neighborhood) ;
//    extract_neighbors_by_range(pcl::search::KdTree<PointType>::Ptr kdtree, PointType p, pcl::PointCloud<PointType>::Ptr neighborhood);
//    extract_neighbors_by_knn(pcl::search::KdTree<PointType>::Ptr kdtree, PointType p, pcl::PointCloud<PointType>::Ptr neighborhood);
//public:
//    SF_Normal::SF_Normal(typename pcl::PointCloud<PointType>::Ptr cloud_in,  PointCloudSF_PCA::Ptr features_out);
//    virtual void compute_features();
//    SF_Point_PCA compute_features_from_neighbors(typename pcl::PointCloud<PointType>::Ptr neighborhood, const PointType &p, const Eigen::Vector4f& xyz_centroid);
//    SF_Point_PCA compute_features_from_point(const PointType& p, pcl::search::KdTree<PointType>::Ptr kd_tree);
//    void set_parameters(float range, bool use_range );
//    void set_parameters(int k);
//}


#include "sf_pca.hpp"

#endif // SF_PCA_H
